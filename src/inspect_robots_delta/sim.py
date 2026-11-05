"""A kinematic pick-and-place world for a 3-RRR delta robot with a suction cup.

A puck lies on a table under the robot; a seeded bin marks where it must go.
Suction picks the puck up when the cup touches its top face, and dropping it
lets it fall straight to the table. Success: puck resting inside the bin with
suction off. Two policy interfaces drive the same robot:

- ``delta_sim``       4-D ``eef_delta_pos``: (dx, dy, dz) in metres + suction.
- ``delta_sim_joint`` 4-D ``joint_pos``: three motor angles in rad + suction.

Both clamp every command to the Cartesian workspace and the joint limits inside
``step()``, independent of any framework guard.
"""

from __future__ import annotations

import numpy as np

from inspect_robots.embodiment import (
        AUTO_RESET,
        PRIVILEGED_SUCCESS,
        RENDERABLE,
        RESETTABLE,
        SEEDABLE,
        EmbodimentInfo,
)
from inspect_robots.scene import Scene
from inspect_robots.spaces import (
        ActionSemantics,
        Box,
        CameraSpec,
        ObservationSpace,
        StateField,
        StateSpec,
)
from inspect_robots.types import Action, Observation, StepResult

from inspect_robots_delta.kinematics import DeltaGeometry, Unreachable, forward, inverse

WORK_RADIUS = 0.25  # m, reachable cylinder (checked by tests/test_kinematics.py)
Z_TOP = -0.56  # m, highest TCP position
TABLE_Z = -0.80  # m, table surface
PUCK_H = 0.02  # m
PUCK_R = 0.02  # m
BIN_R = 0.045  # m, success radius around the bin centre
HOME = np.array([0.0, 0.0, -0.65])
PICK_TOL_XY = 0.012  # m, cup-to-puck-centre tolerance for a grip
PICK_TOL_Z = 0.006  # m, cup above the puck top face
MAX_STEP = 0.01  # m per control step (0.5 m/s at 50 Hz)
MAX_JOINT_STEP = 0.05  # rad per control step
CONTROL_HZ = 50.0
_IMG = 96
MORPHOLOGY = "delta-3rrr-suction-sim"

_STATE = StateSpec(
        fields=(
                StateField("joint_pos", (4,), "rad"),  # theta1..3 + suction (0/1)
                StateField("eef_pos", (3,), "m"),
                StateField("puck_pos", (3,), "m"),
                StateField("bin_pos", (2,), "m"),
                StateField("holding", (1,), ""),
        )
)

_DOCS = (
        "Rotary delta robot (3 arms at 0/120/240 deg azimuth) with a suction cup, seen from "
        "above. Base frame: origin at the base plate centre, z up, so the TCP works at "
        f"negative z. Workspace: radius {WORK_RADIUS} m around the z axis, z from "
        f"{TABLE_Z + 0.0} to {Z_TOP} m; the table surface is at z={TABLE_Z} m. The puck "
        f"(green, {PUCK_H} m tall) is gripped when suction is switched on with the cup within "
        f"{PICK_TOL_XY} m of its centre and touching its top face (z = {TABLE_Z + PUCK_H} m). "
        "Switching suction off drops the puck straight down. Goal: puck inside the blue bin "
        f"ring (radius {BIN_R} m) with suction off. "
        "Camera: image right is +x, image up is +y; the red dot is the TCP and grows "
        "as it rises."
)


class _DeltaWorld:
        """Shared state and physics for both interfaces."""

        def __init__(
                self, name: str, action_space: Box, suction_doc: str, geometry: DeltaGeometry | None = None
        ):
                self.geo = geometry or DeltaGeometry()
                self.num_steps = 0
                self.clamped_steps = 0
                self._eef = HOME.copy()
                self._theta = inverse(self.geo, self._eef)
                self._suction = False
                self._holding = False
                self._puck = np.array([0.1, 0.0, TABLE_Z])
                self._bin = np.array([-0.1, 0.0])
                self.info = EmbodimentInfo(
                        name=name,
                        action_space=action_space,
                        observation_space=ObservationSpace(
                                cameras=(CameraSpec(name="top", height=_IMG, width=_IMG, channels=3),),
                                state_keys=_STATE.keys,
                                state=_STATE,
                        ),
                        control_hz=CONTROL_HZ,
                        is_simulated=True,
                        capabilities=frozenset(
                                {SEEDABLE, RESETTABLE, AUTO_RESET, PRIVILEGED_SUCCESS, RENDERABLE}
                        ),
                        docs=_DOCS + " " + suction_doc,
                        morphology=MORPHOLOGY,
                )

        # -- lifecycle -------------------------------------------------------------
        def reset(self, scene: Scene, *, seed: int | None = None) -> Observation:
                """Seed a puck and a bin at least 0.15 m apart and home the robot."""
                rng = np.random.RandomState(seed if seed is not None else 0)
                while True:
                        puck, bin_ = (self._sample_xy(rng) for _ in range(2))
                        if np.linalg.norm(puck - bin_) >= 0.15:
                                break
                self._puck = np.array([*puck, TABLE_Z])
                self._bin = bin_
                self._eef = HOME.copy()
                self._theta = inverse(self.geo, self._eef)
                self._suction = self._holding = False
                self.num_steps = self.clamped_steps = 0
                return self._observe(scene.instruction)

        def close(self) -> None:
                """Nothing to release: the world is in memory."""
                return None

        # -- physics -----------------------------------------------------------------
        def _advance(self, target: np.ndarray, suction: bool) -> bool:
                """Move to ``target`` (clamped to the workspace), update suction. True if clamped."""
                safe = self._clamp_cartesian(target)
                clamped = not np.allclose(safe, target, atol=1e-9)
                try:
                        theta = inverse(self.geo, safe)
                except Unreachable:  # pragma: no cover - the cylinder lies inside the workspace
                        return True
                self._theta, self._eef = theta, forward(self.geo, theta)
                if suction and not self._holding and self._can_grip():
                        self._holding = True
                if not suction and self._holding:
                        self._holding = False
                        self._puck[2] = TABLE_Z  # drop straight down
                self._suction = suction
                if self._holding:
                        self._puck = self._eef - np.array([0.0, 0.0, PUCK_H])
                return clamped

        def _clamp_cartesian(self, p: np.ndarray) -> np.ndarray:
                p = np.asarray(p, dtype=np.float64).copy()
                r = float(np.hypot(p[0], p[1]))
                if r > WORK_RADIUS:
                        p[:2] *= WORK_RADIUS / r
                floor = TABLE_Z + (PUCK_H if self._holding else 0.0)
                # The cup cannot pass through the puck it is not holding.
                if not self._holding and np.hypot(*(p[:2] - self._puck[:2])) < PUCK_R:
                        floor = TABLE_Z + PUCK_H
                p[2] = np.clip(p[2], floor, Z_TOP)
                return p

        def _can_grip(self) -> bool:
                top = self._puck[2] + PUCK_H
                return bool(
                        np.hypot(*(self._eef[:2] - self._puck[:2])) <= PICK_TOL_XY
                        and -1e-9 <= self._eef[2] - top <= PICK_TOL_Z
                )

        def _result(self, clamped: bool) -> StepResult:
                self.num_steps += 1
                self.clamped_steps += int(clamped)
                dist = float(np.hypot(*(self._puck[:2] - self._bin)))
                resting = not self._holding and self._puck[2] <= TABLE_Z + 1e-9
                success = bool(resting and dist <= BIN_R and not self._suction)
                return StepResult(
                        observation=self._observe(None),
                        reward=-dist,
                        terminated=success,
                        termination_reason="success" if success else None,
                        truncated=False,
                        info={
                                "success": success,
                                "distance": dist,
                                "holding": self._holding,
                                "workspace_clamped": clamped,
                        },
                )

        @staticmethod
        def _sample_xy(rng: np.random.RandomState) -> np.ndarray:
                r = 0.2 * np.sqrt(rng.uniform())
                a = rng.uniform(0, 2 * np.pi)
                return np.array([r * np.cos(a), r * np.sin(a)])

        # -- observation -------------------------------------------------------------
        def _observe(self, instruction: str | None) -> Observation:
                return Observation(
                        images={"top": self._render()},
                        state={
                                "joint_pos": np.array([*self._theta, float(self._suction)]),
                                "eef_pos": self._eef.copy(),
                                "puck_pos": self._puck.copy(),
                                "bin_pos": self._bin.copy(),
                                "holding": np.array([float(self._holding)]),
                        },
                        instruction=instruction,
                )

        def _render(self) -> np.ndarray:
                img = np.full((_IMG, _IMG, 3), 60, dtype=np.uint8)
                yy, xx = np.mgrid[0:_IMG, 0:_IMG]
                scale = (_IMG / 2 - 2) / (WORK_RADIUS + 0.03)

                def px(xy: np.ndarray) -> tuple[float, float]:
                        return _IMG / 2 + xy[0] * scale, _IMG / 2 - xy[1] * scale

                def disc(xy: np.ndarray, radius_m: float) -> np.ndarray:
                        cx, cy = px(xy)
                        return np.asarray((xx - cx) ** 2 + (yy - cy) ** 2 <= (radius_m * scale) ** 2)

                img[disc(np.zeros(2), WORK_RADIUS)] = (95, 95, 95)
                ring = disc(self._bin, BIN_R) & ~disc(self._bin, BIN_R - 0.006)
                img[ring] = (40, 90, 230)
                img[disc(self._puck[:2], PUCK_R)] = (40, 200, 60) if not self._holding else (240, 200, 40)
                height = (self._eef[2] - TABLE_Z) / (Z_TOP - TABLE_Z)
                img[disc(self._eef[:2], 0.006 + 0.012 * height)] = (230, 40, 40)
                return img


class DeltaSimEmbodiment(_DeltaWorld):
        """Cartesian interface: per-step TCP displacement plus a suction command."""

        def __init__(self, *, max_step: float = MAX_STEP):
                self.max_step = float(max_step)
                box = Box(
                        shape=(4,),
                        low=np.array([-max_step] * 3 + [-1.0]),
                        high=np.array([max_step] * 3 + [1.0]),
                        semantics=ActionSemantics(
                                control_mode="eef_delta_pos",
                                gripper="binary",
                                frame="base",
                                dim_labels=("dx", "dy", "dz", "suction"),
                        ),
                )
                super().__init__(
                        "delta_sim",
                        box,
                        "Suction dim latches like the other displacement dims: any positive value switches "
                        "it on, any negative value off, 0 (or leaving it out) keeps the current state.",
                )

        def step(self, action: Action) -> StepResult:
                """Apply one clamped displacement and the suction command."""
                a = np.asarray(action.data, dtype=np.float64)
                delta = np.clip(a[:3], -self.max_step, self.max_step)
                # Displacement semantics: 0 means "no change", so suction latches. Any
                # sign counts: an agent spreads +1 over an N-step move as N x (1/N).
                suction = True if a[3] > 1e-6 else False if a[3] < -1e-6 else self._suction
                clamped = self._advance(self._eef + delta, suction)
                return self._result(clamped or not np.allclose(delta, a[:3]))


class DeltaSimJointEmbodiment(_DeltaWorld):
        """Joint interface: absolute motor angles plus a suction command."""

        def __init__(self, *, max_joint_step: float = MAX_JOINT_STEP):
                self.max_joint_step = float(max_joint_step)
                geo = DeltaGeometry()
                box = Box(
                        shape=(4,),
                        low=np.array([geo.theta_min] * 3 + [0.0]),
                        high=np.array([geo.theta_max] * 3 + [1.0]),
                        semantics=ActionSemantics(
                                control_mode="joint_pos",
                                gripper="binary",
                                frame="base",
                                dim_labels=("theta1", "theta2", "theta3", "suction"),
                                # Suction toggles in one step; without this the default
                                # delta-limit ramps it at 5 % of its range per step.
                                max_step=(max_joint_step,) * 3 + (1.0,),
                        ),
                )
                super().__init__(
                        "delta_sim_joint",
                        box,
                        "Suction dim is an absolute target like the joints: 1 on, 0 off.",
                        geo,
                )

        def step(self, action: Action) -> StepResult:
                """Rate-limit the joint target, map it to Cartesian, clamp, and apply."""
                a = np.asarray(action.data, dtype=np.float64)
                lim = self.max_joint_step
                theta = self._theta + np.clip(a[:3] - self._theta, -lim, lim)
                theta = np.clip(theta, self.geo.theta_min, self.geo.theta_max)
                clamped = not np.allclose(theta, a[:3])
                try:
                        target = forward(self.geo, theta)
                except Unreachable:
                        target, clamped = self._eef, True
                clamped |= self._advance(target, bool(a[3] > 0.5))
                return self._result(clamped)
