"""Scripted pick-and-place oracles for the delta world, one per interface.

Both read privileged state (puck and bin positions), plan the same Cartesian
path (rise to a travel height, move over the puck, descend, grip, rise, move
over the bin, descend, release) and predict their own motion forward so each
chunk is a genuine open-loop trajectory.
"""

from __future__ import annotations

import numpy as np

from inspect_robots.policy import PolicyConfig, PolicyInfo
from inspect_robots.scene import Scene
from inspect_robots.spaces import ActionSemantics, Box, ObservationSpace
from inspect_robots.types import Action, ActionChunk, Observation

from inspect_robots_delta.kinematics import DeltaGeometry, inverse
from inspect_robots_delta.sim import MAX_STEP, PUCK_H, TABLE_Z

TRAVEL_Z = -0.70  # m
DROP_CLEARANCE = 0.005  # m above the table when releasing
_OBS = ObservationSpace(state_keys=frozenset({"eef_pos", "puck_pos", "bin_pos", "holding"}))


def plan(
        eef: np.ndarray, puck: np.ndarray, bin_xy: np.ndarray, holding: bool, max_step: float
) -> tuple[np.ndarray, bool]:
        """Return the next TCP target (at most ``max_step`` away) and the suction command."""
        goal_xy = bin_xy if holding else puck[:2]
        low_z = TABLE_Z + PUCK_H + (DROP_CLEARANCE if holding else 0.0)
        off = float(np.hypot(*(goal_xy - eef[:2])))
        if off > 1e-6:
                # Rise first when low and still far; otherwise travel at TRAVEL_Z.
                target = np.array([*eef[:2], TRAVEL_Z]) if eef[2] < TRAVEL_Z - 1e-6 and off > 0.02 else (
                        np.array([*goal_xy, max(eef[2], TRAVEL_Z)])
                )
                suction = holding
        elif eef[2] > low_z + 1e-9:
                target, suction = np.array([*goal_xy, low_z]), holding
        else:
                return eef.copy(), not holding  # at the bottom: toggle suction
        d = target - eef
        n = float(np.linalg.norm(d))
        return (eef + d * min(1.0, max_step / n) if n > 0 else eef.copy()), suction


class _Oracle:
        def __init__(self, name: str, box: Box, chunk_size: int, max_step: float):
                self.chunk_size = chunk_size
                self.max_step = max_step
                self.num_inferences = 0
                self.info = PolicyInfo(name=name, action_space=box, observation_space=_OBS)
                self.config = PolicyConfig(action_horizon=chunk_size)

        def reset(self, scene: Scene) -> None:
                """Clear the inference counter."""
                self.num_inferences = 0

        def act(self, observation: Observation) -> ActionChunk:
                """Plan ``chunk_size`` steps ahead from the observed state."""
                self.num_inferences += 1
                s = observation.state
                eef = np.asarray(s["eef_pos"], dtype=np.float64).copy()
                puck = np.asarray(s["puck_pos"], dtype=np.float64).copy()
                bin_xy = np.asarray(s["bin_pos"], dtype=np.float64)
                holding = bool(s["holding"][0] > 0.5)
                actions: list[Action] = []
                for _ in range(self.chunk_size):
                        nxt, suction = plan(eef, puck, bin_xy, holding, self.max_step)
                        actions.append(Action(data=self._encode(eef, nxt, suction)))
                        toggled = suction != holding
                        eef, holding = nxt, suction
                        if holding:
                                puck = eef - np.array([0.0, 0.0, PUCK_H])
                        if toggled:
                                break  # replan from the real outcome of a grip/release
                return ActionChunk(actions=actions)

        def _encode(self, eef: np.ndarray, nxt: np.ndarray, suction: bool) -> np.ndarray:
                raise NotImplementedError


class DeltaScriptedPolicy(_Oracle):
        """Oracle for ``delta_sim``: emits (dx, dy, dz, suction)."""

        def __init__(self, *, chunk_size: int = 8, max_step: float = MAX_STEP):
                box = Box(
                        shape=(4,),
                        semantics=ActionSemantics(
                                control_mode="eef_delta_pos", gripper="binary", frame="base"
                        ),
                )
                super().__init__("delta_scripted", box, chunk_size, max_step)

        def _encode(self, eef: np.ndarray, nxt: np.ndarray, suction: bool) -> np.ndarray:
                step = np.clip(nxt - eef, -self.max_step, self.max_step)  # (eef+d)-eef can overshoot by 1 ulp
                return np.array([*step, 1.0 if suction else -1.0])


class DeltaJointScriptedPolicy(_Oracle):
        """Oracle for ``delta_sim_joint``: solves IK and emits (theta1..3, suction)."""

        def __init__(self, *, chunk_size: int = 8, max_step: float = MAX_STEP):
                self.geo = DeltaGeometry()
                box = Box(
                        shape=(4,),
                        semantics=ActionSemantics(control_mode="joint_pos", gripper="binary", frame="base"),
                )
                super().__init__("delta_joint_scripted", box, chunk_size, max_step)

        def _encode(self, eef: np.ndarray, nxt: np.ndarray, suction: bool) -> np.ndarray:
                return np.array([*inverse(self.geo, nxt), 1.0 if suction else 0.0])
