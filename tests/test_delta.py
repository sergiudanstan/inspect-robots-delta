from __future__ import annotations

import numpy as np
import pytest

from inspect_robots import eval
from inspect_robots.compat import check_compatibility
from inspect_robots.conformance import (
        assert_embodiment_conformant,
        check_evaluation_interface,
        check_policy_interfaces,
)
from inspect_robots.mock import NoopPolicy, ScriptedPolicy
from inspect_robots.scene import Scene
from inspect_robots.types import Action

from inspect_robots_delta import (
        DeltaJointScriptedPolicy,
        DeltaScriptedPolicy,
        DeltaSimEmbodiment,
        DeltaSimJointEmbodiment,
        delta_pick_place,
)
from inspect_robots_delta.kinematics import DeltaGeometry, Unreachable, forward, inverse
from inspect_robots_delta.policies import _Oracle
from inspect_robots_delta.sim import PUCK_H, TABLE_Z, WORK_RADIUS, Z_TOP

PAIRS = [
        (DeltaScriptedPolicy, DeltaSimEmbodiment),
        (DeltaJointScriptedPolicy, DeltaSimJointEmbodiment),
]
SCENE = Scene(id="t", instruction="pick", init_seed=0)


# -- kinematics ------------------------------------------------------------------
def test_fk_inverts_ik_over_the_workspace() -> None:
        geo = DeltaGeometry()
        rng = np.random.default_rng(1)
        for _ in range(3000):
                r, a = WORK_RADIUS * np.sqrt(rng.uniform()), rng.uniform(0, 2 * np.pi)
                p = np.array([r * np.cos(a), r * np.sin(a), rng.uniform(TABLE_Z, Z_TOP)])
                assert np.allclose(forward(geo, inverse(geo, p)), p, atol=1e-9)


def test_declared_cylinder_is_reachable_on_its_boundary() -> None:
        geo = DeltaGeometry()
        for z in np.linspace(TABLE_Z, Z_TOP, 14):
                for a in np.linspace(0, 2 * np.pi, 72, endpoint=False):
                        inverse(geo, np.array([WORK_RADIUS * np.cos(a), WORK_RADIUS * np.sin(a), z]))


def test_ik_rejects_unreachable_targets() -> None:
        with pytest.raises(Unreachable):
                inverse(DeltaGeometry(), np.array([0.0, 0.0, -1.2]))  # beyond full stretch
        with pytest.raises(Unreachable):
                inverse(DeltaGeometry(), np.array([0.0, 0.0, -0.30]))  # needs theta < theta_min


def test_ik_is_symmetric_on_the_axis() -> None:
        t = inverse(DeltaGeometry(), np.array([0.0, 0.0, -0.65]))
        assert np.allclose(t, t[0])


# -- framework contracts ---------------------------------------------------------
@pytest.mark.parametrize("emb", [DeltaSimEmbodiment, DeltaSimJointEmbodiment])
def test_conformance_kit(emb: type) -> None:
        e = emb()
        assert_embodiment_conformant(e.info)
        assert check_evaluation_interface(e).ok


def test_two_interfaces_on_one_morphology_are_distinct() -> None:
        assert check_policy_interfaces([DeltaSimEmbodiment().info, DeltaSimJointEmbodiment().info]).ok


@pytest.mark.parametrize(("pol", "emb"), PAIRS)
def test_matching_pairs_are_compatible(pol: type, emb: type) -> None:
        rep = check_compatibility(pol(), emb(), delta_pick_place())
        assert not [i for i in rep.issues if i.severity == "error"], rep.issues


def test_mismatched_pairs_fail_before_motion() -> None:
        codes = {i.code for i in check_compatibility(ScriptedPolicy(), DeltaSimEmbodiment()).issues}
        assert "action_dim" in codes  # 2-D CubePick oracle on a 4-D delta
        codes = {
                i.code for i in check_compatibility(DeltaScriptedPolicy(), DeltaSimJointEmbodiment()).issues
        }
        assert "control_mode" in codes  # displacements sent to an absolute-joint interface


# -- end-to-end evals ------------------------------------------------------------
@pytest.mark.parametrize(("pol", "emb"), PAIRS)
def test_oracle_solves_every_layout(pol: type, emb: type, tmp_path) -> None:
        (log,) = eval(delta_pick_place(num_scenes=20), pol(), emb(), log_dir=str(tmp_path))
        assert log.status == "success"
        assert log.results.metrics["success_at_end"] == 1.0


@pytest.mark.parametrize("emb", [DeltaSimEmbodiment, DeltaSimJointEmbodiment])
def test_wrong_bin_scores_zero(emb: type, tmp_path, monkeypatch) -> None:
        # Negative control: the same oracle aiming 0.1 m beside the bin must fail,
        # so a 1.0 above comes from the success check, not a path that always passes.
        import inspect_robots_delta.policies as pol_mod

        real = pol_mod.plan
        monkeypatch.setattr(
                pol_mod, "plan", lambda e, p, b, h, s: real(e, p, b + np.array([0.1, 0.0]), h, s)
        )
        pol = DeltaScriptedPolicy() if emb is DeltaSimEmbodiment else DeltaJointScriptedPolicy()
        (log,) = eval(delta_pick_place(num_scenes=5, max_steps=200), pol, emb(), log_dir=str(tmp_path))
        assert log.results.metrics["success_at_end"] == 0.0


def test_noop_never_succeeds(tmp_path) -> None:
        class Noop4(NoopPolicy):
                def __init__(self) -> None:
                        super().__init__()
                        self.info = DeltaScriptedPolicy().info

                def act(self, observation):  # type: ignore[no-untyped-def]
                        chunk = super().act(observation)
                        return type(chunk)(actions=[Action(data=np.zeros(4)) for _ in chunk.actions])

        (log,) = eval(delta_pick_place(num_scenes=3, max_steps=50), Noop4(), DeltaSimEmbodiment(), log_dir=str(tmp_path))
        assert log.results.metrics["success_at_end"] == 0.0


# -- safety and physics ----------------------------------------------------------
def test_hard_clamp_keeps_tcp_in_workspace() -> None:
        e = DeltaSimEmbodiment()
        e.reset(SCENE, seed=0)
        rng = np.random.default_rng(0)
        for _ in range(2000):
                a = np.r_[rng.uniform(-0.05, 0.05, 3), -1.0]  # 5x the declared per-step limit
                r = e.step(Action(data=a))
                p = r.observation.state["eef_pos"]
                assert np.hypot(p[0], p[1]) <= WORK_RADIUS + 1e-9
                assert TABLE_Z - 1e-9 <= p[2] <= Z_TOP + 1e-9
                assert np.all(np.abs(r.observation.state["joint_pos"][:3]) < 1.5)
        assert e.clamped_steps > 0


def test_joint_interface_rate_limits_large_jumps() -> None:
        e = DeltaSimJointEmbodiment()
        o = e.reset(SCENE, seed=0)
        start = o.state["joint_pos"][:3].copy()
        r = e.step(Action(data=np.array([1.3, 1.3, 1.3, 0.0])))
        assert np.allclose(r.observation.state["joint_pos"][:3] - start, e.max_joint_step)
        assert r.info["workspace_clamped"]


def test_suction_in_the_air_does_not_grip() -> None:
        e = DeltaSimEmbodiment()
        e.reset(SCENE, seed=0)
        r = e.step(Action(data=np.array([0.0, 0.0, 0.0, 1.0])))
        assert not r.info["holding"]


def test_cup_cannot_pass_through_the_puck() -> None:
        e = DeltaSimEmbodiment()
        o = e.reset(SCENE, seed=0)
        puck = o.state["puck_pos"]
        e._eef = np.array([*puck[:2], -0.70])
        for _ in range(30):
                o = e.step(Action(data=np.array([0.0, 0.0, -0.01, -1.0]))).observation
        assert o.state["eef_pos"][2] == pytest.approx(TABLE_Z + PUCK_H)


def test_oracle_base_class_is_abstract() -> None:
        with pytest.raises(NotImplementedError):
                _Oracle("x", DeltaScriptedPolicy().info.action_space, 1, 0.01)._encode(
                        np.zeros(3), np.zeros(3), False
                )


@pytest.mark.parametrize(("pol", "emb"), PAIRS)
def test_default_cli_guards_leave_oracle_actions_untouched(pol: type, emb: type, tmp_path) -> None:
        # The CLI wires clamp + delta-limit by default. An in-spec policy must pass
        # them unmodified; in particular a binary suction toggle must not be ramped.
        import json

        (log,) = eval(
                delta_pick_place(num_scenes=5), pol(), emb(), log_dir=str(tmp_path),
                guard=["clamp", "delta-limit"],
        )
        assert log.results.metrics["success_at_end"] == 1.0
        text = "".join(p.read_text() for p in tmp_path.rglob("*.json"))
        assert '"DeltaLimitGuard"' in text.replace("inspect_robots.guard.", "")  # guards really ran
        assert text.count('"modified": true') == 0


def test_cartesian_suction_latches_when_left_at_zero() -> None:
        # An LLM agent's move_by leaves unnamed displacement dims at 0; that must
        # not release a held puck.
        e = DeltaSimEmbodiment()
        o = e.reset(SCENE, seed=0)
        puck = o.state["puck_pos"]
        e._eef = np.array([*puck[:2], TABLE_Z + PUCK_H])
        assert e.step(Action(data=np.array([0.0, 0.0, 0.0, 1.0]))).info["holding"]
        for _ in range(5):
                r = e.step(Action(data=np.array([0.0, 0.0, 0.01, 0.0])))
        assert r.info["holding"]
        assert not e.step(Action(data=np.array([0.0, 0.0, 0.0, -1.0]))).info["holding"]


def test_cartesian_suction_switches_on_a_split_command() -> None:
        # move_by(suction=1) over 2 steps arrives as 0.5 + 0.5.
        e = DeltaSimEmbodiment()
        o = e.reset(SCENE, seed=0)
        e._eef = np.array([*o.state["puck_pos"][:2], TABLE_Z + PUCK_H])
        assert e.step(Action(data=np.array([0.0, 0.0, 0.0, 0.5]))).info["holding"]
