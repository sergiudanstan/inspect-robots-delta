# inspect-robots-delta

[![CI](https://github.com/sergiudanstan/inspect-robots-delta/actions/workflows/ci.yml/badge.svg)](https://github.com/sergiudanstan/inspect-robots-delta/actions/workflows/ci.yml)
[![CodeQL](https://github.com/sergiudanstan/inspect-robots-delta/actions/workflows/codeql.yml/badge.svg)](https://github.com/sergiudanstan/inspect-robots-delta/actions/workflows/codeql.yml)
[![OpenSSF Scorecard](https://api.scorecard.dev/projects/github.com/sergiudanstan/inspect-robots-delta/badge)](https://scorecard.dev/viewer/?uri=github.com/sergiudanstan/inspect-robots-delta)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

Simulated 3-RRR rotary delta robot (Clavel type) with a suction cup, packaged as an
[inspect-robots](https://github.com/robocurve/inspect-robots) plugin. Tested against
inspect-robots 1.0.0 (PyPI) and main @ 7720195.

| Component | Name | Contract |
|---|---|---|
| embodiment | `delta_sim` | 4-D `eef_delta_pos`: dx, dy, dz (±0.01 m/step) + suction |
| embodiment | `delta_sim_joint` | 4-D `joint_pos`: θ1..θ3 (rad, max 0.05 rad/step) + suction |
| policy | `delta_scripted`, `delta_joint_scripted` | privileged-state pick-and-place oracles |
| task | `delta-pick-place` | puck → bin over seeded layouts, `success_at_end` |

Geometry (`kinematics.py`): base radius 0.20 m, effector radius 0.05 m, upper arm
0.30 m, forearm 0.70 m, θ ∈ [-0.70, 1.40] rad. Usable workspace: cylinder r = 0.25 m,
z ∈ [-0.80, -0.56] m. Control rate 50 Hz. Both interfaces clamp to the workspace and
joint limits inside `step()`.

```bash
uv venv --python 3.12 && uv pip install -e . pytest
.venv/bin/python -m pytest -q
source .venv/bin/activate
inspect-robots doctor --embodiment delta_sim
inspect-robots run --task delta-pick-place --policy delta_scripted --embodiment delta_sim \
    --store-frames --no-rerun --no-prompt
inspect-robots video logs/<log>.json && inspect-robots view logs/
```

## Note for adapter authors

In `joint_pos` mode, the CLI's default `delta-limit` guard rate-limits every dim with no
declared `max_step` to 5 % of its range per step, including a `gripper="binary"` dim. An
undeclared 0/1 suction dim is ramped 0 → 0.05 → 0.10 … rather than switched. This
adapter declares `max_step=1.0` for suction;
`test_default_cli_guards_leave_oracle_actions_untouched` fails if that declaration is
removed.

## LLM agent run

The stock `inspect-robots-agent` policy drives `delta_sim` with no adapter-specific code:

```bash
uv pip install inspect-robots-agent
inspect-robots run --task delta-pick-place -T num_scenes=3 --policy agent \
    --embodiment delta_sim -P model=anthropic/claude-opus-5-5 -P wire=messages \
    -P effort=low --no-rerun --no-prompt
```

Result (2026-11-05, Claude Opus 5.5, effort low): 3/3 layouts, 6–7 `move_by` calls each.

The first agent runs exposed two contract bugs in this adapter: in `eef_delta_pos`
mode the agent sends 0 for any dim a move leaves out, and it splits a commanded
displacement over the interpolated steps (suction +1 over 2 steps arrives as
0.5 + 0.5). Suction is therefore latched by sign: positive on, negative off, 0 holds.
`test_cartesian_suction_latches_when_left_at_zero` and
`test_cartesian_suction_switches_on_a_split_command` cover both.

## Development

`uv venv --python 3.12 && uv pip install -e . pytest && .venv/bin/python -m pytest -q` runs the 24 tests. See [CONTRIBUTING.md](CONTRIBUTING.md) for branches and releases, [SECURITY.md](SECURITY.md) for reporting vulnerabilities, and [CHANGELOG.md](CHANGELOG.md) for history.

## License

MIT
