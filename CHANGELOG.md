# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-11-25

First tagged release. A simulated 3-RRR rotary delta robot (Clavel type) with a suction cup,
packaged as an inspect-robots plugin: two embodiments (`delta_sim` with Cartesian actions,
`delta_sim_joint` with joint actions), scripted pick-and-place policies for each, and the
`delta-pick-place` task.

### Added
- CI runs the test suite on Python 3.11 and 3.13 on every push and pull request.
- CI, CodeQL, OpenSSF Scorecard and a tag-triggered release workflow with build provenance and
  a CycloneDX SBOM; Dependabot for actions and pip.
- `SECURITY.md`, `CONTRIBUTING.md` and this changelog.

[Unreleased]: https://github.com/sergiudanstan/inspect-robots-delta/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/sergiudanstan/inspect-robots-delta/releases/tag/v0.1.0
