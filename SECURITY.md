# Security Policy

This is a plugin for [inspect-robots](https://github.com/robocurve/inspect-robots): a pure-Python,
numpy-based simulation of a 3-RRR delta robot, with scripted policies and one task. It registers its
components through inspect-robots entry points (`pyproject.toml`), opens no network connection, reads
no files and spawns no processes; its inputs are the actions and scene parameters that inspect-robots
passes in.

## Supported versions

| Version | Supported |
|---|---|
| 0.1.x | Current |

Single maintainer; only the latest release receives fixes.

## Reporting a vulnerability

**Please do not open a public issue for a security problem.** Report it privately:
**[Report a vulnerability](https://github.com/sergiudanstan/inspect-robots-delta/security/advisories/new)**
(Security, Advisories, Report a vulnerability).

| Stage | Target |
|---|---|
| Acknowledgement | within 3 business days |
| Triage | within 10 business days |
| Fix, or a documented decision not to fix | within 90 days |

These are best-effort targets from a single unpaid maintainer, not a contractual SLA.
There is no bug bounty; credit is offered in the advisory and changelog.

## Scope

In scope: a crash, hang or unbounded memory use in the embodiments or policies caused by crafted
actions, scene parameters or observations (for example NaN or infinite values reaching the
kinematics), and anything in this repository's workflows that could publish a release not built from `main`.

Out of scope: vulnerabilities in inspect-robots, numpy or any model or agent driving the simulation
(report upstream), and the physical fidelity of the simulation. It is a simulated robot with no
hardware interface and says nothing about a real delta robot; a wrong result is a correctness bug,
so file it as a normal issue. Installing any plugin runs its code inside your inspect-robots process,
as with every inspect-robots plugin.

## Supply chain

CI runs the tests on Python 3.11 and 3.13, `pip-audit` on the declared dependencies and on the
hash-pinned CI toolchain, CodeQL, and `zizmor` on the workflows. Dependencies are version ranges in
`pyproject.toml` and there is no lockfile, so installs are not reproducible. Actions are pinned to
full commit SHAs and kept current by Dependabot. Releases attach a source archive and a CycloneDX SBOM
of the environment resolved at release time, both with Sigstore build provenance:

```
gh attestation verify inspect-robots-delta-v0.1.0.tar.gz --repo sergiudanstan/inspect-robots-delta
```
