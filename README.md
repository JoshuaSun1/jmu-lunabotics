# JMU Lunabotics autonomy software

ROS 2 software for the JMU NASA Lunabotics robot. This repository is being
rebuilt around `docs/lunabotics_autonomy_software_spec.md`; the Phase 0
scaffold intentionally contains no motor, sensor, localization, Nav2, or
mission implementation.

## Current status

Phase 0 establishes the repository structure, package boundaries, version
lock, documentation, and quality tooling. Hardware-dependent values remain
explicit `TBD`s in [the TBD register](docs/tbd_register.md).

The intended robot runtime is:

| Component | Proposed lock |
|---|---|
| Compute | NVIDIA Jetson Orin Nano Developer Kit, 8 GB |
| JetPack / L4T | 6.2.2 / 36.5.0 |
| OS / ROS 2 | Ubuntu 22.04 / Humble |
| ZED Mini software | ZED SDK 5.2.3; ZED ROS 2 Wrapper `v5.2.2` |

This is a proposed baseline, not proof of target validation. See
[`docker/versions.env`](docker/versions.env) and
[the platform lock note](docs/resources/software_platform_lock.md).

## Repository layout

```text
.
├── docker/                 # arm64 Jetson container scaffold and version lock
├── docs/                   # architecture, safety, testing, TBDs, and change log
├── firmware/               # MCU placeholder; no protocol is assumed
├── lb_*/                   # ROS 2 packages, one responsibility per package
├── scripts/                # scoped bootstrap, build, test, and utility scripts
└── resources/              # living software-development record
```

## Development workflow

The runtime lock requires ROS 2 Humble. On a correctly configured Humble
environment:

```bash
./scripts/bootstrap_dev.sh --check
./scripts/build.sh
./scripts/test.sh
./scripts/lint.sh
```

These commands build into the enclosing `dev_ws/build`, `dev_ws/install`, and
`dev_ws/log` directories. Source the resulting workspace with
`source ../../install/setup.bash` when this repository is at
`dev_ws/src/jmu-lunabotics`.

The current x86_64 Ubuntu 24.04/Jazzy workstation may run structural checks
only by explicitly setting `LUNABOT_ROS_DISTRO=jazzy`; that does **not** validate
the Jetson, L4T, CUDA, ZED SDK, or arm64 container runtime.

```bash
LUNABOT_ROS_DISTRO=jazzy ./scripts/test.sh
```

Read [operations](docs/operations.md) before running commands on hardware.
The bootstrap script performs checks by default and never silently modifies the
host, ROS installation, shell profile, JetPack image, ZED SDK, or robot.

## Safety boundary

Motors must remain disabled until the later safety and hardware phases are
implemented and validated. This repository does not authorize direct motor
control, power switching, or bypassing the physical emergency stop.

## Prototype history

The pre-Phase-0 prototype is preserved at the Git tag
`prototype-before-phase0-2026-08-04` and local archive
`../old-lunabotics`. The active repository retains its original Git history
and remote.
