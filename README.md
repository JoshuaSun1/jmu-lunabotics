# JMU Lunabotics autonomy software

ROS 2 software for the JMU NASA Lunabotics robot. This repository is being
rebuilt around `docs/lunabotics_autonomy_software_spec.md`. Phase 1 provides a
headless, mock-only description and TF stack; it contains no real motor,
sensor, localization, Nav2, mission, or safety implementation.

## Current status

Phase 1 adds a parameterized Xacro, four-wheel skid-steer mock frame tree,
upstream `ros2_control` `GenericSystem`, joint-state broadcaster, and
`robot_state_publisher`. The geometry profile is deliberately synthetic rather
than a claim about the physical robot; all measured dimensions and mounting
transforms remain explicit `TBD`s in [the TBD register](docs/tbd_register.md).
The exact contract is in [the Phase 1 mock-model note](docs/phase1_mock_model.md).

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

On a Humble development environment with the Phase 1 dependencies installed,
inspect the mock TF tree in RViz without any motor hardware:

```bash
source ../../install/setup.bash
ros2 launch lb_sim mock_robot.launch.py use_rviz:=true
# In a separate terminal that has sourced the same workspace:
scripts/check_tf_authority.py --runtime
```

`lb_launch bringup.launch.py` defaults to the same mock hardware and keeps
`enable_motors:=false`; setting it true is intentionally inert in Phase 1.

Read [operations](docs/operations.md) before running commands on hardware.
The bootstrap script performs checks by default and never silently modifies the
host, ROS installation, shell profile, JetPack image, ZED SDK, or robot.

## Safety boundary

Motors must remain disabled until the later hardware and safety phases are
implemented and validated. The Phase 1 mock has no `/cmd_vel` consumer,
odometry publisher, real transport, power switching, or hardware enable path.
This repository does not authorize bypassing the physical emergency stop.

## Prototype history

The pre-Phase-0 prototype is preserved at the Git tag
`prototype-before-phase0-2026-08-04` and local archive
`../old-lunabotics`. The active repository retains its original Git history
and remote.
