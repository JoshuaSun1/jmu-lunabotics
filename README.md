# JMU Lunabotics autonomy software

ROS 2 software for the JMU NASA Lunabotics robot. This repository is being
rebuilt around `docs/lunabotics_autonomy_software_spec.md`. Phase 2 provides a
headless, mock-first description/drive stack; it contains no real motor,
sensor, localization, Nav2, mission, or physical safety implementation. Phase
4.1 adds only a non-actuating webcam AprilTag-observation bench pipeline; it
does not produce a robot or global pose.

## Current status

Phase 1 adds a parameterized Xacro, four-wheel skid-steer mock frame tree,
upstream `ros2_control` `GenericSystem`, joint-state broadcaster, and
`robot_state_publisher`. Phase 2 adds a protocol-neutral drive transport,
in-memory mock, fail-closed real skeleton, custom ros2_control hardware
plugin, four-wheel `diff_drive_controller` mock configuration, and a bench
launch. The geometry and controller profile are deliberately synthetic rather
than claims about the physical robot. The exact boundaries are in the
[Phase 1 model note](docs/phase1_mock_model.md) and
[Phase 2 drive-interface note](docs/phase2_drive_interface.md).

Phase 4.1 adds a calibrated Logitech webcam pipeline in `lb_sensors`:
`v4l2-ctl` fail-closed 15 Hz source-rate preflight, `v4l2_camera`, `image_proc`
rectification, and `apriltag_ros`. It publishes only source-scoped webcam data
and newly stamped dynamic `webcam_optical_frame -> webcam_observation_tag_0`
samples while the configured tag is visible. A future consumer must reject a
cached stale TF sample after tag loss. The phase has no physical base-to-camera
extrinsic, tag map, localizer, EKF, `map`/`odom` transform, or robot-pose
output. See the [Phase 4.1 webcam AprilTag note](docs/phase41_webcam_apriltag.md).

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

For the non-actuating Phase 4.1 webcam bench, use a stable camera path rather
than an unstable `/dev/videoN` number:

```bash
source ../../install/setup.bash
ros2 launch lb_sensors webcam_apriltag.launch.py
```

The launch must first set and verify the 15 Hz V4L2 source rate. With the
user-confirmed `tag36h11` ID 0 tag visible, inspect
`/sensors/webcam/tag_detections` and the dynamic
`webcam_optical_frame -> webcam_observation_tag_0` transform. This is a
camera-relative observation only; do not use it as robot/global localization.

On a Humble development environment with the Phase 1 dependencies installed,
inspect the mock TF tree in RViz without any motor hardware:

```bash
source ../../install/setup.bash
ros2 launch lb_sim mock_robot.launch.py use_rviz:=true
# In a separate terminal that has sourced the same workspace:
scripts/check_tf_authority.py --runtime
```

`lb_launch bringup.launch.py` and `lb_launch bench_test.launch.py` default to
disabled mock output. Setting `enable_motors:=true` enables only the in-memory
mock profile; the real skeleton fails closed and cannot open a hardware device.

Read [operations](docs/operations.md) before running commands on hardware.
The bootstrap script performs checks by default and never silently modifies the
host, ROS installation, shell profile, JetPack image, ZED SDK, or robot.

## Safety boundary

Motors must remain disabled until the later hardware and safety phases are
implemented and validated. The Phase 2 mock can consume a synthetic command
and publish mock wheel odometry, but has no real transport, power switching,
or physical hardware-enable path. This repository does not authorize bypassing
the physical emergency stop. Phase 4.1 camera detection has no command,
localization, or actuation path and cannot authorize motion.

## Prototype history

The pre-Phase-0 prototype is preserved at the Git tag
`prototype-before-phase0-2026-08-04` and local archive
`../old-lunabotics`. The active repository retains its original Git history
and remote.
