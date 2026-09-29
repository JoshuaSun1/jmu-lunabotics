# Phase 1 validation evidence — 2026-08-10

This record captures the available development-host evidence for the Phase 1
mock description. It is not target or physical-hardware validation.

## Environment

| Item | Value |
|---|---|
| Host | x86_64 Ubuntu 24.04 development environment |
| Installed ROS used for structural checks | ROS 2 Jazzy |
| Runtime target | arm64 Jetson / ROS 2 Humble (not validated here) |
| Physical motors and sensors | Not connected or commanded |

## Results

| Check | Result |
|---|---|
| `LUNABOT_ROS_DISTRO=jazzy ./scripts/test.sh` | Ten packages built; 53 tests passed; zero errors/failures; one expected runtime-launch skip because this host lacks Xacro and ros2_control executables. |
| Temporary non-installed Jazzy Xacro expansion | Generated `lb_mock.urdf` with 14 links and 13 joints using the default synthetic profile. |
| `check_urdf` on generated model | Parsed successfully; root `base_link`; direct branches were the four wheels, scoop proxy, ZED mount, webcam mount, and LiDAR mount. |
| `ros2 launch lb_model description.launch.py publish_joint_states:=false` | `robot_state_publisher` initialized successfully with the generated Xacro description. DDS sockets are blocked by the restricted sandbox, so no inter-process TF observation was asserted. |
| `scripts/check_tf_authority.py` | Passed the architecture/source-level contract check. |
| `./scripts/lint.sh` in unrestricted local execution | All pre-commit hooks passed, including Black. |

## Limits and required follow-up

- The host lacks installed `xacro`, `controller_manager`,
  `joint_state_broadcaster`, and `mock_components` runtime packages. The full
  `lb_sim` launch test is correctly conditional and did not run locally.
- The sandbox blocks DDS socket creation; it cannot establish live ROS graph or
  TF evidence even for the standalone robot-state publisher.
- Humble CI/container must run the headless mock-launch test, including the
  active `joint_state_broadcaster`, sole-publisher, and TF assertions.
- arm64 Jetson/Humble validation, RViz visual inspection, actual geometry,
  sensor mounts, drivetrain calibration, real transport, and safety behavior
  remain outside Phase 1 and/or require physical hardware.
