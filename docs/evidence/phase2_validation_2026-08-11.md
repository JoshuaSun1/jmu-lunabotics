# Phase 2 validation evidence — 2026-08-11

## Scope

This record covers the mock-first drive transport, custom ros2_control plugin
source, four-wheel mock controller configuration, and bench-launch contracts.
It does not validate a motor controller, MCU, encoder, physical E-stop, CAN,
serial, USB-to-CAN, battery, or a physical robot.

## Environment

| Item | Value |
|---|---|
| Development host | x86_64 Ubuntu 24.04 environment |
| Available ROS underlay | ROS 2 Jazzy structural-check override |
| Target runtime | Still Humble/arm64 Jetson lock; not available for this validation |
| Host control packages | `hardware_interface`, `controller_manager`, Xacro, and controller executables unavailable |

## Commands and results

| Command | Result |
|---|---|
| `python3 -m py_compile lb_hardware/launch/hardware.launch.py lb_launch/launch/bench_test.launch.py lb_launch/launch/bringup.launch.py lb_hardware/test/test_phase2_contract.py lb_launch/test/test_phase2_launch_contract.py` | Passed. |
| `cmake -S lb_hardware -B /tmp/lb_phase2_cmake -DBUILD_TESTING=ON` | Passed; correctly reported that the ROS-control plugin cannot build on this host. |
| `cmake --build /tmp/lb_phase2_cmake --parallel 2 && ctest --test-dir /tmp/lb_phase2_cmake --output-on-failure` | Passed: mock transport unit test, Phase 2 static contract, CMake lint, XML lint. |
| `LUNABOT_ROS_DISTRO=jazzy ./scripts/test.sh` | Passed: 10 packages, 63 tests, 0 errors, 0 failures, 1 expected skip. |
| `colcon test-result --test-result-base /home/jsun3/dev_ws/build --all` | Confirmed the 63/0/0/1 result above. |
| `./scripts/lint.sh` | Passed in unrestricted execution: whitespace, YAML, Black, and tracked-file formatting checks all passed. |

The expected skip is the existing Xacro-dependent model expansion test. The
host has no Xacro or ros2_control runtime components, so it also cannot load
`LunabotDriveHardware`, spawn `drive_controller`, exercise `/cmd_vel`, or
prove the controller-manager remaps. The package CMake deliberately compiles
the ROS-independent transport on this host and compiles the custom plugin only
when target ROS-control dependencies are installed.

The restricted sandbox blocks Black's worker-process behavior; the full lint
gate was therefore rerun outside that sandbox and passed.

## Verified behavior

- The mock transport starts disabled, integrates explicit mock wheel commands
  in rad/rad/s, zeros output on `stop()`, reports a timeout, and injects a
  communication-loss failure deterministically.
- The real skeleton has no device I/O and rejects connect, enable, command, and
  state-read operations with a protocol-unavailable/communication fault.
- Static contracts verify four wheel groups, synthetic-only geometry and
  limits, stamped command configuration, `enable_odom_tf: false`, the custom
  plugin selection, disabled bench defaults, and the common stop/disable error
  path.

## Pending target validation

1. Build the custom plugin under the locked Humble/arm64 image.
2. Launch `lb_launch bench_test.launch.py` with mock hardware and verify
   controller activation, `/joint_states`, mock wheel response, `/odom/wheel`,
   command timeout, and absence of `odom -> base_link` TF.
3. Verify that the declared Humble in-process controller remaps expose
   `/cmd_vel` and `/odom/wheel`; add an adapter if they do not.
4. Do not test real output until `DRIVE-02`, `DRIVE-03`, `SAFE-01`, and the
   physical bench prerequisites are closed.
