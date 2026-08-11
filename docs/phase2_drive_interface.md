# Phase 2 drive interface

Phase 2 implements a **mock-first** `ros2_control` boundary for the confirmed
four-wheel skid-steer topology. It does not implement a motor-controller
protocol, a physical enable path, encoder calibration, a microcontroller, or
any physical safety mechanism.

## Ownership and components

| Component | Package/files | Purpose |
|---|---|---|
| Transport contract | `lb_hardware/include/lb_hardware/drive_transport.hpp` | Protocol-neutral boundary between ROS control and a future MCU. |
| Mock transport | `mock_drive_transport.*` | In-memory four-wheel state/command simulation and deterministic fault injection. |
| Real transport skeleton | `real_drive_transport.*` | Explicitly unavailable placeholder; opens no CAN, serial, USB, or controller device. |
| Hardware plugin | `LunabotDriveHardware` | Humble `hardware_interface::SystemInterface` plugin exported as `lb_hardware/LunabotDriveHardware`. |
| Mock controller profile | `config/mock_drive_controllers.yaml` | Four-wheel `diff_drive_controller` configuration using synthetic geometry. |
| Hardware launch | `launch/hardware.launch.py` | Starts robot-state publisher, controller manager, joint-state broadcaster, and drive controller. |
| Bench entry point | `lb_launch/launch/bench_test.launch.py` | Safe public launch wrapper for the mock profile. |

## Transport contract and units

`DriveTransport` intentionally has no vendor or electrical assumptions. Its
required methods are `connect()`, `disconnect()`, `read_state()`,
`write_command()`, `set_enabled()`, `stop()`, and `get_faults()`.

The ROS/transport boundary is always SI:

| Value | Unit | Meaning |
|---|---:|---|
| Wheel position | rad | Encoder-derived wheel-joint position. |
| Wheel velocity command/state | rad/s | Wheel-joint angular velocity, not motor-phase control or controller-native units. |
| Body linear command | m/s | `TwistStamped.twist.linear.x`. |
| Body yaw command | rad/s | `TwistStamped.twist.angular.z`. |

There are four wheel slots at this boundary, ordered by the Xacro joint list:
front-left, front-right, rear-left, rear-right. This is a software model of
four wheel joints, not a statement that the final drivetrain uses four
independent controller channels. The future MCU mapping may be two or four
channels only after `DRIVE-03` closes.

## Mock and real behavior

`MockDriveTransport` begins disconnected and disabled. After `connect()` it
still returns zero output until the mock-only enable flag is explicitly set.
When enabled, it integrates commanded wheel velocity into position. It can
inject communication loss after a configurable number of reads and stops output
when its mock command age exceeds the configured timeout.

`RealDriveTransport` is intentionally nonfunctional and fail-closed. Every
connection, command, enable, state-read, and stop request reports failure. It
contains no CAN bitrate/IDs, serial device, framing, SPARK MAX mode, ODrive
mapping, encoder scale, gear ratio, heartbeat packet, reset behavior, or
physical pinout. Those choices remain `DRIVE-03`; physical enable/reset and
E-stop behavior remain `SAFE-01`.

`LunabotDriveHardware` validates exactly four velocity-commanded joints with
position and velocity state interfaces. On activation it zeros commands,
connects the selected transport, calls `stop()`, and only then applies its
explicit `enable_on_activate` setting. On a failed state read, write, or
non-finite command, it zeros interfaces, calls `stop()` and `set_enabled(false)`,
and returns a ROS-control error. This is software fail-closed behavior only;
it is not a physical E-stop or a substitute for an MCU heartbeat.

## Controller and ROS contract

The mock controller groups `front_left_wheel_joint` and
`rear_left_wheel_joint` on the left, and their right counterparts on the
right. It is configured with `position_feedback: true`, `open_loop: false`, and
`enable_odom_tf: false`. Thus, `diff_drive_controller` may produce mock wheel
odometry but it must never publish `odom -> base_link`; that remains reserved
for the Phase 3 local EKF.

| Interface | Type | Phase 2 producer/consumer | Status |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Bench remap to `drive_controller` native `~/cmd_vel` | Declared; Humble runtime remap validation pending. |
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_broadcaster` to `robot_state_publisher` | Mock-capable. |
| `/odom/wheel` | `nav_msgs/msg/Odometry` | Bench remap of controller native `~/odom` | Declared mock output; runtime validation pending. |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` for the `base_link` subtree | Existing Phase 1 ownership unchanged. |

Humble's controller uses stamped commands at `~/cmd_vel`; the launch declares
remaps from `/drive_controller/cmd_vel` to `/cmd_vel` and from
`/drive_controller/odom` to `/odom/wheel`. Because Humble loads controllers
in the controller-manager process, those remaps require target runtime
verification before they are treated as proven application-boundary behavior.

## Parameter provenance

All values in `mock_drive_controllers.yaml` are mock-only:

| Parameter group | Value/source | Status |
|---|---|---|
| Wheel radius and separation | 0.12 m and 0.68 m, derived only from `mock_geometry.yaml` | `synthetic_mock_only`; prohibited for physical use. |
| Wheel groups | Four confirmed mock wheel joint names | Structural model decision; final motor-channel mapping remains `DRIVE-03`. |
| Update/publish rate | 50/30 Hz | Synthetic visualization/test cadence. |
| Command timeout | 0.25 s | Mock stale-command exercise; not a reviewed safety threshold. |
| Velocity, acceleration, jerk limits | Values in mock controller YAML | Synthetic test limits; no physical authority. |
| Odom covariance | All zero | Mock-only; `LOC-01` must supply measured values. |
| Mock fault injection | `mock_communication_loss_after_reads`, default `-1` | Test-only; `0` fails the first state read. |
| Mock transport timeout | `mock_command_timeout_s`, default 0.25 s | Test-only heartbeat exercise. |

`DRIVE-02`, `DRIVE-03`, `SAFE-01`, and `GEOM-01` remain open. Do not replace
these values with preliminary handoff estimates or use this profile with a
physical robot.

## Safe mock bench use

After building in a Humble environment with ros2_control installed, start the
disabled mock bench with:

```bash
ros2 launch lb_launch bench_test.launch.py
```

The launch is headless and has zero output by default. Setting
`enable_motors:=true` permits only in-memory mock motion; it does not enable
propulsion. `use_mock_hardware:=false` selects the unavailable real skeleton,
which should fail closed rather than connect to anything.

Before any physical drive test, complete the ordered checks in
[`testing.md`](testing.md), including electrical/safety review, a secured motor
bench test, encoder sign validation, command-timeout test, and independent
physical E-stop test.
