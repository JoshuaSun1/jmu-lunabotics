# Architecture

## Phase 1 boundary

This document records the target architecture and the implemented Phase 1
mock-description boundary. Each future phase must preserve the ownership rules
below rather than introduce parallel publishers or hardware-specific constants.

## Package ownership

| Source directory | ROS package | Sole responsibility |
|---|---|---|
| `lb_interfaces` | `lb_interfaces` | Custom ROS actions, messages, and services only. |
| `lb_model` | `lb_model` | URDF/Xacro, mock geometry profile, ros2_control tags, robot-state-publisher launch. |
| `lb_hardware` | `lb_hardware` | ros2_control hardware plugin and drive transport abstraction. |
| `lb_localization` | `lb_localization` | EKF configuration, tag map/localizer, localization health. |
| `lb_sensors` | `lb_sensors` | LiDAR filtering, ZED depth processing, terrain hazards. |
| `lb_navigation` | `lb_navigation` | Maps, Nav2 configuration, costmaps, navigation behavior trees. |
| `lb_state_manager` | `lb_state_manager` | High-level mission state machine and mechanism clients. |
| `lb_safety` | `lb_safety` | Motion locks, safety supervision, fault reporting. |
| `lb_launch` | `lb_launch` | System launch files and environment profiles. |
| `lb_sim` | `lb_sim` | Mock hardware, mock sensors, test worlds, bag replay, and Phase 1 mock launch. |

## Coordinate-frame ownership

```text
map -> odom -> base_link -> sensor and wheel frames
```

| Transform | Sole publisher |
|---|---|
| `map -> odom` | Global `robot_localization` EKF |
| `odom -> base_link` | Local `robot_localization` EKF |
| `base_link -> fixed sensor and mount frames` | `robot_state_publisher` |
| `base_link -> wheel and mechanism joint frames` | `robot_state_publisher` consuming `/joint_states` |

No other node may publish those transforms during normal operation. Camera
optical-frame ownership will be selected during integration and recorded as a
decision; it must not be duplicated by the ZED wrapper and URDF. The Phase 1
mock has no camera wrapper, so `robot_state_publisher` publishes mock optical
frames solely for the visual/test profile. That ownership must be switched or
the frames removed before a real driver publishes the same transforms.

## Control boundary

The eventual command path is Nav2/teleoperation -> command multiplexer ->
drive controller -> ros2_control hardware plugin -> microcontroller -> motor
controllers. The hardware emergency stop remains independent of ROS. Exact
communications and drivetrain details are intentionally not assumed; see
[the TBD register](tbd_register.md).

## Chassis decision and Phase 1 implementation

The planned chassis has four driven wheels and a large scooping bucket. The
six motors listed in the BOM must not be interpreted as six drive motors; the
non-drive motors are planned for the digging arm. Team confirmation and the
design handoff establish four-wheel skid-steer/tank steering; the model has
four continuous wheel joints and no steering joints.

Phase 1 uses a clearly labelled synthetic geometry profile to exercise the
four-wheel, conceptual-scoop, and sensor-mount frame tree. Its upstream
`mock_components/GenericSystem` and `joint_state_broadcaster` are the sole
`/joint_states` path; `robot_state_publisher` is the sole `/tf` and
`/tf_static` path. No `diff_drive_controller`, `/cmd_vel`, `/odom/wheel`,
`map`, or `odom` is introduced. Physical dimensions, mounting transforms,
footprint, actuator linkage, and limits remain unmeasured. See
[`phase1_mock_model.md`](phase1_mock_model.md) and the formal decision record
[`0001_four_wheel_chassis.md`](decisions/0001_four_wheel_chassis.md).
