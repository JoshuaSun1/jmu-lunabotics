# Architecture

## Phase 0 boundary

This document records the target architecture but does not implement it. Each
future phase must preserve the ownership rules below rather than introduce
parallel publishers or hardware-specific constants.

## Package ownership

| Package | Sole responsibility |
|---|---|
| `lunabot_interfaces` | Custom ROS actions, messages, and services only. |
| `lunabot_description` | URDF/Xacro, meshes, ros2_control tags, static robot frames. |
| `lunabot_hardware` | ros2_control hardware plugin and drive transport abstraction. |
| `lunabot_localization` | EKF configuration, tag map/localizer, localization health. |
| `lunabot_perception` | LiDAR filtering, ZED depth processing, terrain hazards. |
| `lunabot_navigation` | Maps, Nav2 configuration, costmaps, navigation behavior trees. |
| `lunabot_mission` | High-level mission state machine and mechanism clients. |
| `lunabot_safety` | Motion locks, safety supervision, fault reporting. |
| `lunabot_bringup` | System launch files and environment profiles. |
| `lunabot_sim` | Mock hardware, mock sensors, test worlds, and bag replay. |

## Coordinate-frame ownership

```text
map -> odom -> base_link -> sensor and wheel frames
```

| Transform | Sole publisher |
|---|---|
| `map -> odom` | Global `robot_localization` EKF |
| `odom -> base_link` | Local `robot_localization` EKF |
| `base_link -> rigid sensor/wheel frames` | `robot_state_publisher` |

No other node may publish those transforms during normal operation. Camera
optical-frame ownership will be selected during integration and recorded as a
decision; it must not be duplicated by the ZED wrapper and URDF.

## Control boundary

The eventual command path is Nav2/teleoperation -> command multiplexer ->
drive controller -> ros2_control hardware plugin -> microcontroller -> motor
controllers. The hardware emergency stop remains independent of ROS. Exact
communications and drivetrain details are intentionally not assumed; see
[the TBD register](tbd_register.md).

## Chassis decision and Phase 1 boundary

The planned chassis has four driven wheels and a large scooping bucket. The
six motors listed in the BOM must not be interpreted as six drive motors; the
non-drive motors are planned for the digging arm. The steering/kinematic model
is still `TBD`: a bulldozer-like appearance is not sufficient evidence to
choose skid-steer over a steering linkage.

Phase 1 may use a clearly labelled synthetic geometry to exercise the four
wheel and bucket frame tree. It must keep all physical dimensions, mounting
transforms, footprint, and limits as unmeasured parameters rather than treating
mock values as the robot design. The formal decision record is
[`0001_four_wheel_chassis.md`](decisions/0001_four_wheel_chassis.md).
