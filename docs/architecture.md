# Architecture

## Phase 1, Phase 2, and Phase 4.1 boundary

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
| `lb_sensors` | `lb_sensors` | Source-scoped camera acquisition/rectification/AprilTag observations, then LiDAR filtering, ZED depth processing, and terrain hazards. |
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

Phase 4.1 does not make the webcam a physical member of the robot TF tree.
`v4l2_camera` uses `webcam_optical_frame` as an image-header frame ID but
publishes no rigid camera transform. `apriltag_ros` is the sole owner of its
temporary, dynamic observation edge:

```text
webcam_optical_frame -> webcam_observation_tag_0
```

Newly stamped samples for that edge are published only while tag 0 is
detected. A TF listener can retain the last dynamic sample after loss, so a
future consumer must reject a stale transform timestamp. The edge is not a
surveyed world tag, does not establish `base_link -> webcam_optical_frame`, and
must not publish `map -> odom` or `odom -> base_link`. The existing mock-model
`base_link` camera branch remains synthetic and cannot be used as an extrinsic.
Before physical integration, a measured webcam mount and one authoritative
rigid-frame publisher are required.

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

Phase 2 adds `lb_hardware`'s protocol-neutral `DriveTransport`, in-memory
`MockDriveTransport`, fail-closed `RealDriveTransport` skeleton, and
`LunabotDriveHardware` ros2_control system plugin. The Phase 2 bench profile
uses all four wheel joints through `diff_drive_controller`, but its radius,
track, timeout, limits, and covariance values are explicitly synthetic. The
controller publishes mock wheel odometry with its odometry TF disabled, so it
does not take `odom -> base_link` ownership from the future local EKF. The
real skeleton cannot open a device or enable output until the MCU protocol and
safety design are reviewed. See [`phase2_drive_interface.md`](phase2_drive_interface.md).

## Phase 4.1 webcam observation boundary

`lb_sensors` now owns one calibrated webcam baseline:

```text
V4L2 rate preflight -> v4l2_camera -> image_proc/rectify_node -> apriltag_ros
```

Before the driver opens the camera, the launch uses `v4l2-ctl` from `v4l-utils`
to set and verify a 15 Hz V4L2 source rate. A failed/mismatched preflight aborts
the launch. The source publishes under `/sensors/webcam`, so a later ZED or
second camera can retain its own image, calibration, detection, and
observation-frame namespace. The configured tag is `tag36h11` ID 0 with a
user-confirmed 0.250 m detector-corner edge and `max_hamming: 0`. The detector
may publish only `webcam_observation_tag_0`; a later ZED must use a distinct
child frame such as `zed_observation_tag_0` to avoid TF collisions.

This phase has no tag map, multi-tag fusion, quality/covariance gates,
`lb_localization/tag_localizer`, local EKF, global EKF, or map/odometry TF
publication. Those remain the later Phase 3/4 architecture. The detailed
camera interface, parameter provenance, bench procedure, and failure boundary
are in [`phase41_webcam_apriltag.md`](phase41_webcam_apriltag.md).
