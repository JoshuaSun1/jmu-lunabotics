# Interface contract

Canonical application-boundary names are reserved now to prevent later vendor
topic leakage into the system design:

| Interface | Producer | Consumer |
|---|---|---|
| `/scan` | LiDAR driver/filter | Nav2 costmap |
| `/imu/data` | ZED wrapper/remap | Local EKF |
| `/odom/wheel` | Drive controller | Local EKF |
| `/odometry/local` | Local EKF | Global EKF, Nav2 |
| `/sensors/<camera>/tag_detections` | Camera-specific AprilTag detector | Future source adapter / tag localizer |
| `/tag_detections` | Future normalized/aggregated AprilTag interface | Tag localizer |
| `/localization/apriltag_pose` | Tag localizer | Global EKF |
| `/perception/terrain_hazards` | Terrain node | Nav2 costmap |
| `/cmd_vel_nav`, `/cmd_vel_teleop`, `/cmd_vel` | Nav2/teleop/mux | Drive controller |
| `/diagnostics` | Major subsystems | Operator/logger |

High-rate sensor data uses sensor-data QoS unless a driver requires reliable
delivery. Commands, safety, mission, and hardware state are reliable. Exact
message adaptations and parameters are deferred to the relevant implementation
phase.

The source-scoped detection name is intentional. Phase 4.1 publishes only
`/sensors/webcam/tag_detections`; the unscoped `/tag_detections` row remains a
future application-boundary reservation and has no Phase 4.1 publisher.

## Phase 1 mock interfaces

| Interface | Type | Publisher | Consumer | Boundary |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_broadcaster` over `mock_components/GenericSystem` | `robot_state_publisher` | Mock wheel position/velocity state only; no command controller. |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` | TF consumers/RViz | Owns the Phase 1 `base_link` subtree only; no `map` or `odom`. |

No Phase 1 node publishes or consumes `/cmd_vel`, `/odom/wheel`, sensor data,
or mechanism commands. The mock command interfaces are unclaimed and cannot
move a physical robot.

## Phase 2 drive interfaces

| Interface | Type | Publisher | Consumer | Boundary |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Bench/manual test source; later command mux | `drive_controller` through declared launch remap | The controller accepts linear X in m/s and angular Z in rad/s. Humble remap validation is pending. |
| `front_*` / `rear_*` wheel velocity interfaces | ros2_control internal interface, rad/s | `drive_controller` | `LunabotDriveHardware` | Four mock wheel-joint commands; not a public ROS topic or a claim about final motor-channel count. |
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_broadcaster` | `robot_state_publisher` | Position in rad and velocity in rad/s from mock transport. |
| `/odom/wheel` | `nav_msgs/msg/Odometry` | `drive_controller` through declared launch remap | Future local EKF | Mock feedback-derived odometry only; `enable_odom_tf: false` preserves Phase 3 TF ownership. |

No Phase 2 custom diagnostics topic is published yet. Transport fault state is
internal to the hardware plugin and returned as a ROS-control error. Phase 9
will expose the full `/diagnostics` and safety contract.

## Phase 4.1 webcam AprilTag interfaces

| Interface | Type | Publisher | Consumer | Boundary |
|---|---|---|---|---|
| `/sensors/webcam/image_raw` | `sensor_msgs/msg/Image` | `v4l2_camera` | `image_proc/rectify_node` | 640 x 480 RGB capture after a verified 15 Hz V4L2 source-rate preflight; header frame is `webcam_optical_frame`. |
| `/sensors/webcam/camera_info` | `sensor_msgs/msg/CameraInfo` | `v4l2_camera` | `image_proc/rectify_node`, `apriltag_ros` | User-confirmed archived calibration; must match the image timestamp/profile. |
| `/sensors/webcam/image_rect` | `sensor_msgs/msg/Image` | `image_proc/rectify_node` | `apriltag_ros` | Rectified camera image; no robot-pose interpretation. |
| `/sensors/webcam/tag_detections` | `apriltag_msgs/msg/AprilTagDetectionArray` | `apriltag_ros` | Bench tools; future source adapter/localizer | Family, ID, hamming, decision margin, centre/corners, and homography. It is not a map-frame or robot-pose message. |
| `/tf` observation edge | `tf2_msgs/msg/TFMessage` | `apriltag_ros` | TF consumers / future localizer | Newly stamped dynamic `webcam_optical_frame -> webcam_observation_tag_0` samples while visible; TF consumers must reject cached stale samples. No `map`/`odom` ownership. |

The current `apriltag_msgs` detection array has no metric-pose member. Metric
camera-to-tag information is exposed through the configured dynamic observation
TF. No Phase 4.1 node publishes `/tag_detections`,
`/localization/apriltag_pose`, `/odometry/local`, `/odometry/global`,
`map -> odom`, or `odom -> base_link`. A later ZED/second camera must publish
source-scoped names and a unique child frame (for example,
`zed_observation_tag_0`) instead of reusing the webcam observation frame.
