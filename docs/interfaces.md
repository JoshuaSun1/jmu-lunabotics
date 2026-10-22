# Interface contract

Canonical application-boundary names are reserved now to prevent later vendor
topic leakage into the system design:

| Interface | Producer | Consumer |
|---|---|---|
| `/scan` | LiDAR driver/filter | Nav2 costmap |
| `/imu/data` | ZED wrapper/remap | Local EKF |
| `/odom/wheel` | Drive controller | Local EKF |
| `/odometry/local` | Local EKF | Global EKF, Nav2 |
| `/sensors/<camera>/tag_detections` | Camera-specific AprilTag detector | Source-scoped `tag_localizer` subscription |
| `/tag_detections` | Future normalized/aggregated AprilTag interface | Tag localizer |
| `/localization/apriltag_pose` | Tag localizer | Global EKF |
| `/localization/fused_pose` | Initial AprilTag pass-through; future global-fusion adapter | Localization monitor, mission/UI, RViz |
| `/odometry/global` | Future global EKF | Fused-pose adapter, monitoring |
| `/perception/terrain_hazards` | Terrain node | Nav2 costmap |
| `/cmd_vel_nav`, `/cmd_vel_teleop`, `/cmd_vel` | Nav2/teleop/mux | Drive controller |
| `/diagnostics` | Major subsystems | Operator/logger |
| `/map` | Nav2 `map_server` | RViz; future global costmap | Static `nav_msgs/msg/OccupancyGrid` in `map`; not a TF. |

High-rate sensor data uses sensor-data QoS unless a driver requires reliable
delivery. Commands, safety, mission, and hardware state are reliable. Exact
message adaptations and parameters are deferred to the relevant implementation
phase.

The source-scoped detection name is intentional. Phase 4.1 publishes only
`/sensors/webcam/tag_detections`; Phase 4.2 consumes configured source-scoped
topics directly. The unscoped `/tag_detections` row remains a future
application-boundary reservation and has no current publisher.

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

## Phase 4.2 known-tag-localizer interfaces

| Interface | Type | Publisher | Consumer | Boundary |
|---|---|---|---|---|
| `/sensors/webcam/tag_detections` | `apriltag_msgs/msg/AprilTagDetectionArray` | Webcam detector | `tag_localizer` | The configured source header must equal `webcam_optical_frame`; actual production use waits for a surveyed webcam extrinsic. |
| `/sensors/zed/tag_detections` | `apriltag_msgs/msg/AprilTagDetectionArray` | Future ZED detector | `tag_localizer` | Reserved source contract; no ZED pipeline exists yet. Its observation children must use `zed_observation_tag_<id>`. |
| `/tf` camera observation | `tf2_msgs/msg/TFMessage` | Camera-specific AprilTag detector | `tag_localizer` | Exact-stamp dynamic `<camera>_optical_frame -> <source>_observation_tag_<id>` transform supplies metric camera-to-tag geometry. Timestamp/frame checks reject zero, stale, and mismatched samples; target-runtime cached-TF behavior remains to be validated. |
| `/tf` camera extrinsic | `tf2_msgs/msg/TFMessage` | Future measured rigid-frame authority | `tag_localizer` | `base_link -> <camera>_optical_frame`; a static sample is accepted only after its source configuration is marked surveyed. No mock URDF extrinsic is valid. |
| `/localization/apriltag_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `tag_localizer` | Future global EKF | Fresh fused absolute pose in `map`, using covariance; it is planar yaw with explicitly large unobserved roll/pitch covariance, and the localizer publishes no TF. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | `tag_localizer` | Operator/logger | Accepted/rejected counts, reasons, cache/fusion status, and age since the last accepted correction. |

`tag_localizer` deduplicates observations by physical `(family, id)` before
fusion. Two cameras seeing tag 1 therefore do not falsely satisfy a
multi-tag requirement. It never publishes `/tag_detections`, `/odometry/global`,
`map -> odom`, or `odom -> base_link`.

## Planned fused-pose and Nav2 localization boundary

| Interface | Type | Planned publisher | Consumer | Boundary |
|---|---|---|---|---|
| `/localization/apriltag_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `tag_localizer` | Initial fused-pose pass-through; future global EKF | Absolute `map`-frame measurement and covariance; no TF ownership. |
| `/localization/fused_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Initial freshness-gated pass-through; future global-fusion adapter | Localization monitor, mission/UI, RViz, tests | Stable application-facing pose. Initially exactly preserves each accepted AprilTag pose/timestamp/covariance and emits nothing between detections; it is not yet true fusion. |
| `/localization/status` | `diagnostic_msgs/msg/DiagnosticArray` initially | Localization monitor/pass-through | Safety/operator/tests | Reports pose source, accepted/rejected/stale state, freshness, and navigation-inhibit reason. |
| `/odometry/global` | `nav_msgs/msg/Odometry` | Future global EKF | Fused-pose adapter, monitoring | Full filtered global state after continuous localization exists; it is not fabricated by the AprilTag-only pass-through. |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `bt_navigator` | Operator/RViz/future mission manager | Planned action; goals and active motion require valid localization and current TF. |
| `/cmd_vel_nav` | Nav2-compatible stamped twist | Nav2 controller chain | Future command mux/localization gate | Every nonzero command is inhibited when AprilTag localization is absent or invalid until continuous localization exists. |

No publisher or command gate in this section is implemented merely by this
documentation change. The planned pass-through publishes no TF. Nav2 motion
also requires a current, sole-owned `map -> odom -> base_link` chain; a fused
pose topic alone is insufficient. The full contract and staged implementation
are in
[`nav2_specification_and_implementation.md`](nav2_specification_and_implementation.md).

## Development bag-replay visualization interfaces

| Interface | Type | Publisher | Consumer | Boundary |
|---|---|---|---|---|
| `/visualization/webcam/tag_1_pose` | `geometry_msgs/msg/PoseStamped` | `tag_replay_visualizer` | RViz | Fresh `webcam_observation_tag_1` pose expressed in `webcam_optical_frame`; this is not a robot or map pose. |
| `/visualization/webcam/tag_1_path` | `nav_msgs/msg/Path` | `tag_replay_visualizer` | RViz | Bounded current contiguous observation segment. It is cleared on staleness and backward replay-clock jumps so tag-loss motion is never interpolated. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | `tag_replay_visualizer` | Operator/logger | Reports `LOST`, `REACQUIRED`, and `CLOCK_RESET` replay state. |

The replay launch consumes an explicit rosbag path and `/clock`. It starts no
camera, detector, hardware, map server, Nav2 component, or TF broadcaster.

### Provisional map-frame pose replay

| Interface | Type | Publisher | Consumer | Boundary |
|---|---|---|---|---|
| `/visualization/replay/webcam_pose_raw` | `geometry_msgs/msg/PoseStamped` | `tag_replay_robot_pose` | Inspection/tools | Raw six-degree-of-freedom camera pose in `map`, derived from one tag and estimated geometry; it is not a filtered robot pose. |
| `/visualization/replay/base_pose_planar` | `geometry_msgs/msg/PoseStamped` | `tag_replay_robot_pose` | Inspection/tools | Provisional `base_link` XY/yaw pose with `z=0`; no covariance, odometry propagation, or control use is permitted. |
| `/visualization/replay/base_path_planar` | `nav_msgs/msg/Path` | `tag_replay_robot_pose` | RViz | Bounded current contiguous segment; clears on tag loss, reacquisition boundary, and backward `/clock` jump. |
| `/visualization/replay/markers` | `visualization_msgs/msg/MarkerArray` | `tag_replay_robot_pose` | RViz | Assumed tag plane, raw camera RGB axes, planar robot arrow, synthetic chassis/footprint, and provisional webcam/ZED bodies and view arrows. Dynamic markers expire and are deleted on tag loss. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | `tag_replay_robot_pose` | Operator/logger | Reports `LOST`, `REACQUIRED`, and `CLOCK_RESET`, plus explicit estimated-geometry and no-odometry status. |

No pose replay topic is an input to an EKF, Nav2, or a motor path. The node
publishes no TF; therefore it cannot conflict with the future global EKF's
`map -> odom` authority or local EKF's `odom -> base_link` authority. A
`PoseStamped` subscriber may retain its last sample, so the reusable RViz
profile uses expiring/deleted markers for the visible current pose.

## Static map interfaces

| Interface | Type | Publisher | Consumer | Boundary |
|---|---|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | `nav2_map_server/map_server` | RViz; future static costmap layer | Transient-local static arena grid with `header.frame_id=map`; the map origin is grid metadata, not a transform. |
| `/map_server/load_map` | `nav2_msgs/srv/LoadMap` | `nav2_map_server/map_server` | Operator/future bringup | Standard Nav2 map replacement service; only explicitly supplied map YAML files are valid. |
| `/map_server/*` lifecycle services | `lifecycle_msgs` services | `nav2_map_server/map_server` | Lifecycle manager/operator | Standard managed-node state and transition interfaces. |
| `/visualization/arena/markers` | `visualization_msgs/msg/MarkerArray` | `arena_scene_markers` | RViz only | Transient-local, visualization-only arena outline, zone boundaries, berm target, and assumed tag poses. It is not occupancy/costmap data and publishes no column or TF. |

`lifecycle_manager_map` manages only `map_server`. Neither node publishes
`map -> odom`, `odom -> base_link`, a robot pose, a costmap, or a velocity.
