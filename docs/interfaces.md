# Interface contract

Canonical application-boundary names are reserved now to prevent later vendor
topic leakage into the system design:

| Interface | Producer | Consumer |
|---|---|---|
| `/scan` | LiDAR driver/filter | Nav2 costmap |
| `/imu/data` | ZED wrapper/remap | Local EKF |
| `/odom/wheel` | Drive controller | Local EKF |
| `/odometry/local` | Local EKF | Global EKF, Nav2 |
| `/tag_detections` | AprilTag detector | Tag localizer |
| `/localization/apriltag_pose` | Tag localizer | Global EKF |
| `/perception/terrain_hazards` | Terrain node | Nav2 costmap |
| `/cmd_vel_nav`, `/cmd_vel_teleop`, `/cmd_vel` | Nav2/teleop/mux | Drive controller |
| `/diagnostics` | Major subsystems | Operator/logger |

High-rate sensor data uses sensor-data QoS unless a driver requires reliable
delivery. Commands, safety, mission, and hardware state are reliable. Exact
message adaptations and parameters are deferred to the relevant implementation
phase.

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
