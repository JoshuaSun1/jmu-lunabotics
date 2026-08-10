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
