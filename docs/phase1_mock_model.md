# Phase 1 mock robot model and TF contract

## Scope and provenance

`lb_model/urdf/lb_mock.urdf.xacro` is a parameterized, executable **mock**
model. Its default input,
[`lb_model/config/mock_geometry.yaml`](../lb_model/config/mock_geometry.yaml),
has `source: synthetic_mock_only` and `physical_robot_use: prohibited`.
Every number in that profile exists only to make a visible, nondegenerate test
tree. It is not a CAD dimension, calibration value, safety limit, controller
limit, payload claim, or competition-envelope claim.

The user-provided
[`Current Lunabot Robot Design Codex Handoff`](resources/Current_Lunabot_Robot_Design_Codex_Handoff.md)
confirms the four-wheel skid/tank topology, front scoop role, planned ZED Mini,
and planned webcam use. Its wheel, scoop, arm, drivetrain, and envelope numbers
remain preliminary study inputs and are intentionally not loaded by the mock
profile. Replacing the profile requires a reviewed CAD or measured calibration
record under `GEOM-01`/`DRIVE-02`.

## Coordinate convention and frames

The mock follows REP-103: `+x` forward, `+y` left, `+z` up. `base_link` is the
synthetic chassis geometric center. That origin is a mock convention only; the
physical frame origin is still `GEOM-01`.

```text
base_link
├── front_left_wheel_link          (continuous joint)
├── front_right_wheel_link         (continuous joint)
├── rear_left_wheel_link           (continuous joint)
├── rear_right_wheel_link          (continuous joint)
├── scoop_link                     (fixed conceptual proxy)
├── zed_camera_mount_link -> zed_camera_link -> zed_camera_optical_frame
│                                      └── imu_link
├── webcam_mount_link -> webcam_link -> webcam_optical_frame
└── lidar_mount_link               (generic future mount, not a selected LiDAR)
```

`robot_state_publisher` is the sole owner of this subtree. It receives the
wheel joints from `/joint_states`; the fixed mount and optical frames are also
published by it. No `map`, `odom`, or `base_footprint` is present in Phase 1.
The global and local EKFs remain the future sole owners of `map -> odom` and
`odom -> base_link`, respectively.

The mock owns camera optical frames only because no camera wrapper runs. A real
ZED or webcam integration must choose exactly one owner for internal/optical
transforms and document the change before both URDF and driver frames exist.

## Mock control and interfaces

`lb_sim/launch/mock_robot.launch.py` expands one Xacro description and passes
the same result to `robot_state_publisher` and `controller_manager`.
`controller_manager` loads upstream `mock_components/GenericSystem`; it is not
a custom `lb_hardware` plugin. `joint_state_broadcaster` is the sole
`/joint_states` publisher. The only live application-visible Phase 1 interfaces
are:

| Name | Type | Producer | Consumer | Safety boundary |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_broadcaster` | `robot_state_publisher` | Synthetic wheel state only. |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` | RViz/test tools | `base_link` subtree only. |

Each wheel declares velocity command and position/velocity state interfaces so
the mock can exercise ros2_control wiring. No controller claims command
interfaces in Phase 1. There is no `diff_drive_controller`, `/cmd_vel`,
`/odom/wheel`, real motor transport, actuator linkage, or power-enable path.
Therefore the mock cannot command a physical robot.

`scoop_link` is deliberately fixed. The handoff describes unresolved lift and
dump concepts, not a validated pivot, axis, stroke, linkage, or limits; none is
inferred here.

## Running and verifying

On a Humble development environment after building and sourcing the workspace:

```bash
ros2 launch lb_sim mock_robot.launch.py use_rviz:=true
```

Keep RViz opt-in; headless CI and the competition Jetson use the default
`use_rviz:=false`. In another sourced terminal, verify the launched graph:

```bash
scripts/check_tf_authority.py --runtime
```

The automated launch test waits for an active `joint_state_broadcaster`, checks
the required transforms, confirms that `robot_state_publisher` is the sole
`/tf` and `/tf_static` publisher and the broadcaster the sole `/joint_states`
publisher, and rejects `map`/`odom` in the mock frame graph. It does not
validate physical geometry, camera calibration, drivetrain behavior, odometry,
or safety behavior.

## Dependencies and remaining work

The mock requires `xacro`, `robot_state_publisher`, `joint_state_publisher`
(for standalone description viewing), `ros2_control`, `ros2_controllers`, and
launch-testing packages. CI and the generic container declare the Humble
packages. The available Jazzy host lacks the control/Xacro executables, so
runtime launch validation remains a Humble CI/container or target task.

Remaining blockers include `GEOM-01`, `DRIVE-02`, `DRIVE-03`, `ZED-01`,
`LIDAR-01`, `TAG-01`, `MISS-01`, and all safety configuration. Phase 2 will
add the transport abstraction and drive-controller configuration only after the
related evidence is available.
