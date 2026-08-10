# Lunabotics Autonomy Software Specification

**Document ID:** LUNA-SW-SPEC-001
**Version:** 0.1 (implementation handoff draft)
**Date:** 2026-08-04
**Target platform assumption:** NVIDIA Jetson Orin Nano 8 GB
**Project:** JMU NASA Lunabotics Capstone

## 1. Purpose

This document defines the software architecture, interfaces, responsibilities, repository structure, implementation sequence, and acceptance criteria for the Lunabotics robot autonomy stack. It is written to be handed to Codex or another developer as the primary implementation specification.

The system shall:

- Navigate in a known arena without live SLAM.
- Load a preconstructed static occupancy-grid map.
- Estimate robot pose using wheel odometry, the ZED IMU, and AprilTag observations.
- Use a two-filter `robot_localization` architecture:
  - A local EKF owns `odom -> base_link`.
  - A global EKF owns `map -> odom`.
- Detect obstacles using a 2D LiDAR and ZED depth data.
- Represent detected hazards in Nav2 costmaps so unsafe cells are excluded from planning.
- Use Nav2 for path planning, path following, obstacle avoidance, and navigation recovery.
- Use a custom mission state machine for Lunabotics task sequencing.
- Provide teleoperation, independent safety interlocks, diagnostics, and data recording.

## 2. Scope

### 2.1 In scope

- ROS 2 workspace and package architecture.
- Robot description and TF tree.
- Drive hardware abstraction and wheel odometry.
- ZED camera and IMU integration.
- AprilTag detection and known-tag global localization.
- Local and global EKF configuration.
- Static arena occupancy-grid loading.
- 2D LiDAR obstacle integration.
- Depth-based raised-obstacle and negative-obstacle detection.
- Nav2 configuration.
- Mission state machine.
- Teleoperation and command arbitration.
- Safety supervisor and command watchdogs.
- Diagnostics, logging, rosbag replay, and performance monitoring.
- Simulation/fake-hardware support sufficient for software development without the completed robot.

### 2.2 Out of scope for the initial implementation

- Live 2D or 3D SLAM.
- Persistent 3D world reconstruction.
- Neural-network object classification.
- Semantic terrain classification.
- Multi-robot coordination.
- Autonomous tag-map generation.
- Direct Jetson control of motor phase wires or motor-controller power electronics.
- Competition mechanism details that have not yet been specified, such as exact excavation actuator hardware.

## 3. System assumptions and unresolved inputs

The implementation shall not invent values for the following items. Each shall be represented by a named parameter and documented as `TBD` until measured or selected.

| Item | Required value |
|---|---|
| Drive geometry | Differential drive or skid-steer confirmation |
| Wheel radius | meters |
| Effective wheel separation | meters; must be tuned for skid-steer turning |
| Gear ratio | motor-to-wheel ratio |
| Encoder resolution | counts or rotations per wheel revolution |
| Motor communication | CAN, serial, PWM, or another microcontroller interface |
| Microcontroller protocol | packet format, units, heartbeat, fault response |
| Robot footprint | polygon in `base_link` coordinates |
| Maximum linear/angular velocity | safe tested values |
| ZED mounting transform | translation and rotation from `base_link` |
| LiDAR mounting transform | translation and rotation from `base_link` |
| AprilTag camera | ZED RGB camera or separate Logitech camera |
| AprilTag sizes | meters by tag ID |
| AprilTag world poses | position and orientation in `map` frame |
| Static arena map | image, resolution, origin, occupancy thresholds |
| LiDAR model and driver | exact package and serial settings |
| Excavation/deposition interfaces | commands, feedback, limits, and fault signals |

### 3.1 Platform compatibility gate

The runtime image shall use a Stereolabs/NVIDIA-validated combination of:

- JetPack/L4T,
- Ubuntu,
- CUDA,
- ZED SDK,
- ROS 2 distribution,
- ZED ROS 2 wrapper.

Do not select the ROS distribution independently of the Jetson and ZED software matrix.

Preferred policy:

1. Use ROS 2 Humble on Ubuntu 22.04 when that is the validated JetPack/ZED combination for the actual Orin Nano image.
2. Use ROS 2 Jazzy on Ubuntu 24.04 only when the selected JetPack and ZED SDK combination is validated on the target hardware.
3. Pin exact versions in `docker/versions.env`, the container image tag, and the repository README.
4. Do not implement against ROS Rolling or Kilted unless the team explicitly approves the compatibility risk.

A legacy Jetson Nano 4 GB is outside the performance assumptions of this document. The expected target is a Jetson Orin Nano 8 GB.

## 4. High-level architecture

```text
                              STATIC / GLOBAL

    arena map files --------------------------> Nav2 map_server
          |                                          |
          |                                          v
          |                                  global costmap
          |
    known tag poses
          |
          v
    tag_localizer <--- AprilTag detections <--- camera image
          |
          | PoseWithCovarianceStamped in map frame
          v
    global EKF --------------------------------------> map -> odom
          ^
          |
    local filtered odometry
          ^
          |
    local EKF ---------------------------------------> odom -> base_link
       ^       ^
       |       |
 wheel odom   ZED IMU

                              DYNAMIC / LOCAL

    2D LiDAR -----------------> LaserScan -----------> Nav2 obstacle layer

    ZED depth -> terrain_hazard_node -> PointCloud2 -> Nav2 voxel/obstacle layer

                              COMMAND / MISSION

    mission_manager -> Nav2 NavigateToPose action -> cmd_vel_nav

    teleop -----------------------------------------> cmd_vel_teleop
    safety locks -----------------------------------> twist_mux / hardware stop

    twist_mux -> diff_drive_controller -> hardware interface -> microcontroller
                                                        -> motor controllers
```

## 5. Required coordinate frames and TF ownership

### 5.1 TF tree

```text
map
└── odom
    └── base_link
        ├── left_wheel_link
        ├── right_wheel_link
        ├── zed_camera_link
        │   └── zed_camera_optical_frame
        ├── imu_link
        ├── lidar_link
        └── mechanism frames as required
```

`base_footprint` may be added later if Nav2 or visualization requires a gravity-projected base frame. The initial implementation shall use `base_link` consistently.

### 5.2 Transform authority table

| Transform | Sole publisher | Notes |
|---|---|---|
| `map -> odom` | Global `robot_localization` EKF | No other node may publish this transform during normal operation. |
| `odom -> base_link` | Local `robot_localization` EKF | Disable odometry TF publication from `diff_drive_controller`. |
| `base_link -> sensor frames` | `robot_state_publisher` from URDF/Xacro | These are rigid mounting transforms. |
| Camera internal/optical frames | ZED wrapper or URDF, but not both for the same transform | Confirm authority during integration. |
| Tag observation frames | AprilTag detector, optional | These must not publish `map -> odom`. |

The static arena occupancy grid is not a transform. It is a `nav_msgs/msg/OccupancyGrid` whose `header.frame_id` is `map`.

## 6. Localization specification

### 6.1 Local EKF

**Purpose:** Provide a continuous and locally smooth estimate of robot motion.

**Package:** `robot_localization` / `ekf_node`
**Node name:** `ekf_local`
**World frame:** `odom`
**TF output:** `odom -> base_link`
**Output topic:** `/odometry/local`

Inputs:

- Wheel odometry from `diff_drive_controller` or the wheel-odometry node.
- ZED IMU orientation and angular velocity.

Initial fusion policy:

- Fuse wheel-derived forward velocity.
- Fuse wheel-derived yaw velocity only if it is not duplicated by another input configuration.
- Fuse IMU yaw orientation only after frame alignment, magnetic assumptions, and covariance are validated.
- Fuse IMU angular velocity about Z.
- Do not use integrated linear acceleration for long-term X/Y position in the initial configuration.
- Enable `two_d_mode`.

Target rates:

- Wheel state/odometry: 50 Hz minimum.
- IMU: 50-100 Hz.
- Local EKF output: 30-50 Hz.

### 6.2 AprilTag detector

**Recommended baseline:** `christianrauch/apriltag_ros` or NVIDIA Isaac ROS AprilTag if CPU utilization is unacceptable and the selected software image supports it.

Inputs:

- Rectified monochrome or RGB image.
- Matching `CameraInfo` with the same timestamp.

Outputs:

- Detection array including ID, pose, decision margin, and hamming information.
- Optional observation TF frames.

Initial detector configuration:

- Use one tag family only, expected to be `tag36h11` unless the team selects another family.
- Run at 10-15 FPS initially.
- Use reduced image resolution if tag detection range remains acceptable.
- Do not run multiple tag families simultaneously without a measured need.

### 6.3 Known-tag localization node

**Package:** `lb_localization`
**Executable:** `tag_localizer`
**Input:** AprilTag detections and TF
**Output:** `/localization/apriltag_pose` as `geometry_msgs/msg/PoseWithCovarianceStamped` in `map` frame

The node shall:

1. Load known tag poses and tag sizes from YAML.
2. Obtain the rigid transform from `base_link` to the camera optical frame from TF.
3. Convert each camera-to-tag observation into a `map -> base_link` pose estimate.
4. Reject detections that fail configurable quality gates.
5. Combine multiple simultaneous valid detections using covariance-weighted position and orientation averaging.
6. Publish a pose covariance based on range, view angle, decision margin, tag pixel size, and inter-tag disagreement.
7. Publish diagnostics including accepted/rejected detection counts and the last valid correction age.
8. Never publish `map -> odom` directly.

Required quality-gate parameters:

- `max_tag_distance_m`
- `max_view_angle_deg`
- `min_decision_margin`
- `max_hamming`
- `max_pose_jump_m`
- `max_yaw_jump_rad`
- `max_detection_age_s`
- `minimum_valid_tags`

Known tag YAML schema:

```yaml
tags:
  - id: 0
    family: tag36h11
    size_m: TBD
    frame_id: tag_0
    pose_map:
      x: TBD
      y: TBD
      z: TBD
      roll: TBD
      pitch: TBD
      yaw: TBD
```

### 6.4 Global EKF

**Purpose:** Fuse smooth local odometry with intermittent absolute AprilTag pose corrections.

**Package:** `robot_localization` / `ekf_node`
**Node name:** `ekf_global`
**World frame:** `map`
**TF output:** `map -> odom`
**Output topic:** `/odometry/global`

Inputs:

- `/odometry/local`
- `/localization/apriltag_pose`

Requirements:

- Use `two_d_mode`.
- AprilTag poses are absolute, not differential.
- The global filter shall continue propagating from local odometry while no tag is visible.
- The global filter shall accept corrections based on measurement covariance.
- Correction jumps shall be observed and logged; the tag localizer shall reject implausible outliers before they reach the filter.
- TF ownership shall be verified with an automated launch test.

### 6.5 Localization validity

A `localization_monitor` component shall publish whether global localization is valid.

At minimum, validity shall consider:

- Local EKF output is current.
- IMU and wheel odometry are current.
- Global covariance is below configurable limits.
- The age since the last accepted AprilTag correction is below a mission-dependent limit.
- No frame or timestamp errors are active.

Recommended output:

- `/localization/status` using a custom message or `diagnostic_msgs/msg/DiagnosticArray`.
- `localization_valid` Boolean exposed to the mission manager and safety supervisor.

## 7. Static arena map

### 7.1 Map artifacts

The repository shall contain:

```text
navigation/maps/
├── arena.pgm
├── arena.yaml
├── keepout_mask.pgm        # optional
└── keepout_mask.yaml       # optional
```

The static map shall contain only fixed geometry and permanent forbidden areas, such as:

- Arena walls.
- Fixed field structures.
- Areas the robot is never allowed to enter.
- Known mechanism-clearance exclusions.

Dynamic rocks, regolith piles, and transient obstacles shall not be permanently written into the static map.

### 7.2 Map and tag coordinate consistency

The map origin and all AprilTag world poses shall use the same `map` coordinate frame and coordinate convention. A calibration document shall define:

- Map origin physical reference.
- Positive X direction.
- Positive Y direction.
- Zero-yaw direction.
- Tag pose measurement procedure.
- Expected measurement uncertainty.

## 8. Obstacle and terrain perception

### 8.1 2D LiDAR pipeline

**Input topic:** `/scan` as `sensor_msgs/msg/LaserScan`
**Primary use:** Fast detection of walls, boulders, robot-sized objects, and side/rear clearance.

Pipeline:

```text
LiDAR driver -> LaserScan filter -> Nav2 obstacle layer
```

The filter shall support:

- Minimum and maximum valid range.
- Removal of returns from the robot chassis/mechanisms.
- Angle masks for permanently blocked sectors.
- Rejection of invalid, NaN, and infinite samples as appropriate.

Target scan rate: 5-10 Hz minimum.

The LiDAR may be evaluated for AMCL later, but AMCL shall not be part of the baseline stack and shall not publish `map -> odom` while the global EKF is active.

### 8.2 ZED depth pipeline

**Package:** `lb_sensors`
**Executable:** `terrain_hazard_node`

Preferred input:

- Registered depth image plus `CameraInfo`.

Alternative input:

- Downsampled and cropped `PointCloud2` when depth-image projection is not sufficient.

The node shall avoid processing a full-resolution dense cloud unless profiling proves it is necessary.

Processing stages:

1. Validate timestamp and camera calibration.
2. Crop to a configurable forward region of interest.
3. Downsample spatially and/or temporally.
4. Transform samples to `base_link`.
5. Estimate traversable ground or compare measurements to an expected ground envelope.
6. Detect raised obstacles using height, slope, and connected-region thresholds.
7. Detect negative obstacles using missing-ground evidence, sudden depth discontinuities, unsafe slope, or crater-edge geometry.
8. Convert hazards into costmap-compatible output.

Outputs:

- `/perception/depth_obstacles` as `sensor_msgs/msg/PointCloud2`.
- `/perception/terrain_hazards` as `sensor_msgs/msg/PointCloud2` containing real or synthetic points that mark unsafe cells.
- `/perception/terrain_debug` optional visualization output, disabled in competition mode.
- Diagnostics and processing latency.

### 8.3 Nav2 costmap integration

The system shall not modify the static map for every detection. It shall use Nav2 costmap layers.

#### Global costmap

Initial layers:

- Static layer.
- Optional obstacle layer for hazards that must influence the full route.
- Inflation layer.
- Optional keepout filter.

#### Local costmap

Initial layers:

- 2D LiDAR obstacle layer.
- ZED voxel or obstacle layer.
- Terrain-hazard observation source.
- Inflation layer.
- Rolling window centered on the robot.

Recommended initial parameters, all configurable:

- Resolution: 0.05 m/cell.
- Local window: approximately 4-8 m square, selected from sensor range and arena size.
- Update frequency: 5-10 Hz.
- Publish frequency: 2-5 Hz.
- Obstacle persistence: short, normally 0.5-2.0 s unless repeated observations maintain it.
- Inflation radius: robot footprint margin plus localization uncertainty.

Negative-obstacle handling is a project-specific risk. The initial implementation may mark detected crater edges and unsafe regions using synthetic obstacle points. A custom Nav2 costmap layer is a later option if synthetic points are insufficient.

## 9. Drive control and hardware interface

### 9.1 Control chain

```text
Nav2 cmd_vel_nav ----\
Teleop cmd_vel_teleop ---> twist_mux ---> diff_drive_controller
Safety lock ----------/                         |
                                                v
                                      ros2_control hardware plugin
                                                |
                                                v
                                         microcontroller
                                                |
                                                v
                                  motor controllers and encoders
```

### 9.2 `ros2_control`

Use `ros2_control` and `diff_drive_controller` if the drivetrain is differential or skid-steer.

The controller shall:

- Accept linear X and angular Z velocity commands.
- Apply velocity, acceleration, and jerk limits.
- Stop automatically when command timeout is exceeded.
- Publish wheel-derived odometry but not the `odom -> base_link` TF.
- Expose wheel joint states.

### 9.3 Hardware plugin

**Package:** `lb_hardware`
**Class:** `LunabotDriveHardware` deriving from a `ros2_control` system hardware interface.

Responsibilities:

- Open and monitor the microcontroller communication channel.
- Write left/right wheel velocity or motor setpoint commands.
- Read encoder position/velocity.
- Read motor-controller fault and status data when available.
- Enforce a command heartbeat.
- Return an error state if communication is lost.
- Convert all hardware units to SI units at the ROS boundary.

The hardware plugin shall not assume the final packet protocol. Define an internal transport interface with at least:

```text
connect()
disconnect()
read_state()
write_command()
set_enabled()
stop()
get_faults()
```

Provide a `MockDriveTransport` for CI and simulation, and a real transport implementation after the electrical protocol is finalized.

### 9.4 Microcontroller requirements

The microcontroller shall provide the time-critical control and safety boundary. It shall:

- Receive high-level wheel commands, not raw motor phase switching.
- Command the brushless motor controllers through their supported interface.
- Report encoder and controller status.
- Stop motors if the Jetson heartbeat expires.
- Start in a disabled state after boot or reconnection.
- Require an explicit enable command after faults are cleared.
- Respect the hardware emergency stop independently of ROS.

## 10. Nav2 navigation configuration

### 10.1 Nav2 responsibilities

Nav2 shall provide:

- Static-map loading.
- Global planning.
- Local path following.
- Collision checking against local and global costmaps.
- Replanning when obstacles are detected.
- Standard navigation recovery behaviors.
- `NavigateToPose` and optional `FollowWaypoints` actions.

### 10.2 Recommended initial plugins

The exact plugins remain configurable. The recommended first implementation is:

- Planner: Smac Planner 2D or NavFn.
- Controller: Regulated Pure Pursuit Controller.
- Goal checker: Simple Goal Checker.
- Progress checker: Simple Progress Checker.
- Smoother: default/simple smoother if needed.
- Recovery behaviors: wait, backup only when mechanically safe, clear costmap, and controlled spin only if the drivetrain and excavation mechanism permit it.

MPPI, Hybrid-A*, and custom controllers are not baseline requirements.

### 10.3 Robot model and constraints

Nav2 shall use the measured robot footprint, not a point-robot approximation.

Parameters shall include:

- Footprint polygon.
- Maximum linear velocity.
- Maximum angular velocity.
- Linear/angular acceleration limits.
- Minimum turning behavior for the actual drivetrain.
- Goal position and yaw tolerance.
- Inflation radius and cost scaling.

All recovery motions shall account for the excavation mechanism footprint and ground clearance.

## 11. Mission control

### 11.1 Architecture decision

Use a custom mission state machine for mission sequencing. Use Nav2's internal behavior tree only for navigation execution and navigation recovery.

### 11.2 Mission states

The initial state machine shall support:

```text
BOOT
  -> SELF_CHECK
  -> WAIT_FOR_START
  -> ACQUIRE_LOCALIZATION
  -> NAVIGATE_TO_EXCAVATION
  -> EXCAVATE
  -> NAVIGATE_TO_DEPOSITION
  -> DEPOSIT
  -> RETURN_OR_REPEAT
  -> COMPLETE

At any state:
  -> SAFE_STOP
  -> ERROR_RECOVERY
  -> ABORTED
```

### 11.3 Mission manager

**Package:** `lb_state_manager`
**Executable:** `mission_manager`

Responsibilities:

- Execute explicit state transitions.
- Call Nav2 actions asynchronously.
- Call excavation and deposition actions asynchronously.
- Monitor localization, safety, battery, communication, and mechanism status.
- Implement timeouts and retry limits.
- Publish the current state and transition reason.
- Support start, pause, resume, abort, and reset operations.
- Never bypass the safety supervisor.

### 11.4 Mechanism interfaces

Define the following actions even before the mechanism implementation is complete:

```text
Excavate.action
--- Goal ---
float32 target_duration_s
float32 target_depth_m       # optional/TBD
--- Result ---
bool success
string message
--- Feedback ---
float32 progress
string phase
```

```text
Deposit.action
--- Goal ---
float32 target_duration_s
--- Result ---
bool success
string message
--- Feedback ---
float32 progress
string phase
```

A mock mechanism action server shall be available for navigation and mission testing.

## 12. Teleoperation and command arbitration

Use `twist_mux` or an equivalent command multiplexer.

Recommended priority, highest first:

1. Hardware emergency stop / hardware disable.
2. Safety supervisor lock and zero command.
3. Manual teleoperation.
4. Nav2 autonomous command.

Topics:

- `/cmd_vel_nav`
- `/cmd_vel_teleop`
- `/cmd_vel` as the final selected command

Teleoperation shall be usable without Nav2 for bench tests. Autonomous control shall not activate until the operator explicitly enables it.

## 13. Safety supervisor

**Package:** `lb_safety`
**Executable:** `safety_supervisor`

The safety supervisor shall monitor:

- Physical emergency-stop state.
- Jetson-to-microcontroller heartbeat.
- Motor-controller faults.
- Stale `/cmd_vel`.
- Stale wheel odometry.
- Stale IMU data.
- Lost or invalid localization.
- Excessive pitch/roll.
- Excessive motor current when available.
- Battery voltage thresholds.
- Depth and LiDAR health when autonomous navigation is active.
- Mission timeout or software node failure.

Required behaviors:

- Publish a `twist_mux` lock or equivalent motion-disable signal.
- Command zero velocity.
- Request hardware disable for severe faults.
- Latch severe faults until an explicit reset.
- Publish a clear fault code and human-readable reason.
- Continue to allow diagnostics while motion is disabled.

The physical emergency stop shall not depend on the Jetson, ROS, Wi-Fi, or mission software.

## 14. ROS interface contract

The following names are canonical at the application boundary. Vendor topics may be remapped to these names.

| Interface | Type | Publisher | Consumer |
|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR driver/filter | Nav2 costmap |
| `/imu/data` | `sensor_msgs/msg/Imu` | ZED wrapper/remap | Local EKF |
| `/odom/wheel` | `nav_msgs/msg/Odometry` | Diff drive controller | Local EKF |
| `/odometry/local` | `nav_msgs/msg/Odometry` | Local EKF | Global EKF, Nav2 |
| `/tag_detections` | AprilTag detection array | AprilTag detector | Tag localizer |
| `/localization/apriltag_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Tag localizer | Global EKF |
| `/odometry/global` | `nav_msgs/msg/Odometry` | Global EKF | Monitoring/UI |
| `/perception/depth_obstacles` | `sensor_msgs/msg/PointCloud2` | Terrain node | Nav2 costmap |
| `/perception/terrain_hazards` | `sensor_msgs/msg/PointCloud2` | Terrain node | Nav2 costmap |
| `/cmd_vel_nav` | `geometry_msgs/msg/TwistStamped` or stack-required Twist type | Nav2 | Twist mux |
| `/cmd_vel_teleop` | Matching Twist type | Teleop | Twist mux |
| `/cmd_vel` | Matching Twist type | Twist mux | Drive controller |
| `/mission/state` | Custom or standard status message | Mission manager | UI/logger |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | All major subsystems | Operator/logger |

The implementation shall use stamped velocity messages where supported. If a selected component requires an unstamped message, perform conversion in one documented adapter node rather than mixing types throughout the stack.

## 15. QoS and timing requirements

### 15.1 QoS policy

- High-rate sensor topics: sensor-data QoS, best effort unless the driver requires reliable delivery.
- Commands, mission actions, safety state, and hardware state: reliable.
- Static map: transient local/reliable as provided by map server.
- Parameters and services: reliable.
- TF: standard ROS 2 TF QoS.

### 15.2 Maximum data age

Initial targets:

| Data | Maximum age before warning | Maximum age before motion inhibit |
|---|---:|---:|
| Final velocity command | 0.15 s | 0.25 s |
| Wheel odometry | 0.15 s | 0.30 s |
| IMU | 0.10 s | 0.25 s |
| LiDAR during autonomy | 0.50 s | 1.00 s |
| Depth hazards during autonomy | 0.50 s | 1.00 s |
| Local EKF output | 0.15 s | 0.30 s |
| Last AprilTag correction | Mission-dependent warning | Mission-dependent speed limit or relocalization |

These are provisional engineering targets and shall be tuned from measured rates and stopping distance.

## 16. Repository structure

```text
lunabot_autonomy/
├── README.md
├── LICENSE
├── .gitignore
├── .pre-commit-config.yaml
├── docker/
│   ├── Dockerfile.jetson
│   ├── docker-compose.yml
│   ├── versions.env
│   └── README.md
├── docs/
│   ├── architecture.md
│   ├── calibration.md
│   ├── hardware_protocol.md
│   ├── operations.md
│   ├── testing.md
│   └── performance_budget.md
├── firmware/
│   └── drive_mcu/                # placeholder until MCU is selected
├── scripts/
│   ├── build.sh
│   ├── test.sh
│   ├── record_bag.sh
│   ├── performance_test.sh
│   └── check_tf_authority.py
└── src/
    ├── interfaces/
    ├── model/
    ├── hardware/
    ├── localization/
    ├── sensors/
    ├── navigation/
    ├── state_manager/
    ├── safety/
    ├── launch/
    └── sim/
```

### 16.1 Package responsibilities

| Package | Responsibility |
|---|---|
| `lb_interfaces` | Custom actions/messages/services only. |
| `lb_model` | URDF/Xacro, meshes, ros2_control tags, robot_state_publisher launch. |
| `lb_hardware` | ros2_control hardware plugin, transport abstraction, mock transport. |
| `lb_localization` | EKF configs, tag map, tag localizer, localization monitor. |
| `lb_sensors` | Depth processing, terrain hazards, LiDAR filters/config. |
| `lb_navigation` | Nav2 parameters, maps, costmap configuration, Nav2 BT XML. |
| `lb_state_manager` | Mission state machine and mechanism clients/mock servers. |
| `lb_safety` | Safety supervisor and motion locks. |
| `lb_launch` | System launch files and environment profiles. |
| `lb_sim` | Fake hardware, mock sensors, test worlds or bag-replay launch. |

## 17. Launch architecture

Required launch files:

```text
model/launch/description.launch.py
hardware/launch/hardware.launch.py
localization/launch/localization.launch.py
sensors/launch/perception.launch.py
navigation/launch/navigation.launch.py
state_manager/launch/mission.launch.py
safety/launch/safety.launch.py
launch/launch/bringup.launch.py
launch/launch/bench_test.launch.py
launch/launch/bag_replay.launch.py
sim/launch/mock_robot.launch.py
```

`bringup.launch.py` shall support arguments including:

- `use_sim_time`
- `use_mock_hardware`
- `enable_motors`
- `enable_zed`
- `enable_lidar`
- `enable_apriltags`
- `enable_depth_perception`
- `enable_nav2`
- `enable_mission`
- `map`
- `params_file`

Default launch behavior shall keep motors disabled.

## 18. Configuration files

```text
config/
├── ros2_control.yaml
├── ekf_local.yaml
├── ekf_global.yaml
├── tag_map.yaml
├── tag_localizer.yaml
├── zed.yaml
├── lidar.yaml
├── terrain_hazard.yaml
├── nav2_params.yaml
├── twist_mux.yaml
├── safety.yaml
├── mission.yaml
└── robot_limits.yaml
```

Rules:

- No hardware dimensions or safety limits shall be hard-coded in source.
- Topic and frame names shall be parameters or launch remappings.
- All parameters shall have units in comments and documentation.
- Invalid or missing safety-critical parameters shall fail startup rather than silently using unsafe defaults.

## 19. Compute and performance budget

### 19.1 Target hardware

- NVIDIA Jetson Orin Nano 8 GB.
- Active cooling.
- Sustained power mode selected from measured workload, potentially up to the board's maximum supported mode.
- Headless runtime during competition.
- RViz, Groot, and heavy visualization run on a separate development laptop when possible.

### 19.2 Expected workload

| Component | Expected relative load |
|---|---|
| ZED stereo depth | High GPU and shared-memory load |
| ZED image publication | Medium memory/bandwidth load |
| AprilTag detection | Medium CPU; GPU option available if needed |
| Depth terrain analysis | Medium CPU/GPU depending implementation |
| Nav2 and costmaps | Medium CPU, modest memory |
| 2D LiDAR | Low |
| Two EKFs | Very low |
| Hardware, mission, safety | Very low |
| RViz on Jetson | Avoid in competition mode |

### 19.3 Initial camera/runtime profile

- Depth/RGB resolution: 720p or lower.
- Depth frame rate: 15 FPS initial target.
- AprilTag processing: 10-15 FPS.
- Full dense point cloud: disabled by default.
- ZED positional tracking: disabled.
- ZED object detection: disabled.
- Debug image publication: disabled in competition profile.
- Rosbag: selective topics only during full-system operation.

### 19.4 Resource acceptance target

During a 30-minute representative autonomous run:

- No out-of-memory event.
- No sustained thermal throttling.
- At least 10-15 percent shared-memory headroom under typical load.
- Control loop and local EKF rates remain within 90 percent of targets.
- Depth hazard processing latency remains below 150 ms initial target.
- Nav2 controller command age remains below the safety timeout.
- No unbounded queue growth.

Create `scripts/performance_test.sh` to log:

- `tegrastats` output.
- Node CPU and memory use.
- Topic rates and bandwidth.
- Callback latency where measurable.
- Dropped frames.
- Costmap update timing.

If the Orin Nano 8 GB cannot meet the target, optimize in this order:

1. Disable unused ZED topics and modules.
2. Lower image/depth resolution and rate.
3. Crop and downsample before point-cloud generation.
4. Process depth images instead of dense point clouds.
5. Move visualization offboard.
6. Reduce costmap size and publication rate.
7. Use composable nodes/intra-process transport where compatible.
8. Evaluate GPU-accelerated AprilTag or NITROS only after the baseline is stable.
9. Upgrade to a 16 GB Orin platform only if profiling still shows memory exhaustion.

## 20. Diagnostics and logging

Every custom node shall:

- Publish lifecycle/health information.
- Report stale input and transform lookup failures.
- Use throttled error logging for repeated failures.
- Expose key thresholds as parameters.
- Include timestamps and frame IDs in debug output.

Recordable topic groups:

- `localization`: wheel odom, IMU, local/global odometry, tag detections, tag pose, TF.
- `perception`: scan, depth obstacles, terrain hazards, selected depth/debug stream.
- `navigation`: plans, costmaps, cmd_vel, feedback, result.
- `hardware`: joint states, commands, motor state, faults.
- `mission`: state transitions, mechanism feedback, safety state.

Bag replay shall support testing localization and perception without motor output.

## 21. Testing strategy

### 21.1 Unit tests

Required unit tests include:

- Tag transform composition.
- Multi-tag weighted fusion.
- AprilTag quality-gate rejection.
- Covariance scaling.
- Terrain threshold and clustering logic.
- Mission state transitions.
- Safety fault latching and reset.
- Hardware protocol encoding/decoding after protocol definition.

### 21.2 Launch and integration tests

Required automated checks:

- All packages build with warnings treated as errors for custom C++ code.
- `bringup.launch.py` starts with mock hardware.
- Exactly one publisher exists for `map -> odom`.
- Exactly one publisher exists for `odom -> base_link`.
- Required transforms become available within a timeout.
- Static map loads and is in `map` frame.
- A synthetic LiDAR obstacle marks the local costmap.
- A synthetic depth hazard marks the local costmap.
- A synthetic tag detection shifts the global pose without interrupting local odometry.
- Nav2 can reach a goal in simulation/mock environment.
- Safety lock prevents all nonzero commands from reaching the drive controller.

### 21.3 Hardware integration tests

Perform in this order:

1. Motor controller bench test with motor mechanically secured.
2. Wheel direction and encoder sign test with wheels off the ground.
3. Command-timeout stop test.
4. Physical emergency-stop test.
5. Straight-line wheel-odometry calibration.
6. In-place/low-speed turning calibration.
7. IMU axis and yaw-sign validation.
8. Static AprilTag pose accuracy test.
9. Moving AprilTag correction test.
10. LiDAR obstacle marking/clearing test.
11. Raised obstacle depth test.
12. Crater/drop-off test.
13. Nav2 low-speed path following.
14. Full mission with mock mechanisms.
15. Full mission with real mechanisms.

## 22. Provisional acceptance criteria

These are initial targets and shall be revised after mechanical dimensions and competition requirements are confirmed.

### 22.1 Localization

- TF tree is complete and has no competing transform publishers.
- Local odometry is smooth and does not jump when an AprilTag correction occurs.
- AprilTag correction updates `map -> odom` through the global EKF.
- A rejected outlier tag does not shift the global pose.
- Robot reacquires a valid global pose after tag visibility returns.
- Pose and covariance are timestamp-correct and frame-correct.

### 22.2 Navigation and perception

- Static obstacles in the arena map are never planned through.
- LiDAR obstacles are marked and cleared according to observation/raytrace rules.
- Depth hazards become lethal or high-cost cells before the robot footprint reaches them.
- Nav2 replans or stops when the active path becomes unsafe.
- The robot remains within configured speed and acceleration limits.

### 22.3 Safety

- Hardware emergency stop removes propulsion independently of ROS.
- Loss of command heartbeat stops the drive system.
- Safety lock prevents autonomous and teleop commands from reaching motors.
- Severe faults remain latched until an explicit safe reset.
- Motors remain disabled after software restart until explicitly enabled.

### 22.4 Compute

- Full headless stack runs for 30 minutes without OOM, thermal shutdown, or process crash.
- Control and safety deadlines are maintained while ZED depth processing is active.
- Competition profile does not require RViz on the Jetson.

## 23. Implementation phases and Codex work packages

Codex shall implement small, reviewable phases. It shall not attempt the entire robot in one change.

### Phase 0 - Environment and repository scaffold

Deliverables:

- Repository structure.
- ROS distribution and container version lock.
- Formatting/lint/test configuration.
- Empty packages with documented ownership.
- Top-level README and build scripts.

Exit criteria:

- Container builds on target architecture.
- `colcon build` and `colcon test` pass.

### Phase 1 - Description, TF, and mock robot

Deliverables:

- Parameterized Xacro.
- Sensor frames.
- ros2_control mock hardware.
- Joint state publisher and robot_state_publisher.
- TF authority test.

Exit criteria:

- Correct TF tree appears in RViz.
- No motor hardware is required.

### Phase 2 - Drive hardware interface

Deliverables:

- Transport abstraction.
- Mock transport.
- Real transport skeleton with explicit TODOs for unknown protocol fields.
- Diff drive controller configuration.
- Bench-test launch.

Exit criteria:

- Mock wheels respond to commands.
- Hardware plugin fails safely on communication loss.

### Phase 3 - Local localization

Deliverables:

- Wheel odometry interface.
- ZED IMU remapping/configuration.
- Local EKF config.
- Localization diagnostics.

Exit criteria:

- `odom -> base_link` is smooth and sole-owned by local EKF.

### Phase 4 - AprilTag global localization

Deliverables:

- Detector launch/config.
- Tag map schema.
- `tag_localizer` with unit tests.
- Global EKF config.

Exit criteria:

- Global EKF solely publishes `map -> odom`.
- Synthetic and recorded detections correct global pose.

### Phase 5 - Static map and basic Nav2

Deliverables:

- Map server configuration.
- Placeholder arena map.
- Nav2 planner/controller configuration.
- Twist mux.

Exit criteria:

- Mock robot navigates between two poses in a known map.

### Phase 6 - 2D LiDAR perception

Deliverables:

- Driver integration placeholder/remapping.
- Scan filter.
- Costmap obstacle source.

Exit criteria:

- Recorded or synthetic scans mark and clear obstacles.

### Phase 7 - ZED depth terrain hazards

Deliverables:

- Depth input adapter.
- Crop/downsample pipeline.
- Raised-obstacle detection.
- Negative-obstacle/crater-edge prototype.
- Costmap-compatible outputs.
- Performance instrumentation.

Exit criteria:

- Test obstacles and drop-offs create avoidable costmap cells within latency target.

### Phase 8 - Mission and mechanism interfaces

Deliverables:

- Mission state machine.
- Excavate/Deposit actions.
- Mock mechanism servers.
- Nav2 action client.

Exit criteria:

- Full mock mission completes with logged transitions.

### Phase 9 - Safety and field validation

Deliverables:

- Safety supervisor.
- Hardware heartbeat integration.
- Fault injection tests.
- Operations guide.
- Performance report.

Exit criteria:

- Safety tests pass.
- Full stack meets compute and endurance targets.

## 24. Coding standards

- C++ for hardware interfaces and high-rate/performance-sensitive perception.
- Python is acceptable for launch files, mission orchestration, calibration utilities, and low-rate glue nodes.
- Use type annotations in Python.
- Use `ament_cmake` or `ament_python` appropriately.
- Use ROS parameters instead of constants for tunable behavior.
- Use standard ROS messages unless a custom message adds clear value.
- Avoid global mutable state.
- Do not block executor threads while waiting for actions or services.
- Use lifecycle nodes where startup order and controlled shutdown are important.
- Use `tf2` with message timestamps; do not use latest-transform lookups as a default workaround.
- Treat compiler warnings and lint errors as failures in CI.
- Document all frame, unit, and sign conventions.

## 25. Instructions to Codex

When implementing from this specification:

1. Begin with Phase 0 only unless explicitly told to continue.
2. Read the entire specification before changing files.
3. Produce an implementation plan and list all unresolved `TBD` hardware values.
4. Do not fabricate electrical pinouts, serial/CAN packet formats, wheel dimensions, or tag poses.
5. Use mocks and interfaces when hardware details are unknown.
6. Preserve TF ownership exactly as specified.
7. Add tests with each functional package.
8. Run `colcon build`, `colcon test`, and lint before reporting completion.
9. Summarize changed files, commands run, test results, assumptions, and remaining blockers.
10. Keep each change small enough for human review.

### Recommended first Codex prompt

```text
Read docs/lunabotics_autonomy_software_spec.md. Implement Phase 0 only.
Create the repository and ROS 2 package scaffold, container/version placeholders,
build/test scripts, and documentation skeleton. Do not implement hardware logic.
Mark unresolved platform and hardware values as explicit TBDs. Run available build,
test, and lint commands and report the results.
```

## 26. Key risks

| Risk | Mitigation |
|---|---|
| Wheel slip corrupts odometry | Frequent AprilTag corrections; tune covariance; slow near mission zones. |
| Tag occlusion or poor lighting | Multiple tags; quality gating; relocalization state; appropriate illumination. |
| Stereo depth fails on low-texture regolith | 2D LiDAR redundancy; test lighting; crop/threshold tuning; conservative stop behavior. |
| Craters are negative obstacles | Dedicated terrain hazard logic; mark rim/unsafe region; extensive mock-arena tests. |
| 8 GB shared memory is exhausted | Headless mode; lower resolution/rate; disable clouds/modules; profile continuously. |
| Multiple TF publishers conflict | Sole-authority table plus automated launch test. |
| Unknown motor interface delays software | Transport abstraction and mock hardware. |
| State machine grows complex | Keep mission-level states coarse; delegate navigation recovery to Nav2 BT. |
| Mechanism changes robot footprint | Parameterized footprint and mechanism-state-aware safety margins. |
| Dust/vibration causes sensor failure | Rigid mounting, diagnostics, stale-data safety stop, redundant obstacle sensing. |

## 27. Official technical references

The architecture was checked against official or primary project documentation available on 2026-08-04:

1. ROS 2 installation and supported platforms: ROS 2 Jazzy documentation.
2. Nav2 Costmap 2D, Map Server, Obstacle Layer, Voxel Layer, and Behavior Tree documentation.
3. `robot_localization` state-estimation documentation and Nav2 odometry setup guide.
4. `ros2_control` Jazzy documentation and `diff_drive_controller` user documentation.
5. Stereolabs ZED ROS 2 integration and container guidance.
6. `christianrauch/apriltag_ros` ROS 2 repository and AprilRobotics AprilTag reference implementation.
7. `twist_mux` ROS 2 package documentation.

Version-specific links shall be pinned in the repository README after the actual JetPack/ROS/ZED compatibility matrix is selected.

## 28. Approval checklist

Before implementation begins, the team should confirm:

- [ ] The computer is an NVIDIA Jetson Orin Nano 8 GB.
- [ ] The validated JetPack, Ubuntu, ROS 2, ZED SDK, and wrapper versions.
- [ ] Differential/skid-steer drive assumption.
- [ ] Motor-controller and microcontroller communication method.
- [ ] Wheel dimensions, gearing, and encoder resolution.
- [ ] ZED and LiDAR mounting locations.
- [ ] AprilTag camera choice, tag family, tag size, and placement plan.
- [ ] Arena map origin and coordinate convention.
- [ ] Robot footprint in all mechanism configurations.
- [ ] Safety limits and physical emergency-stop design.
- [ ] Exact excavation and deposition mechanism interfaces.
