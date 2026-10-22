# Nav2 specification and implementation plan

**Document status:** implementation draft
**Date:** 2026-10-22
**Owning package:** `lb_navigation`
**Related packages:** `lb_localization`, `lb_safety`, `lb_hardware`,
`lb_model`, `lb_sensors`, `lb_sim`, and `lb_launch`

## 1. Purpose

This document defines the Nav2 boundary, prerequisites, interfaces, safety
rules, staged implementation, and acceptance criteria for the Lunabotics
robot. It specializes the broader
[`lunabotics_autonomy_software_spec.md`](lunabotics_autonomy_software_spec.md)
without weakening its TF, safety, evidence, or parameter-provenance rules.

The first implementation is a non-physical development profile. It may use
explicitly estimated geometry and AprilTag-only localization, but it shall not
represent those inputs as measured, continuously propagated, or safe for the
physical robot.

## 2. Agreed inputs and temporary constraints

- The robot is estimated to fit within a maximum `1.10 m` front-to-back by
  `0.60 m` wide rectangle. This is a team estimate, not CAD or a completed
  measurement, and it does not include a verified mechanism swept envelope or
  navigation padding.
- ROS REP-103 applies: robot `+X` is forward, `+Y` is left, and `+Z` is up.
- The AprilTag absolute robot-pose measurement is
  `/localization/apriltag_pose` with type
  `geometry_msgs/msg/PoseWithCovarianceStamped` in `map`.
- The stable application-facing pose is `/localization/fused_pose`, also
  `geometry_msgs/msg/PoseWithCovarianceStamped` in `map`.
- Initially, `/localization/fused_pose` is a freshness-gated direct
  pass-through of `/localization/apriltag_pose`. This is not multi-sensor
  fusion, filtering, interpolation, or dead reckoning.
- Until continuous localization exists, an absent, rejected, invalid, or stale
  AprilTag pose shall inhibit all nonzero Nav2 commands.
- The current map, arena control points, camera extrinsic, tag pose, footprint,
  drive calibration, and motion limits retain the statuses recorded in
  [`assumptions_register.md`](assumptions_register.md).

## 3. Scope

### 3.1 In scope

- A stable fused-pose application interface.
- Localization validity and navigation-command gating.
- A provisional rectangular development footprint.
- Nav2 lifecycle and parameter configuration.
- Global and local costmaps.
- Planner, controller, smoother, goal checker, progress checker, navigator,
  behavior server, and velocity smoother integration.
- `NavigateToPose` operation with mock hardware.
- RViz goal, plan, footprint, localization, and costmap displays.
- Synthetic obstacle-source tests followed by LiDAR and ZED sources when their
  hardware contracts exist.
- Replanning, recovery, diagnostics, logging, and acceptance evidence.

### 3.2 Out of scope for the first implementation

- Physical motor enablement.
- Claiming continuous localization from intermittent AprilTag observations.
- Final footprint or recovery-clearance approval.
- Production velocity, acceleration, jerk, inflation, or stopping limits.
- Final LiDAR or ZED terrain parameters.
- Autonomous mission sequencing, excavation, or deposition.
- AMCL, SLAM, persistent 3D reconstruction, or autonomous tag-map generation.

## 4. Architecture

### 4.1 Localization boundary

```text
Current development profile

AprilTag observations
        |
        v
tag_localizer
        |
        | /localization/apriltag_pose
        v
fused-pose boundary ----------------> /localization/fused_pose
        |                                      |
        v                                      v
localization validity                  RViz / monitoring
        |
        v
Nav2 command inhibit/permit
```

```text
Future continuous profile

wheel odometry + ZED IMU -> local EKF -> /odometry/local
                                          |
AprilTag pose ----------------------------+--> global EKF
                                                   |
                                                   +--> map -> odom
                                                   +--> /odometry/global
                                                             |
                                                             v
                                                  fused-pose adapter
                                                             |
                                                             v
                                              /localization/fused_pose
```

`/localization/fused_pose` decouples monitoring, mission, and UI consumers from
the changing localization implementation. The future global EKF may continue
to expose `/odometry/global` as `nav_msgs/msg/Odometry`; a small adapter shall
copy its pose and pose covariance into `/localization/fused_pose` without
discarding the original measurement timestamp.

The AprilTag-only pass-through shall:

- accept only `map`-frame `PoseWithCovarianceStamped` samples;
- preserve the input timestamp, pose, and covariance exactly;
- publish one fused-pose sample per accepted AprilTag-pose sample;
- publish no synthetic samples between detections;
- never refresh an old sample with the current time;
- reject zero, future-skewed, out-of-order, or stale timestamps according to
  configurable and validated limits;
- publish validity and rejection diagnostics; and
- publish no TF.

The direct pass-through is temporary. It expires when the continuous local and
global localization pipeline is validated.

### 4.2 TF contract

Nav2 requires a current transform chain:

```text
map -> odom -> base_link
```

- The global `robot_localization` EKF is the sole production owner of
  `map -> odom`.
- The local `robot_localization` EKF is the sole production owner of
  `odom -> base_link`.
- The fused-pose pass-through owns neither transform.
- No Nav2 controller server may be activated merely because
  `/localization/fused_pose` exists; the complete current TF chain must also be
  available.
- A development TF provider, if required before the EKFs exist, must be
  explicitly test-only, mutually exclusive with the EKFs, disabled by default,
  and prohibited from physical bringup.

### 4.3 Navigation data flow

```text
arena.yaml -> map_server -> /map -> global costmap -> planner_server

LiDAR / ZED hazards -----------------> local/global costmap layers
                                               |
/localization/fused_pose + valid TF ------------+--> bt_navigator
                                                       |
                                                       v
                                                controller_server
                                                       |
                                                       v
                                                 /cmd_vel_nav
                                                       |
localization validity lock ----------------------------+--> command mux
teleoperation -----------------------------------------+        |
safety supervisor -------------------------------------+        v
                                                            /cmd_vel
```

## 5. ROS interface contract

| Interface | Type | Producer | Consumer | Requirement |
|---|---|---|---|---|
| `/localization/apriltag_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `tag_localizer` | Current fused-pose pass-through; future global EKF | Absolute `map`-frame measurement; never a TF authority. |
| `/localization/fused_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Initial planned pass-through; future global-fusion adapter | Localization monitor, mission/UI, RViz, tests | Stable application-facing pose; its source and validity must be diagnosed. |
| `/localization/status` | `diagnostic_msgs/msg/DiagnosticArray` initially | Localization monitor/pass-through | Safety, operator, tests | Reports source, freshness, validity, covariance checks, and inhibit reason. |
| `/odometry/local` | `nav_msgs/msg/Odometry` | Future local EKF | Global EKF, Nav2 velocity consumers | Continuous `odom`-frame state; absent in the initial AprilTag-only profile. |
| `/odometry/global` | `nav_msgs/msg/Odometry` | Future global EKF | Fused-pose adapter, monitoring | Full filtered global state; not fabricated by the AprilTag pass-through. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `map_server` | Global costmap, RViz | Transient-local, reliable, `map` frame. |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `bt_navigator` action server | Operator, RViz, future mission manager | Goals accepted only while navigation prerequisites are valid. |
| `/cmd_vel_nav` | Nav2-compatible stamped twist type | Nav2 controller chain | Command mux/gate | Must be blocked or forced to zero when localization is invalid. |
| `/cmd_vel` | Drive-controller-compatible stamped twist type | Command mux | Drive controller | Final software command; physical output remains disabled in development. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Major components | Operator/logger | Includes Nav2 lifecycle, TF, costmap, pose-source, and command-gate state. |

The concrete command-gate topic or `twist_mux` lock interface shall be selected
with the installed Humble-compatible mux implementation. It must be reliable,
fail locked on publisher/node loss, and covered by an integration test. A topic
name shall not be invented before that component is selected.

## 6. Localization validity and command inhibition

### 6.1 AprilTag-only development state

The localization state machine shall have at least these states:

- `UNAVAILABLE`: no accepted AprilTag pose has been received.
- `VALID`: the most recent accepted pose is within the configured freshness
  limit and all frame/timestamp checks pass.
- `STALE`: the last accepted pose exceeds the freshness limit.
- `INVALID`: a pose was rejected for frame, timestamp, covariance, quality, or
  transform reasons.

The AprilTag freshness limit is a required parameter whose production value is
still `TBD`; startup shall fail or navigation shall remain inhibited when it is
missing or invalid. The replay visualizer's `0.50 s` value is not silently a
production default.

### 6.2 Required fail-closed behavior

When localization is not `VALID`:

- reject new navigation goals or immediately return a clear unavailable result;
- cancel or halt the active navigation action;
- inhibit nonzero `/cmd_vel_nav` from reaching `/cmd_vel`;
- command zero velocity through the software command path;
- retain diagnostics and visualization; and
- never continue using the last visible pose as though it were current.

Reacquiring a valid AprilTag pose clears the localization condition, but shall
not silently resume a previously cancelled physical motion. Resumption requires
a new goal or an explicit higher-level retry policy.

The command gate is defense in depth. It does not replace the physical E-stop,
hardware disable, motor-controller timeout, or future safety supervisor.

## 7. Provisional robot footprint

### 7.1 Estimated envelope

The current maximum estimate is:

- front-to-back length: `1.10 m`;
- lateral width: `0.60 m`; and
- height: `TBD` and irrelevant to the planar polygon except for sensor filters.

If a centered `base_link` development profile is needed before its physical
origin is measured, the provisional polygon is:

```yaml
footprint: "[[0.55, 0.30], [0.55, -0.30], [-0.55, -0.30], [-0.55, 0.30]]"
```

This polygon assumes `base_link` is at the rectangle center. It is not an
approved physical footprint and shall be stored only in an explicitly named
development parameter file. Production configuration must replace it with a
measured polygon referenced to the actual `base_link` origin.

### 7.2 Missing envelope information

The estimate does not yet establish:

- bucket or excavation-mechanism swept volume;
- wheel, fastener, guard, cable, or sensor protrusions;
- changes between travel, excavation, and deposition states;
- stopping and localization-uncertainty margin;
- minimum wall, berm, column, and obstacle clearance; or
- recovery-motion clearance, especially for backing and spinning.

Footprint padding and costmap inflation shall therefore remain separately
parameterized and explicitly provisional. A circular `robot_radius` shall not
replace the polygon merely for convenience.

## 8. Costmap specification

### 8.1 Global costmap

The initial global costmap shall use:

- `map` as global frame;
- `base_link` as robot base frame unless a reviewed `base_footprint` is added;
- the lifecycle-managed static arena map;
- the provisional polygon in the development profile only;
- static and inflation layers initially; and
- optional obstacle/keepout layers only after their inputs and semantics are
  tested.

The arena-scene markers are not costmap input. The starting-zone,
excavation-zone, and berm drawings have no control authority until their zone
semantics and surveyed geometry are approved.

### 8.2 Local costmap

The initial local costmap shall be a rolling window centered on the robot. Its
window, resolution, update rate, publish rate, transform tolerance, inflation,
and persistence remain tunable development parameters. The first runnable
profile may contain no live obstacle layer, but it must be visibly labeled
static-only and must not be used on the physical robot.

Obstacle sources shall be added in this order:

1. deterministic synthetic `LaserScan` for marking and clearing tests;
2. deterministic synthetic terrain-hazard points;
3. selected and calibrated physical 2D LiDAR;
4. validated ZED raised- and negative-obstacle outputs.

No source may be enabled with invented range, height, raytrace, persistence,
QoS, or stale-data parameters.

## 9. Initial Nav2 components

The development baseline shall evaluate the following conservative stack:

- `map_server` and its existing lifecycle manager;
- `planner_server` with NavFn or Smac Planner 2D;
- `controller_server` with Regulated Pure Pursuit;
- Simple Goal Checker and Simple Progress Checker;
- smoother server only if path evidence shows a need;
- behavior server with wait and costmap-clear behaviors initially;
- `bt_navigator` providing `NavigateToPose`;
- waypoint follower only after single-goal navigation passes;
- velocity smoother with limits tied to the same provisional robot-limits
  profile; and
- a separate lifecycle manager for navigation nodes unless evidence supports a
  safe unified manager.

Backup and spin recoveries remain disabled until the mechanism envelope,
rear/side sensing, ground clearance, and low-speed skid-steer behavior show
they are safe. MPPI, Hybrid-A*, and custom planners/controllers are not initial
requirements.

Plugin selection is not complete merely because a node starts. It requires
path feasibility, controller stability, CPU/memory, recovery, and failure-mode
evidence on representative geometry.

## 10. Launch and configuration layout

The implementation should add these owned artifacts incrementally:

```text
lb_localization/
├── config/fused_pose_passthrough.yaml
├── launch/localization_passthrough.launch.py
└── test/

lb_navigation/
├── config/nav2_development.yaml
├── config/robot_limits_development.yaml
├── behavior_trees/
├── launch/navigation.launch.py
├── rviz/navigation.rviz
└── test/

lb_safety/
├── config/navigation_gate.yaml
└── test/

lb_sim/
├── launch/nav2_mock.launch.py
└── test/
```

Names may be refined during implementation to follow installed package
conventions, but ownership shall remain as shown. Development-only files must
identify their provenance and physical-use prohibition in the file itself.

Required launch arguments include:

- `use_sim_time`;
- `map`;
- `params_file`;
- `use_mock_hardware`;
- `enable_nav2`;
- `enable_motors`, default `false`;
- `localization_profile`;
- `footprint_profile`; and
- sensor-source enable flags.

Invalid files, missing required localization thresholds, missing TF, or an
unsafe profile combination shall fail before navigation activation.

## 11. Staged implementation

### Stage N0 — Readiness and dependency audit

- Verify installed Humble/Jazzy Nav2 component names and message compatibility.
- Record required packages and bootstrap checks.
- Create non-runnable or fail-closed parameter/launch scaffolding.
- Verify map-server reuse and lifecycle ownership.

**Exit:** configuration loads structurally, and missing prerequisites produce
clear failures without starting a command path.

### Stage N1 — Fused-pose boundary and localization gate

- Implement the freshness-gated AprilTag pass-through.
- Publish `/localization/fused_pose` and localization diagnostics.
- Implement fail-closed Nav2-command inhibition.
- Test startup without a pose, accepted pose, stale pose, invalid frame/time,
  clock rewind, loss, and reacquisition.

**Exit:** the fused topic exactly mirrors accepted AprilTag poses, and no
nonzero Nav2 command passes without current valid localization.

### Stage N2 — Development footprint and robot limits

- Add the `1.10 x 0.60 m` centered development polygon.
- Keep footprint padding, inflation, and motion limits independently
  parameterized and provenance-labeled.
- Check consistency among costmaps, RViz, and the robot model while preserving
  the distinction from the older synthetic `0.90 x 0.60 m` visual chassis.

**Exit:** one explicit development footprint is consumed consistently, and no
production launch silently selects it.

### Stage N3 — Continuous TF prerequisite for mock navigation

- Prefer implementation of `/odom/wheel`, ZED/mock IMU, and the local/global
  EKFs according to the main specification.
- If scheduling requires a temporary mock TF provider, isolate and gate it as
  described in Section 4.2.
- Add sole-authority and transform-freshness tests.

**Exit:** `map -> odom -> base_link` is complete, current, sole-owned, and
compatible with the selected localization profile.

### Stage N4 — Static global and rolling local costmaps

- Configure global static/inflation layers.
- Configure the local rolling costmap without claiming unimplemented sensors.
- Add RViz displays and lifecycle tests.

**Exit:** footprint collision and inflation appear correctly, static occupied
cells cannot be planned through, and loss of localization deactivates or locks
navigation.

### Stage N5 — Planner, controller, and safe command chain

- Select initial plugins using recorded rationale.
- Configure planner/controller servers, checkers, navigator, velocity smoother,
  and the command mux/gate.
- Route commands only to mock hardware.

**Exit:** plans and bounded mock commands are produced only while every command
precondition is valid.

### Stage N6 — NavigateToPose and reusable RViz workflow

- Add goal, feedback, result, plan, costmap, footprint, pose, diagnostics, and
  command displays.
- Provide RViz and command-line procedures.
- Navigate between at least two collision-free poses using mock hardware.

**Exit:** the action completes in mock operation, and deliberate localization
loss cancels or halts it with zero output.

### Stage N7 — Obstacle sources

- Add deterministic synthetic marking and clearing tests.
- Integrate physical LiDAR and ZED sources only after their own contracts close.
- Test stale-source response and path invalidation.

**Exit:** obstacles and hazards affect costmaps and cause safe replanning or
stopping within validated latency.

### Stage N8 — Replanning, recovery, and field-readiness evidence

- Exercise blocked paths, no progress, unreachable goals, costmap staleness,
  localization loss, and recovery exhaustion.
- Enable only physically justified recoveries.
- Measure timing, CPU, memory, command age, and endurance.

**Exit:** automated and operator tests demonstrate bounded behavior, safe
failure, and documented remaining physical-validation gates.

## 12. Testing and acceptance criteria

### 12.1 Static and unit checks

- Every provisional number has units and provenance.
- Production profiles contain no synthetic fallback footprint or motion limit.
- Topic types, frames, and QoS match the interface contract.
- Fused-pose pass-through preserves pose, timestamp, and covariance.
- Invalid/stale input produces no fused sample and asserts the navigation lock.
- No node other than the selected EKFs owns production map/odometry TF edges.

### 12.2 Launch and integration checks

- Missing map, params, required timeout, or TF fails closed.
- Lifecycle activation order is deterministic.
- `/map` is transient-local and in `map`.
- Global and local costmaps use the selected footprint.
- A goal cannot produce a nonzero final command without valid localization.
- Tag loss during an active goal produces zero command and a cancelled/halted
  action within a measured interval.
- Reacquisition does not silently resume cancelled physical motion.
- Static occupied cells are never planned through.
- Synthetic obstacles mark, clear, and invalidate paths as specified.
- Exactly one publisher owns each TF edge.

### 12.3 Physical validation gates

Physical Nav2 output remains prohibited until all of the following have
evidence:

- measured footprint and mechanism envelopes relative to `base_link`;
- calibrated wheel radius, separation, gearing, encoder signs, and covariance;
- measured camera and LiDAR transforms;
- surveyed tag and arena control points;
- continuous local/global localization with validated freshness/covariance;
- physical E-stop, enable/reset, watchdog, and command-timeout tests;
- safe linear/angular velocity, acceleration, jerk, and stopping distance;
- obstacle and drop-off detection at representative speeds; and
- low-speed path-following and recovery-clearance tests.

## 13. Evidence and documentation requirements

Every completed stage shall update:

- [`requirements_traceability.md`](requirements_traceability.md);
- [`interfaces.md`](interfaces.md) when a topic, action, service, message type,
  or ownership rule becomes executable;
- [`architecture.md`](architecture.md) when node or TF ownership changes;
- [`assumptions_register.md`](assumptions_register.md) when a temporary value is
  introduced, refined, or closed;
- [`testing.md`](testing.md) and an evidence record with commands and results;
- [`operations.md`](operations.md) for operator-visible launch or failure
  behavior; and
- [`../resources/devlog.md`](../resources/devlog.md) with the associated Git
  commit.

Rosbags, TF graphs, ROS graphs, costmap screenshots, plans, diagnostics,
terminal output, timing plots, and performance logs shall be retained where
they substantiate an acceptance criterion.

## 14. Open decisions before physical use

- Actual `base_link` origin within the estimated envelope.
- Measured footprint in each mechanism configuration.
- Footprint padding and inflation margin.
- AprilTag freshness and covariance validity thresholds.
- Behavior when localization quality degrades but has not become invalid.
- Final planner, controller, smoother, goal checker, and progress checker.
- Goal position/yaw tolerances and replanning frequency.
- Safe recovery motions.
- Command-mux/lock package and exact lock interface.
- Continuous local/global EKF parameters.
- LiDAR and ZED obstacle-source parameters.
- Physical motion and stopping limits.
