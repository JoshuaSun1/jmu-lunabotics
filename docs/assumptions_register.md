# Assumptions and temporary decisions register

This living register collects unconfirmed values, proposed defaults, mock
geometry, development-only shortcuts, and unresolved implementation choices
that could otherwise become accidental permanent requirements. It covers the
current repository, the autonomy specification, and the decisions recorded in
the development log.

Status labels used below are:

- **In use:** a provisional value is consumed by current code, configuration,
  visualization, or launch files.
- **Proposed:** a documented baseline exists, but it has not been validated on
  the intended robot or competition field.
- **Unresolved:** the decision or value is still absent; dependent production
  work must remain disabled or fail closed.
- **Development only:** the value exists to support a mock, bag replay, test,
  or visualization and is prohibited from physical control or safety use.

When an assumption is replaced, record the date, source, measurement method,
units, uncertainty where relevant, supporting artifact, reviewed configuration
change, and Git commit. Do not delete its history; move it to **Resolved
assumptions**. The broader dependency-oriented list remains in
[`tbd_register.md`](tbd_register.md).

## Active assumptions

### Platform, compute, and power

- **ASM-PLAT-01 — Proposed robot software platform.** **Proposed:** Jetson Orin
  Nano 8 GB, JetPack 6.2.2, L4T 36.5.0, Ubuntu 22.04, kernel 5.15, CUDA 12.6,
  ROS 2 Humble, ZED SDK 5.2.3, ZED ROS 2 wrapper tag `v5.2.2`, and `aarch64`
  are the current compatibility baseline. They are not a validated installed
  manifest. Close this after flashing the physical Jetson, pinning the wrapper
  commit, capturing every installed version/image digest, and completing a
  target-runtime test. Source: [`software_platform_lock.md`](resources/software_platform_lock.md).

- **ASM-PLAT-02 — Development-host evidence.** **Development only:** successful
  x86_64/ROS 2 Jazzy builds and tests are structural development evidence, not
  proof that arm64/ROS 2 Humble, JetPack, the ZED SDK, DDS, or timing behavior
  works on the target. Close this gap with reproducible arm64 CI and physical
  Jetson/Humble results.

- **ASM-POWER-01 — Jetson robot power.** **Unresolved:** the proposed 19 V
  barrel-jack supply path still lacks a verified connector size, polarity,
  current capacity, regulation, fuse/converter selection, and bench-test
  record. The Jetson-Nano-labelled adapter in the BOM must not be connected to
  the Orin on assumption alone.

- **ASM-PERF-01 — Initial competition runtime profile.** **Proposed:** 720p or
  lower ZED RGB/depth, 15 FPS depth, 10–15 FPS AprilTag processing, no full
  dense cloud, disabled ZED positional tracking/object detection, competition
  debug images disabled, selective rosbag topics, and RViz offboard are
  unprofiled starting choices. Confirm them from representative Orin load,
  bandwidth, perception-range, and accuracy tests.

- **ASM-PERF-02 — Provisional resource acceptance limits.** **Proposed:** a
  30-minute representative run, no OOM or sustained thermal throttling,
  10–15% shared-memory headroom, control/EKF rates within 90% of target, depth
  hazard latency below 150 ms, command age below its safety timeout, and no
  unbounded queues are initial engineering targets. Replace or ratify them
  using `tegrastats`, topic-rate/bandwidth, callback-latency, dropped-frame,
  and costmap-timing evidence from the final stack.

### Physical robot geometry and model

- **ASM-GEOM-01 — Synthetic chassis dimensions.** **Development only:** the
  mock chassis is `0.90 x 0.60 x 0.25 m`. It is used by the Xacro model and
  replay marker, but is not a CAD or measured robot dimension. Replace it with
  reviewed mechanical drawings or measurements. Source:
  [`mock_geometry.yaml`](../lb_model/config/mock_geometry.yaml).

- **ASM-GEOM-02 — `base_link` origin and collision footprint.** **Development
  only:** the mock places `base_link` at the synthetic chassis geometric center,
  and the replay footprint is only the `0.90 x 0.60 m` chassis rectangle. The
  physical origin, ground projection, clearance, wheel protrusion, bucket and
  mechanism swept envelopes, and footprint changes by mechanism state remain
  unknown. Do not use the current rectangle for Nav2 collision checking.

- **ASM-GEOM-03 — Synthetic wheels.** **Development only:** each mock wheel has
  radius `0.12 m`, width `0.08 m`, front/rear X positions `+/-0.30 m`,
  left/right Y positions `+/-0.34 m`, and center Z `-0.17 m`. The controller
  consequently uses `0.12 m` radius and `0.68 m` separation. Replace these with
  loaded effective radius, effective skid-steer separation, placement, gear
  ratio, encoder source/resolution, and direction-sign calibration.

- **ASM-GEOM-04 — Scoop proxy.** **Development only:** the fixed mock bucket is
  `0.28 x 0.48 x 0.16 m` at `[0.56, 0.00, -0.02] m`. It does not define the
  actual bucket, pivot, axes, linkage, actuator stroke, travel limits, digging
  depth, dumping pose, or swept collision envelope. Replace it with reviewed
  mechanism CAD and measured limit/clearance tests.

- **ASM-GEOM-05 — Mock sensor bodies and mounts.** **Development only:** the
  Xacro uses a ZED body `0.08 x 0.04 x 0.03 m` at `[0.18, 0, 0.18] m`, webcam
  body `0.06 x 0.03 x 0.03 m` at `[0.30, 0, 0.12] m`, and LiDAR mount at
  `[0, 0, 0.20] m`, with optical RPY `[-pi/2, 0, -pi/2]`. These are visual
  placeholders, not physical extrinsics. Replace each with the final rigid
  mount transform, mounting-repeatability evidence, and one documented TF
  authority.

- **ASM-GEOM-06 — Missing physical envelopes.** **Unresolved:** ground
  clearance, maximum width/length/height, center of mass, cable protrusions,
  wheel and chassis exclusion geometry, transport configuration, and every
  mechanism-state envelope have not been recorded. These are required before
  final footprints, inflation, keepouts, stability limits, and sensor
  self-filtering can be approved.

- **ASM-GEOM-07 — Model-frame and physics simplification.** **Development
  only:** the mock omits `base_footprint`, mass, inertia, contact/friction, and
  drivetrain dynamics, and configures the mock hardware with dynamics disabled.
  It is therefore a TF/visual/interface model rather than a physics-accurate
  simulator. Decide whether Nav2 needs a gravity-projected `base_footprint`,
  then derive dynamics and terrain interaction from reviewed CAD and tests
  before claiming simulated motion represents the physical robot.

### Drive, odometry, and motor communication

- **ASM-DRIVE-01 — Mock update and publication rates.** **Development only:**
  the Phase 2 controller-manager rate is `50 Hz`, wheel-odometry publish rate
  is `30 Hz`, and the older joint-state visualization profile uses `30 Hz`.
  These are test cadences, not guaranteed hardware/control rates. Select final
  rates from transport latency, encoder sampling, local-EKF needs, compute
  measurements, and stopping behavior.

- **ASM-DRIVE-02 — Mock differential-drive corrections and odometry noise.**
  **Development only:** wheel-separation and left/right radius multipliers are
  all `1.0`, the velocity rolling window is `1`, and pose/twist covariance
  diagonals are all zero. Zero covariance must never be interpreted as perfect
  physical odometry. Calibrate straight travel and turning on representative
  terrain, then fit multipliers and nonzero covariance.

- **ASM-DRIVE-03 — Mock motion limits.** **Development only:** linear velocity
  is limited to `+/-0.50 m/s`, linear acceleration to `+/-0.50 m/s^2`, linear
  jerk to `+/-1.00 m/s^3`, angular velocity to `+/-1.00 rad/s`, angular
  acceleration to `+/-1.00 rad/s^2`, and angular jerk to `+/-2.00 rad/s^3`.
  These are not physical or safety limits. Replace them using drivetrain,
  stability, traction, stopping-distance, perception-latency, and mechanism
  tests.

- **ASM-DRIVE-04 — Mock command timeout.** **Development only:** the drive
  controller uses a `0.25 s` `cmd_vel` timeout as a stale-command exercise.
  It resembles the proposed safety inhibit age but is not yet demonstrated to
  stop the physical robot safely. Tune and validate the complete command,
  heartbeat, controller, and braking path.

- **ASM-DRIVE-05 — Real drive transport.** **Unresolved:** no MCU role,
  Jetson/USB-to-CAN/MCU/SPARK MAX topology, CAN bitrate/IDs, controller mode,
  packet format, units, command rate, heartbeat, acknowledgement, reconnect
  policy, encoder scaling/timestamps, fault fields, or enable/stop/reset
  semantics has been approved. `RealDriveTransport` is intentionally
  unavailable and must remain fail closed until the protocol is reviewed.

### Arena, zones, and static map

- **ASM-MAP-01 — Nominal arena dimensions.** **In use:** the scene outline is
  `6.88 x 5.00 m`, based on the 2027 guidebook figure; guidebook prose describes
  the arena as approximately `6.8 x 5.0 m`. These values are not a team survey.
  Record competition-defined reference surfaces and measured dimensions with
  uncertainty before field alignment.

- **ASM-MAP-02 — Cell-aligned arena length discrepancy.** **In use:** the
  occupancy-grid interior is `6.85 x 5.00 m` because it uses 137 by 100 cells
  at `0.05 m/cell`, while the RViz scene uses the guidebook's `6.88 m` length.
  The `0.03 m` difference must be deliberately reconciled after survey; it must
  not be hidden by changing only one artifact.

- **ASM-MAP-03 — Map origin and axes.** **In use:** the intended physical
  origin is the starting-zone arena-wall corner, `+X` runs along the long axis
  away from the starting zone, `+Y` runs across the arena, `+Z` is up, and zero
  yaw faces `+X`. The REP-103 convention is deliberate, but the physical
  control point and repeatable alignment/survey method remain unverified.

- **ASM-MAP-04 — Occupancy raster representation.** **In use:** the map uses
  `0.05 m/cell`, YAML origin `[-0.05, -0.05, 0]`, one occupied cell outside
  each interior edge, trinary mode, occupied threshold `0.65`, and free
  threshold `0.196`. Resolution, perimeter treatment, thresholds, and whether
  additional permanent keepouts belong in the static map are initial choices,
  not field-calibrated values.

- **ASM-ZONE-01 — Starting-zone geometry.** **In use for visualization only:**
  the starting zone is drawn as `2.0 x 2.0 m`. Only the `2.0 m` X extent is
  numerically labeled in the guidebook; the Y depth is inferred from the
  approximately square dashed drawing. It is intentionally dashed and must
  not become a costmap or mission boundary until an official dimension or
  physical measurement is recorded.

- **ASM-ZONE-02 — Excavation boundary.** **In use for visualization only:** the
  boundary is drawn at `x=2.50 m` from the guidebook figure. Verify it against
  the marked competition field before using it for mission transitions,
  excavation permission, keepouts, or scoring logic.

- **ASM-BERM-01 — Target berm geometry.** **In use for visualization only:**
  the berm target is `2.20 x 0.90 m`, centered at map `(5.38, 0.90) m`. Its
  dimensions come from the guidebook, but its computed map position depends on
  the unsurveyed origin/wall reference. Survey the scoring marks before using
  it to generate an autonomous deposition goal.

- **ASM-ARENA-01 — Central column omission.** **Unresolved:** the guidebook
  identifies a permanent column but provides no trusted numerical pose or
  collision dimensions. It is deliberately absent from the scene and static
  map. Measure its pose, shape, clearance, and uncertainty before adding it to
  visualization or Nav2.

- **ASM-ARENA-02 — Perimeter and fixed collision geometry.** **Unresolved:**
  the exact interior reference surface, wall/PVC collision thickness and
  height, corners, rail supports, keepouts, and robot/mechanism clearance from
  them are not encoded. The guidebook's nominal perimeter-pipe information is
  insufficient without a field-reference decision and survey.

- **ASM-ARENA-03 — Remaining zone boundaries.** **Unresolved:** any exact
  construction-zone polygon, transition line, restricted region, or
  mechanism-specific keepout not already listed above still lacks surveyed
  coordinates and control semantics. Arena drawing pixels must not be used as
  authoritative coordinates.

- **ASM-ARENA-04 — Rocks, craters, and regolith.** **Intentional static-map
  omission:** changing boulders, craters, piles, and excavation effects are not
  baked into the static occupancy grid. Their precise size and location are
  runtime perception inputs. What remains temporary is the unimplemented
  sensing, hazard-envelope, persistence, and failure policy—not the decision
  to keep dynamic obstacles out of the static map.

### Cameras, AprilTags, and LiDAR

- **ASM-CAM-01 — Webcam identity and source rate.** **In use for bench work:**
  the Logitech webcam defaults to by-id device
  `/dev/v4l/by-id/usb-046d_0825_961026E0-video-index0`, camera name
  `uvc_camera_(046d:0825)`, `640 x 480`, raw transport, and a verified-at-launch
  `15 Hz` V4L2 request. Validate stable physical device identity, permissions,
  controls, USB topology, actual timestamps/rate, latency, dropped frames, and
  reconnect behavior on the target.

- **ASM-CAM-02 — Archived webcam calibration.** **Proposed:** the current
  `640 x 480` intrinsics/distortion were copied from the prior project with
  user confirmation; only the camera-name metadata changed. Revalidate them
  with the actual camera, focus, resolution, mount, calibration target, and
  measured reprojection/pose error before competition localization.

- **ASM-TAG-01 — Detector baseline.** **In use for bench work:**
  `tag36h11`, PnP, raw input, one thread, decimate `2.0`, blur `0.0`, refine
  enabled, sharpening `0.25`, debug disabled, and max hamming `0` are archived
  baseline choices rather than competition tuning. Tune detection range,
  false-positive/rejection behavior, latency, CPU load, blur, lighting, and
  motion performance from recorded and field data.

- **ASM-TAG-02 — Bench and field tag sizes.** **Partly confirmed/provisional:**
  bench tag ID 0 uses a user-confirmed `0.250 m` detector-corner edge. Planned
  field IDs 1–3 use nominal `0.300 m` detector-corner edges supplied by the
  team, but the physical tags have not been printed/measured. Confirm that
  dimensions exclude the paper border and record measured edge/print error.

- **ASM-TAG-03 — Planned field-tag inventory and placement.** **Unresolved:**
  the template reserves `tag36h11` IDs 1, 2, and 3, but final count, legal rail
  locations, camera allocation, visibility analysis, frame names, six-DoF
  poses, orientation, uncertainty, mounting rigidity, and survey procedure are
  not approved. Poses remain `null` in
  [`tag_map.template.yaml`](../lb_localization/config/tag_map.template.yaml).

- **ASM-TAG-04 — Replay Tag 1.** **Development only:** bag replay assumes field
  tag ID 1, size `0.300 m`, center `[0.0, 1.0, 0.75] m`, and map RPY
  `[+pi/2, 0, +pi/2]`, with the printed face normal toward arena `+X`. This is
  used by the arena marker and pose-replay transform math, not by production
  localization. Replace it with the measured, legally mounted, surveyed pose
  and convention-validation evidence.

- **ASM-CAM-03 — Replay webcam extrinsic.** **Development only:** replay uses
  `base_link -> webcam_optical_frame` translation `[0, 0, 0.40] m` and RPY
  `[-pi/2, 0, +pi/2]`, placing the rear-looking camera directly above the
  temporary base origin. Measure the final rigid mount in all six degrees of
  freedom, timestamp behavior, mounting repeatability, and uncertainty.

- **ASM-ZED-01 — Replay ZED extrinsic.** **Development only:** visualization
  colocates the forward-facing ZED with the webcam at `[0, 0, 0.40] m`, with
  optical RPY `[-pi/2, 0, -pi/2]`. This is not a calibrated transform and must
  not be reused for depth, IMU, self-filtering, or localization.

- **ASM-ZED-02 — Physical ZED integration.** **Unresolved:** although the ZED
  Mini model is selected by the BOM, its specific unit/serial, firmware, USB
  port/bandwidth, wrapper parameters, RGB/depth/IMU resolutions and rates, QoS,
  timestamps, calibration, mounting transform, frame authority, and measured
  target-compute load are not validated.

- **ASM-LIDAR-01 — 2D LiDAR selection and setup.** **Unresolved:** there is no
  selected LiDAR model, driver, connection, power path, scan rate, angular
  coverage, min/max range, filter, mount transform, blocked-sector mask,
  robot self-filter, timestamp validation, or failure behavior. The specified
  5–10 Hz minimum remains only a provisional architecture target.

### Localization and bag replay

- **ASM-LOC-01 — Local wheel/IMU EKF.** **Unresolved:** wheel odometry and ZED
  IMU inputs do not yet exist in the stack. Final sensor mappings, axes/yaw
  signs, magnetic assumptions, covariances, differential/relative modes,
  `two_d_mode`, frequency, timeouts, and rejection thresholds require
  recorded-data and motion validation. The proposed rates—wheel odometry at
  least 50 Hz, IMU 50–100 Hz, and local EKF 30–50 Hz—are unvalidated targets.

- **ASM-LOC-02 — AprilTag quality gates.** **Unresolved:** maximum tag distance
  and view angle, minimum decision margin, hamming limit, pose/yaw jump limits,
  detection age, minimum valid tags/pixel size, TF timestamp tolerance, future
  skew, fusion window, jump-reference timeout, and inter-tag position/yaw
  spread are all `TBD` in
  [`tag_localizer.template.yaml`](../lb_localization/config/tag_localizer.template.yaml).

- **ASM-LOC-03 — AprilTag covariance model.** **Unresolved:** position/yaw
  noise floors and distance scaling, roll/pitch variance, Z scaling, distance
  and view-angle quality gains, reference decision margin/pixel size, and
  covariance validation against ground truth remain `TBD`. Synthetic unit-test
  coefficients are test fixtures and must never become production defaults.

- **ASM-LOC-04 — Global EKF.** **Unresolved:** the global EKF template has no
  validated frequency, sensor timeout, publish-TF setting, input config
  vectors, input covariances, rejection thresholds, or time-offset handling.
  It remains non-runnable until local odometry exists and tag corrections are
  validated. The planned sole `map -> odom` ownership is a confirmed
  architecture rule, not an assumption.

- **ASM-LOC-05 — Localization validity policy.** **Unresolved:** acceptable
  covariance, tag-correction age, pose discontinuity, relocalization behavior,
  degraded speed, multi-tag agreement, and conditions for stopping or
  resuming autonomy have not been experimentally selected.

- **ASM-REPLAY-01 — Bag-replay freshness and history.** **Development only:**
  the replay adapter uses a `0.50 s` stale timeout, `0.05 s` poll period, and
  maximum 500 path poses; the arena replay defaults to `0.25x` playback while
  the simpler workflow may use `1.0x`. These values support visualization,
  not production localization. Tune any production freshness behavior from
  measured camera/detector latency and safety needs.

- **ASM-REPLAY-02 — Pose through tag loss.** **Development only:** replay
  intentionally deletes the active robot markers and breaks the path when the
  tag is stale; it does not propagate wheel odometry or IMU. This correctly
  exposes the current sensor gap but does not define final localization loss
  behavior.

### Nav2, costmaps, and terrain perception

- **ASM-NAV-01 — Production footprint.** **Unresolved:** no approved Nav2
  footprint or robot radius exists. It must account for measured chassis,
  wheels, bucket/mechanism states, cable/protrusion clearance, localization
  uncertainty, and operational margin rather than copying the mock rectangle.

- **ASM-NAV-02 — Costmap defaults.** **Proposed:** `0.05 m/cell`, a local
  rolling window approximately 4–8 m square, update rate 5–10 Hz, publish rate
  2–5 Hz, obstacle persistence 0.5–2.0 s, and inflation based on footprint plus
  localization uncertainty are specification starting ranges only. Select
  global/local dimensions, layers, resolution, update/publish rates,
  transform tolerances, persistence, inflation, and keepout settings from
  final geometry, sensor range, compute, and field tests.

- **ASM-NAV-03 — Obstacle sources.** **Unresolved:** global/local costmaps do
  not yet have validated LiDAR, ZED depth, or terrain-hazard sources. Topic
  names, message types at each adapter, marking/clearing behavior, observation
  height/range, raytrace range, persistence, QoS, expected update rate, and
  stale-source response require implementation and testing.

- **ASM-NAV-04 — Planner, controller, and recoveries.** **Unresolved:** the
  Nav2 planner, controller, goal/progress checkers, behavior tree, replanning
  frequency, recovery behaviors, goal tolerances, speed profiles, and failure
  escalation have not been selected or tuned. No current RViz goal implies a
  safe autonomous motion path.

- **ASM-PER-01 — ZED terrain-hazard processing.** **Unresolved:** forward ROI,
  crop/downsample rates, ground model, obstacle height/size/slope thresholds,
  crater/drop-off evidence, synthetic-point representation, clustering,
  persistence, transform tolerance, latency budget, and conservative failure
  response have not been determined.

- **ASM-PER-02 — LiDAR obstacle filtering.** **Unresolved:** physical min/max
  range, permanently masked angles, chassis/mechanism return removal,
  invalid-return handling, marking/raytrace distances, and clearing policy
  await the selected sensor, mounting geometry, and arena evidence.

- **ASM-NAV-05 — Zone semantics.** **Unresolved:** the visual starting,
  excavation, and berm boundaries currently have no costmap or mission-control
  authority. Decide which become keepouts, preferred regions, goal constraints,
  or mission transition tests only after geometry and rules are confirmed.

### Safety, mechanisms, mission, and operations

- **ASM-SAFE-01 — Physical safety chain.** **Unresolved:** physical E-stop
  wiring independent of ROS/Jetson/Wi-Fi, propulsion enable, disable and safe
  reset, heartbeat/watchdog, latched fault behavior, restart behavior, and
  acknowledgement paths are not implemented or reviewed. Software fail-closed
  mocks do not replace this hardware safety path.

- **ASM-SAFE-02 — Provisional data-age thresholds.** **Proposed:** warning /
  motion-inhibit ages are `0.15/0.25 s` for final velocity command,
  `0.15/0.30 s` for wheel odometry, `0.10/0.25 s` for IMU, `0.50/1.00 s` for
  LiDAR, `0.50/1.00 s` for depth hazards, and `0.15/0.30 s` for local EKF.
  AprilTag age remains mission-dependent. Tune all thresholds from measured
  rates, jitter, end-to-end latency, braking distance, and degraded-mode tests.

- **ASM-SAFE-03 — Other safety limits.** **Unresolved:** battery warning/cutoff,
  motor current/temperature, Jetson temperature, pitch/roll, communication
  loss, mechanism limits, mission timeout, permissible localization
  uncertainty, speed derating, and fault-severity/reset rules lack reviewed
  values and verification procedures.

- **ASM-MECH-01 — Excavation/deposition mechanism.** **Unresolved:** the
  hardware design, actuators, sensors, controller, command/action interfaces,
  units, feedback, interlocks, limits, current/stall handling, safe state,
  timeouts, target depth/load, dumping behavior, and recovery sequence are not
  defined. The fixed scoop proxy cannot be used as an interface specification.

- **ASM-MISS-01 — Mission behavior.** **Unresolved:** goal poses, state
  transitions, zone thresholds, number/timing of cycles, excavation and
  deposition completion criteria, localization-confidence policy, retry
  counts, Nav2 failure handling, mechanism recovery, pause/resume/abort/reset,
  and operator handoff have not been validated.

- **ASM-OPS-01 — Command arbitration and teleoperation.** **Proposed:** the
  intended priority is physical disable, safety lock/zero, manual teleop, then
  Nav2; exact `twist_mux` configuration, message adapter if needed, teleop
  device, speed limits, enable workflow, network-loss behavior, and bench/test
  profiles remain unimplemented or unvalidated.

- **ASM-OPS-02 — Time, network, and evidence retention.** **Unresolved:** robot
  network configuration, clock synchronization, rosbag storage capacity,
  recording profiles, naming, retention/offload, competition logging, and the
  operator evidence-capture procedure still require an operations review.

## Confirmed decisions that are not temporary assumptions

- The robot has four driven wheels and uses skid/tank steering; mechanism
  motors must not be counted as extra drive wheels.
- ROS REP-103 axes are the coordinate convention. What remains unverified is
  the physical placement of frame origins and measured transforms.
- The global EKF is the sole future owner of `map -> odom`; the local EKF is
  the sole future owner of `odom -> base_link`; rigid robot mounts belong to
  `robot_state_publisher` unless a specific camera-wrapper transform authority
  is deliberately selected.
- Dynamic rocks, craters, piles, and excavation changes do not belong in the
  static map; they belong in runtime perception and costmap layers.
- Current synthetic unit-test fixtures require explicit test opt-in. Their
  numbers are not fallback production parameters and are not separately
  itemized above as robot assumptions.
- Arena-scene marker colors, line widths, label heights, and the 1 Hz durable
  marker republish period are presentation choices, not claims about physical
  arena geometry or control behavior.

## Resolved assumptions

- None has yet been closed with the complete physical, field, or target-runtime
  evidence required by this register.
