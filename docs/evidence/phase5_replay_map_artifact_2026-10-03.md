# Replay visualization and initial arena artifact evidence — 2026-10-03

## Scope

This increment adds non-actuating replay visualization for the recorded
`bags/test1_AT1` AprilTag-1 run and creates the initial static occupancy image.
It deliberately does not configure `nav2_map_server`, Nav2, global
localization, or physical motion.

## Inputs and provenance

- Bag: Jazzy rosbag2 MCAP, 109.852 s, 6,264 messages.
- Recorded interfaces: 1,637 raw images, 1,637 camera-info messages, 1,636
  rectified images, 677 `apriltag_msgs/msg/AprilTagDetectionArray` messages,
  and 677 `/tf` messages.
- Reported operator maneuvers: increase camera/tag distance, move laterally
  and obliquely, lose the tag, then reacquire it. These maneuvers have not yet
  been measured from a live replay in this restricted environment.
- Guidebook-derived arena width: 5.00 m. Team-selected working length: 6.85 m.
- Team-selected map convention: starting-zone corner at `(0, 0)`, +X along
  the long direction away from the starting zone, +Y along the short direction,
  +Z up, zero yaw toward +X.
- Assumed development-only tag-1 center: `(0.0, 1.0, 0.75)` m with its face
  normal toward +X. No tag quaternion or surveyed pose is asserted here.

## Components and behavior

`lb_localization/scripts/tag_replay_visualizer.py` looks up fresh
`webcam_optical_frame -> webcam_observation_tag_1` samples and publishes a
camera-relative `PoseStamped` and bounded `Path`. A 0.5 s initial stale timeout
and 500-pose bound are configurable, unmeasured development values. Loss clears
the active path, reacquisition starts a new segment, and backward simulated-time
jumps clear replay state. `/diagnostics` reports loss, reacquisition, and clock
reset. The node broadcasts no TF and does not infer a robot or map pose.

`lb_sim/launch/bag_replay.launch.py` requires an explicit bag directory, plays
it with `/clock`, starts the visualizer with simulated time, and optionally
opens `lb_navigation/rviz/apriltag_replay.rviz`. It starts no live camera,
detector, controller, map server, or Nav2 process.

The generated `lb_navigation/maps/arena.pgm` is 139 x 102 cells at 0.05 m/cell.
Its one-cell black perimeter lies outside a 137 x 100 white/free interior,
representing `x=[0.0, 6.85)` and `y=[0.0, 5.00)`. The YAML image origin is
`[-0.05, -0.05, 0.0]`. The permanent center column, keepouts, boulders,
craters, and transient obstacles are omitted; the column awaits survey.

## Verification

- Deterministic map generation completed.
- Direct focused pytest run: 8 passed.
- Jazzy selected-package build: `lb_localization`, `lb_navigation`, and
  `lb_sim` completed.
- Jazzy selected-package test result: 106 tests, 0 errors, 0 failures,
  1 skipped. The skip is an existing conditional runtime test.
- Direct `ament_flake8` for all new Python files: no problems found.
- Headless replay launch resolved the explicit bag, started rosbag playback at
  2x, and started the installed visualizer. DDS traffic could not be validated
  because the restricted sandbox denied network-interface/socket access.
- RViz was not opened because graphical validation requires the operator's
  desktop and DDS access.

Operator graphical smoke command:

```bash
cd /home/jsun3/dev_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lb_sim bag_replay.launch.py \
  bag_path:=$PWD/src/jmu-lunabotics/bags/test1_AT1 \
  playback_rate:=0.25
```

Confirm that distance, lateral pose, and orientation change; tag loss removes
the fresh pose/path; reacquisition begins a new segment; and looping clears the
path on the backward clock jump.

## Limitations and next work

The replay represents the tag relative to the camera. It is not camera motion,
robot localization, or `map -> base_link`. The camera extrinsic, tag pose and
orientation, map geometry, and center column remain unmeasured. Humble target
compatibility, resource use, dropout durations, pose extrema, and graphical
behavior require operator/runtime validation. The next increment may configure
the map server only after this artifact and coordinate convention are reviewed.

Git state: uncommitted change based on `1db17cb`.
