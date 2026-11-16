# Provisional arena-pose replay evidence — 2026-10-12

## Scope

This increment visualizes a raw map-frame webcam pose and a planar robot proxy
from the recorded `bags/test1_AT1` AprilTag-1 observation. It adds no production
localization, TF ownership, odometry, Nav2 planning/control, or hardware output.

## Geometry and provenance

- Map convention: REP-103, starting-zone corner origin, arena `+X` away from
  the starting wall, `+Y` across the arena, and `+Z` up.
- Estimated tag-1 center: `[0.0, 1.0, 0.75]` m, provided by the user for this
  development replay and not surveyed.
- Tag front/outward normal: arena `+X`, provided by the user.
- The current `apriltag_ros` PnP implementation constructs the emitted tag
  frame through OpenCV object/image corner correspondence; see the upstream
  [pose-estimation source](https://github.com/christianrauch/apriltag_ros/blob/master/src/pose_estimation.cpp).
  The actual bag's near-frontal sample was translation
  `[-0.061, -0.007, 1.745]` m with RPY
  `[-2.973, -0.153, 0.578]` rad. The approximately pi roll establishes that
  the emitted upright tag axes are `+x` image-right, `+y` image-up, and `+z`
  out of the printed face/toward the camera. This differs from the underlying
  AprilTag library's raw-coordinate description and is why the recorded TF,
  rather than the raw-library convention, is authoritative here.
- Therefore an upright tag facing arena `+X` uses map RPY
  `[+pi/2, 0, +pi/2]` rad: tag `+x -> map +Y`, tag `+y -> map +Z`, and tag
  `+z -> map +X`.
- Estimated `base_link -> webcam_optical_frame`: translation
  `[0.0, 0.0, 0.40]` m and rear-facing optical RPY
  `[-pi/2, 0, +pi/2]`. The height is the user-selected temporary value; X/Y
  coincidence and exact rotation remain development assumptions.

The adapter computes:

```text
T_map_camera = T_map_tag * inverse(T_camera_tag_observation)
T_map_base   = T_map_camera * inverse(T_base_camera)
```

It publishes the full camera pose, then projects the base result to XY/yaw with
`z=0` for the robot proxy. The bag was recorded while the camera was hand-moved,
so the proxy is not a test of chassis dynamics or localization accuracy.

## Components and interfaces

- `lb_localization/scripts/tag_replay_robot_pose.py` owns transform composition,
  timestamp freshness, bounded path segmentation, expiring markers, and replay
  diagnostics.
- `lb_sim/launch/arena_pose_replay.launch.py` requires an explicit bag, uses
  `/clock`, optionally includes the static map launch, and optionally starts
  the reusable RViz profile.
- `lb_navigation/rviz/arena_pose_replay.rviz` shows `/map`, recorded imagery,
  assumed tag geometry, raw camera axes, the planar robot arrow, and its current
  contiguous path.
- Topics are `/visualization/replay/webcam_pose_raw` (`PoseStamped`),
  `/visualization/replay/base_pose_planar` (`PoseStamped`),
  `/visualization/replay/base_path_planar` (`Path`),
  `/visualization/replay/markers` (`MarkerArray`), and `/diagnostics`.
- The node has no TF broadcaster and does not publish
  `/localization/apriltag_pose`, `map -> odom`, or `odom -> base_link`.

## Failure and safety behavior

The latest TF sample is rejected after the configurable 0.5 s timeout. Loss
deletes the dynamic camera/base markers and clears the current path. A new
detection begins a new segment; backward `/clock` motion also clears state.
There is no interpolation or dead reckoning through a gap. Pose messages cannot
be retracted from arbitrary subscribers, so RViz intentionally renders the
expiring markers rather than the last-value pose displays. The launch starts no
controller, command topic, EKF, planner, or hardware component.

## Verification

- Focused transform, planarization, loss/reacquisition, clock-reset, and launch
  contract tests: 12 passed.
- Jazzy `lb_localization`, `lb_navigation`, and `lb_sim` build completed.
- Selected package suites reported 41 tests, zero errors, zero failures, and
  zero skips (26 localization, 8 navigation, and 7 simulation results).
- New Python sources passed `ament_flake8` and `ament_pep257`; repository diff
  whitespace validation passed.
- The installed launch resolved every argument and installed artifact path.
- A headless, map-disabled, 2x loop of the actual bag established the ROS graph
  and all four visualization topics. A captured corrected planar sample was
  `(x, y, z) = (1.815, 0.782, 0.0)` m with yaw approximately `0.084` rad in
  `map`, placing the proxy on the positive-X arena side of the starting wall.
- A live `LOST` diagnostic occurred at 0.52 s observation age and reported a
  zero-length cleared path. Loop rewind produced `CLOCK_RESET`; Ctrl+C then
  stopped both the player and node cleanly.

The first live sample using the raw AprilTag-library axis interpretation landed
at negative X with yaw near pi. Inspection of the bag's emitted PnP transform
showed the approximately pi camera/tag roll described above. The tag roll was
corrected from `-pi/2` to `+pi/2`, unit-axis contracts were updated, and the
positive-X sample was then captured. This is evidence for transform convention,
not physical accuracy.

## Limitations and remaining work

The node does not apply detection decision-margin gates, estimate covariance,
fuse multiple tags, or propagate pose without a tag. Tag pose, mount pose, map
alignment, and visual output remain physically unverified. Required follow-up
includes graphical inspection of the three recorded movements/dropout,
surveying the tag, measuring the webcam extrinsic, adding wheel odometry and ZED
IMU, and validating the production local/global EKF chain. The runtime check
disabled `map_server` because the host still lacks the declared Nav2 map-server
packages; therefore combined `/map` publication and graphical RViz remain
operator checks after dependency installation.

Git evidence: uncommitted implementation based on `553ecb8` (`Phase 2
complete`); replace this line with the resulting commit after review.
