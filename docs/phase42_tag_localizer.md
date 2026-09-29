# Phase 4.2 known-AprilTag localizer

Phase 4.2 implements the software-only known-tag-localization boundary in
`lb_localization`. It does **not** complete Phase 4, establish a physical robot
pose, or start a global EKF. Its purpose is to make the eventual field
localization path explicit and testable before the tag survey, physical camera
extrinsics, ZED integration, and local odometry are available.

## Scope and safety boundary

The installed `tag_localizer` executable subscribes directly to configured
source-scoped AprilTag detection topics and publishes only:

| Interface | Type | Meaning |
|---|---|---|
| `/localization/apriltag_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | A current, accepted absolute robot-pose measurement in `map`; it is input to the future global EKF. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Counts, rejection reasons, cache state, fusion status, and age of the last accepted correction. |

It has no motor, command, service, action, or transform broadcaster. In
particular, it never publishes `map -> odom`, `odom -> base_link`, a camera
extrinsic, or a tag-world transform. The eventual global
`robot_localization` EKF remains the sole intended `map -> odom` authority;
`config/ekf_global.template.yaml` is deliberately nonlaunchable until Phase 3
provides validated `/odometry/local`.

The public `tag_localizer.launch.py` has empty required configuration-file
arguments. It fails before starting a node if paths are missing, and the node
rejects any configuration that is neither fully surveyed nor an explicitly
opted-in, all-synthetic test set. The synthetic fixtures can run only with the
explicit `allow_synthetic_test_data:=true` opt-in and are not included from
`lb_launch` bringup.

## Inputs and TF contract

The actual Jazzy `apriltag_msgs/msg/AprilTagDetectionArray` message contains a
family, ID, hamming value, decision margin, image corners, and homography. It
does not contain a metric pose. For each detection, the localizer therefore
requires an exact-time detector observation transform:

```text
<camera_optical_frame> -> <source>_observation_tag_<id>
```

and an exact-time (or static) measured camera extrinsic:

```text
base_link -> <camera_optical_frame>
```

For example, a future field run may use both of these independent source
contracts:

```text
/sensors/webcam/tag_detections
webcam_optical_frame -> webcam_observation_tag_1

/sensors/zed/tag_detections
zed_camera_optical_frame -> zed_observation_tag_1
```

The child-frame prefixes must be distinct. Two cameras seeing the same
physical tag must never publish the same observation child frame. A detector
TF lookup is requested at the detection header timestamp, not at TF "latest".
The node rejects an absent, frame-mismatched, zero-stamped, stale, or otherwise
timestamp-mismatched observation transform. A static rigid camera extrinsic is
permitted only after its survey status is recorded as `surveyed`.

For an accepted observation, the localizer computes:

```text
T_map_base = T_map_known_tag
             * inverse(T_camera_observed_tag)
             * inverse(T_base_camera)
```

The Phase 1 mock webcam frame is explicitly synthetic and is not an allowed
camera extrinsic for this computation.

## Tag inventory and map schema

The separate Phase 4.1 bench tag remains unchanged:

| Use | Family / ID | Detector-corner edge | Provenance |
|---|---|---:|---|
| Bench only | `tag36h11` / 0 | 0.250 m | User-confirmed physical test tag; not a field landmark. |

The team-selected future field inventory is:

| Planned use | Family / IDs | Nominal detector-corner edge | Status |
|---|---|---:|---|
| Field localization | `tag36h11` / 1, 2, 3 | 0.300 m each | Planned as of 2026-08-24; not printed, measured, mounted, surveyed, or validated. |

`lb_localization/config/tag_map.template.yaml` records that plan but is
intentionally invalid at runtime: every `pose_map` is `null` and its status is
`unverified`. It excludes ID 0. A valid future map requires unique
`(family, id)` values, unique frame IDs, a positive physical size, finite
`x/y/z/roll/pitch/yaw`, a shared `map` coordinate convention, and a
`surveyed` status. The `test/fixtures/tag_map_synthetic.yaml` poses are
invented test geometry only.

## Gates, covariance, and fusion

No operational gate or covariance default is hidden in the code. The template
requires the specification's gates:

- `max_tag_distance_m`
- `max_view_angle_deg`
- `min_decision_margin`
- `max_hamming`
- `max_pose_jump_m`
- `max_yaw_jump_rad`
- `max_detection_age_s`
- `minimum_valid_tags`

It also requires an image-size floor, transform timestamp tolerance, future
clock-skew limit, fusion window, jump-reference timeout, inter-tag
position/yaw disagreement limits, TF lookup timeout, and all covariance-model
coefficients. Values in the synthetic fixture exist only to exercise branches;
they are neither measured nor recommended competition values.

The fused output is deliberately planar: translation and yaw are estimated,
while roll and pitch are set to zero with the required,
explicitly-configured `unobserved_roll_pitch_variance_rad2` covariance. This
prevents an accidental claim that roll or pitch was estimated. The runtime
uses a dedicated TF listener thread so its configured nonzero lookup timeout is
valid on both ROS 2 Humble and Jazzy.

The node rejects unknown field tags (including bench ID 0), bad hamming or
decision margin, small tags, excessive range/view angle, stale/future data,
invalid transforms, and implausible fused jumps. A no-tag interval creates no
new pose message; it cannot refresh a last-known correction. A backwards ROS
time jump clears the pending observation cache and jump reference.

Candidates are retained only for the configured short fusion window and are
deduplicated by physical `(family, id)`, not camera. The best observation of a
shared tag is retained so front and rear cameras cannot falsely count one
landmark twice. Distinct valid tags are fused using inverse-covariance weights;
translation is averaged and yaw is circularly averaged for the planned 2-D
filter. The covariance model increases uncertainty with range, obliquity, low
decision margin, small pixel size, and inter-tag disagreement. Excessive
inter-tag disagreement rejects the fused correction rather than silently
moving the robot estimate.

## Files and verification

`lb_localization` owns the C++ core, ROS node, fail-closed launch,
configuration templates, synthetic fixtures, static contracts, and deterministic
unit tests. The tests cover tag-map parsing, test-only opt-in, transform
composition, two-camera extrinsic equivalence, quality gates, covariance
growth, inverse-covariance fusion, yaw wraparound, duplicate-tag protection,
inter-tag disagreement, jump rejection, and the absence of a TF broadcaster.

See [Phase 4.2 validation evidence](evidence/phase42_validation_2026-08-24.md)
for the development-host result. Required physical follow-up remains:

- Survey tag IDs 1–3 and their uncertainty in the final arena/map convention.
- Print and measure the markers' detector-corner edges and validate detector
  configurations against them.
- Measure and record each rigid camera extrinsic and select exactly one TF
  authority per camera frame.
- Revalidate the webcam calibration and integrate/profile the ZED on the
  locked Jetson/Humble image.
- Tune gates and covariance from recordings, then validate the Phase 3 local
  EKF and the future global-EKF `map -> odom` authority with runtime TF tests.
