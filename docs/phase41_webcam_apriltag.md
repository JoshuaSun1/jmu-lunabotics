# Phase 4.1 webcam AprilTag detection

Phase 4.1 establishes a non-actuating, calibrated webcam-to-AprilTag
observation pipeline. It is a bench baseline for the later known-tag
localizer; it is not Phase 4 global localization.

The implementation deliberately does **not** add a physical
`base_link -> webcam_optical_frame` extrinsic, a tag map, a tag localizer, an
EKF, `map -> odom`, `odom -> base_link`, or a robot pose topic. A detected tag
therefore proves only that this camera observed that tag in the camera frame.

## Components and ownership

| Component | Package/files | Responsibility |
|---|---|---|
| Webcam acquisition | `lb_sensors/launch/webcam_apriltag.launch.py` (`v4l2-ctl` preflight + `v4l2_camera`) | Fail closed unless V4L2 accepts and reports the configured 15 Hz source rate; then open one explicitly selected V4L2 device and publish 640 x 480 RGB images. |
| Calibration record | `lb_sensors/config/webcam_calibration.yaml` | Store the inherited, user-confirmed 640 x 480 camera model. |
| Rectification | `image_proc/rectify_node` | Rectify the source image using the matching `CameraInfo`. |
| Detector | `apriltag_ros/apriltag_node` and `lb_sensors/config/webcam_apriltag.yaml` | Detect the configured tag family and publish camera-relative observation data/TF. |
| Public entry point | `lb_launch/launch/bringup.launch.py` | Includes the webcam pipeline only when `enable_apriltags:=true`, while suppressing the synthetic Phase 1/2 mock bench so it cannot attach the observation to mock robot geometry. The default remains false. |

`lb_sensors` owns the driver, calibration, rectification, detector, and
camera-scoped interfaces. `lb_localization` is intentionally untouched by this
phase; it will own the later surveyed-tag-map/localizer and EKF configuration.

## Data flow and interfaces

```text
stable /dev/v4l/by-id webcam path
              |
              v
v4l2-ctl preflight (--set-parm=15, then --get-parm verification)
              |
              v
v4l2_camera (640 x 480, webcam_optical_frame)
   | image_raw + camera_info
   v
image_proc/rectify_node
   | image_rect + matching camera_info
   v
apriltag_ros
   | tag_detections
   `-- dynamic webcam_optical_frame -> webcam_observation_tag_0
```

The source-scoped names prevent a later ZED stream from silently colliding
with webcam data:

| Interface | ROS type | Producer | Consumer / meaning |
|---|---|---|---|
| `/sensors/webcam/image_raw` | `sensor_msgs/msg/Image` | `v4l2_camera` | Raw RGB image; `header.frame_id` is `webcam_optical_frame`. |
| `/sensors/webcam/camera_info` | `sensor_msgs/msg/CameraInfo` | `v4l2_camera` | Archived, user-confirmed 640 x 480 camera model paired with camera timestamps. |
| `/sensors/webcam/image_rect` | `sensor_msgs/msg/Image` | `image_proc/rectify_node` | Rectified detector input. |
| `/sensors/webcam/tag_detections` | `apriltag_msgs/msg/AprilTagDetectionArray` | `apriltag_ros` | Detection family, ID, hamming value, decision margin, image-space centre/corners, and homography. It is not a robot pose message. |
| `/tf` | `tf2_msgs/msg/TFMessage` | `apriltag_ros` | Newly stamped dynamic `webcam_optical_frame -> webcam_observation_tag_0` samples while tag 0 is observed; future consumers must reject cached stale samples. |

The installed `apriltag_msgs/msg/AprilTagDetectionArray` carries detection
quality and image-space geometry, not a metric pose field. The configured
detector observation TF is the camera-relative pose expression. Its parent is
the image header frame, and its child must never be interpreted as a surveyed
world tag or as a substitute for a tag-localizer output.

No services or actions are introduced in this phase. No unscoped
`/tag_detections`, `/localization/apriltag_pose`, `/odometry/local`, or
`/odometry/global` publisher is introduced either.

## Calibration and detector baseline

| Item | Value | Provenance and status |
|---|---|---|
| Image profile | 640 x 480 | User-confirmed archived calibration. The calibration is valid only for this profile. |
| Camera model | `plumb_bob`, values in `webcam_calibration.yaml` | Inherited from `old-lunabotics` for the same Logitech UVC webcam; no numeric calibration value was changed in Phase 4.1. |
| Calibration metadata name | `uvc_camera_(046d:0825)` | Renamed from the archival `narrow_stereo` label only, to match the confirmed V4L2 camera name. This is not a recalibration. |
| Acquisition rate | 15 Hz configured and verified by `v4l2-ctl` before `v4l2_camera` starts | Source-rate profile chosen to bound detector input. Launch aborts if V4L2 cannot set/verify a rate within 0.1 Hz of the requested 10–15 Hz value; detector throughput remains separately measured and the specification's 10–15 FPS detector target is not yet validated. |
| Tag family | `tag36h11` | User-confirmed bench family. `apriltag_ros` receives the family parameter as `36h11` and reports detections as `tag36h11`. |
| Known bench tag | ID `0` | Current bench setup only; it does not choose the competition tag inventory. |
| Tag size | 0.250 m | User-confirmed detector-corner edge length, not the outside paper edge. Recheck before competition use. |
| Hamming acceptance | `max_hamming: 0` | Explicit Phase 4.1 quality baseline. |
| Pose method | PnP | `apriltag_ros` baseline configuration; metric accuracy is not yet claimed. |
| Detector tuning | one thread, decimate 2.0, blur 0.0, refine enabled, sharpening 0.25 | Archived baseline settings, not experimentally tuned competition values. |

The calibration must be revalidated against a taped/measured target distance
before any metric use. The 0.250 m tag edge and archived intrinsics alone do
not establish distance or angular accuracy, uncertainty, timestamp quality, or
camera-to-robot geometry.

## TF and multi-camera boundary

`apriltag_ros` owns only a dynamic observation transform:

```text
webcam_optical_frame -> webcam_observation_tag_0
```

Newly stamped samples exist only while the configured tag is detected. A TF
listener can retain the last dynamic sample after loss, so any future consumer
must reject a transform whose timestamp exceeds its staleness limit. The edge
has no `map` parent, does not publish `map -> odom`, and does not establish a
fixed transform from the robot to the camera. The physical
`base_link -> webcam_optical_frame` mount transform, its measurement method,
uncertainty, and final authority are still open.

The existing Phase 1 model contains a **synthetic** webcam branch for visual
tests. It is not a physical extrinsic and must not be used by a localizer or
to derive robot pose. Before integrating the real camera with a physical robot
description, replace that mock-only geometry with a surveyed mount decision and
ensure exactly one publisher owns each rigid transform.

A later ZED (or another webcam) must use its own source-scoped image/detection
topics and a distinct observation-frame prefix, for example
`zed_observation_tag_0`. Two cameras must never publish the same dynamic child
frame for the same tag. The future `lb_localization/tag_localizer` will combine
camera-relative observations only after it has a surveyed tag map, calibrated
camera extrinsics, quality gates, and covariance rules.

## Safe behavior and failures

This launch has no `/cmd_vel` consumer, motor-enable path, power-control
interface, localizer, or EKF. Starting it cannot command propulsion and does
not weaken the independent physical E-stop requirement.

- The launch requires nonempty configuration and camera-device arguments; use
  a `/dev/v4l/by-id/...` path rather than unstable `/dev/videoN` enumeration.
- Before the ROS driver starts, the launch resolves the device and uses
  `v4l2-ctl` from `v4l-utils` to set and read back the requested 10–15 Hz
  frame rate. A missing utility, disconnected device, failed V4L2 operation,
  or a verified rate outside tolerance aborts the launch rather than silently
  running a faster source stream.
- A disconnected, permission-denied, or incorrectly selected camera results in
  absent image/detection data or a driver error. It must not be treated as a
  valid no-tag condition for autonomous behavior.
- A missing tag yields no new observation TF sample and no new detection.
  A TF buffer can retain its last dynamic sample, so a later localizer must
  reject stale timestamps rather than treat cached data as a current sighting.
  This phase does not synthesize a last-known pose, a `map` pose, or a
  covariance.
- A resolution/calibration mismatch invalidates metric interpretation. Keep
  the launch at the calibrated 640 x 480 profile until a new calibration is
  recorded.
- Rejected hamming values, poor decision margin, excessive range/view angle,
  timestamp staleness, multi-tag disagreement, and localizer health behavior
  are future `TAG-01`/`LOC-01` work; only `max_hamming: 0` is enforced here.

## Bench procedure

Perform this only with propulsion disabled and a printed `tag36h11` ID 0 tag
of the user-confirmed 0.250 m detector-corner edge length in view.

1. Build and source the workspace in a ROS environment that has `v4l-utils`,
   `v4l2_camera`, `image_proc`, and `apriltag_ros` installed:

   ```bash
   ./scripts/build.sh
   source ../../install/setup.bash
   ```

2. Confirm that the selected `/dev/v4l/by-id/...` symlink resolves to the
   intended webcam. Override `webcam_device:=...` only with another stable
   by-id path. The launch invokes `v4l2-ctl` itself and must report a verified
   15 Hz source rate before it starts `v4l2_camera`.

3. Start the standalone, non-actuating pipeline:

   ```bash
   ros2 launch lb_sensors webcam_apriltag.launch.py
   ```

   The sensor-only public equivalent is
   `ros2 launch lb_launch bringup.launch.py enable_apriltags:=true`. In that
   mode, `lb_launch` suppresses the Phase 1/2 mock bench and its synthetic
   camera transform, so the detector observation cannot be attached to mock
   robot geometry. Do not combine this Phase 4.1 bench with a physical or
   mock robot-localization claim.

4. In separately sourced terminals, inspect the calibrated camera information,
   an expected ID-0 detection, the topic rate, and the observation transform:

   ```bash
   ros2 topic echo /sensors/webcam/camera_info --once
   ros2 topic echo /sensors/webcam/tag_detections --once
   ros2 topic hz /sensors/webcam/tag_detections
   ros2 run tf2_ros tf2_echo webcam_optical_frame webcam_observation_tag_0
   scripts/record_bag.sh --profile webcam_apriltag
   ```

   Newly stamped TF output should arrive while the tag is visible. A TF client
   may retain the last sample after the tag is lost, so record transform
   timestamps and treat a stale result as invalid. Record frame IDs, detection
   ID/family/hamming/decision margin, observed rates, terminal output, and any
   image/TF diagnostics in the evidence file or the `webcam_apriltag` rosbag
   profile. Do not label these values as accuracy measurements without a
   separately recorded distance/angle reference.

5. Stop the launch before disconnecting the camera. Preserve a rosbag and
   screenshots/logs for any later comparison, especially after changing cable,
   port, resolution, camera settings, tag print, or detector parameters.

## Validation status and follow-up

Phase 4.1 includes offline launch/configuration contracts and a development
host webcam bench procedure. Its validation evidence is maintained in
[`evidence/phase41_validation_2026-08-17.md`](evidence/phase41_validation_2026-08-17.md).
It is not a validation of the locked Humble/Jetson/Orin runtime, the ZED, a
physical robot mount, metric pose accuracy, global localization, or autonomous
safety behavior.

The launch verifies a 15 Hz V4L2 **configuration readback** before the driver
opens. A full-image development-host bag recorded post-driver raw output at
14.995 Hz, but its rectified/detection counts were affected by image-recording
load and are not a detector benchmark. Separately, a low-overhead
observation-rate sample varied between approximately 9.17 and 11.86 Hz. This
does not demonstrate the specification's 10–15 FPS detector target (the
metadata-bag result is below it). Repeat the measurement on the selected
Jetson under defined range, angle, lighting, and load conditions before
claiming detector-rate compliance.

Remaining work includes a measured camera-to-tag accuracy study, a surveyed
physical webcam mount, final multi-tag inventory and map, quality/covariance
gates, source adapters for additional cameras, `lb_localization/tag_localizer`,
local/global EKFs, and target-hardware profiling.

**Implementation Git commit:** `PENDING — update after the Phase 4.1 code and
documentation commit is created.`
