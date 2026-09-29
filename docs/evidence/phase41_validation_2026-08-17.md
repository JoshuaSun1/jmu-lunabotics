# Phase 4.1 webcam AprilTag validation evidence — 2026-08-17

## Scope

This record covers the non-actuating Phase 4.1 webcam acquisition,
rectification, and AprilTag-observation baseline. It does not validate a ZED,
a physical camera mount, a robot-relative pose, a surveyed tag map, a tag
localizer, either EKF, `map -> odom`, `odom -> base_link`, autonomous motion,
or physical propulsion.

## Configuration under test

| Item | Intended baseline |
|---|---|
| Camera | Connected Logitech UVC webcam (`046d:0825`) selected through a stable `/dev/v4l/by-id/...` path. |
| Acquisition | `v4l2-ctl`/`v4l-utils` fail-closed preflight configures and verifies 15 Hz before `v4l2_camera` opens the 640 x 480 YUYV source and produces `rgb8`. |
| Calibration | User-confirmed archived same-camera `plumb_bob` calibration in `lb_sensors/config/webcam_calibration.yaml`; only `camera_name` metadata changed from `narrow_stereo` to `uvc_camera_(046d:0825)`; numerical values are unchanged. |
| Pipeline | `v4l2_camera -> image_proc/rectify_node -> apriltag_ros`. |
| Tag | `tag36h11`, ID 0, 0.250 m detector-corner edge; `max_hamming: 0`. |
| Observation TF | Newly stamped dynamic `webcam_optical_frame -> webcam_observation_tag_0` samples only while detected; a future consumer must reject cached stale samples after loss. |
| Explicit exclusions | No physical base-to-camera extrinsic, no tag map/localizer/EKF, no `map`/`odom` TF, no robot global pose. |

## Pre-implementation bench evidence retained from discovery

The preceding discovery record in `resources/devlog.md` captured a direct
development-host webcam test before this launch profile existed. The webcam
captured 640 x 480 YUYV images at 30 Hz through a temporary V4L2 setup, and a
visible tag was detected as `tag36h11` ID 0 with hamming 0 and decision margin
115.7505. That camera stream had no loaded calibration, so it establishes only
capture/detection compatibility; it is **not** metric-pose evidence and is not
the final Phase 4.1 pipeline result.

## Development-host environment

| Item | Value |
|---|---|
| Host | x86_64 Ubuntu 24.04 development environment |
| ROS underlay used for these checks | ROS 2 Jazzy structural/bench override |
| Target runtime | arm64 Jetson / ROS 2 Humble remains unvalidated |
| Physical scope | Webcam and printed tag only; no motors, drive controller, battery, ZED, or physical robot mount tested |

## Final validation matrix

These are the recorded development-host results. They are not a target-runtime
or metric-accuracy result. An unvalidated target criterion is not a pass.

| Check | Command / observation | Final result |
|---|---|---|
| Python/launch syntax | `python3 -m py_compile lb_sensors/launch/webcam_apriltag.launch.py lb_launch/launch/bringup.launch.py` | Passed. |
| Repository build | `LUNABOT_ROS_DISTRO=jazzy ./scripts/build.sh` on the development host | Passed: 10 packages built. |
| Offline contract tests | `LUNABOT_ROS_DISTRO=jazzy ./scripts/test.sh --skip-build` after the recorded build | Passed: 10 packages, 70 tests, 0 failures, 1 expected skip. |
| Lint | `./scripts/lint.sh` in an environment that permits the configured hooks | Passed: end-of-file, whitespace, merge-conflict, YAML, private-key, executable-shebang, large-file, Black, and clang-format hooks. The restricted sandbox stalled Black's worker process; the completed result was obtained in unrestricted execution. |
| Recorder profile | `scripts/record_bag.sh --profile webcam_apriltag --dry-run` | Passed: selects the source-scoped raw/rectified image, camera-info, detection, and `/tf` topics without opening a camera or enabling any actuator. |
| Stable device selection | Resolve the configured `/dev/v4l/by-id/...` path and confirm its USB identity | Passed: stable Logitech UVC `046d:0825` path resolved to `/dev/video2` during this session. `/dev/video2` itself remains session-specific and is not the configured public identifier. |
| Fail-closed source-rate preflight | Launch the profile and retain the `v4l2-ctl --set-parm=15` / `--get-parm` output | Passed: V4L2 active rate read back as **15.000 FPS** before the driver started. |
| Calibrated camera information | `ros2 topic echo /sensors/webcam/camera_info --once` | Passed: `webcam_optical_frame`, 640 x 480, `plumb_bob`, and K/D values matched the calibration YAML. |
| Detector result | `ros2 topic echo /sensors/webcam/tag_detections --once` with the tag visible | Passed: `tag36h11`, ID 0, hamming 0, decision margin 124.291, camera header `webcam_optical_frame`. |
| Observation TF | `ros2 run tf2_ros tf2_echo webcam_optical_frame webcam_observation_tag_0` with the tag visible | Passed: dynamic camera-relative sample translation `[-0.143, 0.360, 2.051]` m. This is an observation sample, not an accuracy measurement or robot pose. |
| No invented global TF | Inspect the standalone launch/contract for localization components and global TF ownership | Passed for the Phase 4.1 boundary: it launches the camera/rectifier/detector only and contains no localizer, EKF, `map -> odom`, or `odom -> base_link` publisher. This is not a full physical-robot TF-authority test. |
| Rate/resource baseline | `ros2 topic hz /sensors/webcam/image_raw` and `/sensors/webcam/tag_detections`; record CPU/memory if available | The pre-driver V4L2 configuration read back as 15.000 FPS. The full-image rosbag captured 80 `image_raw` and 80 `camera_info` messages in 5.334999 s, or **14.995 Hz**, confirming post-driver raw output during that recording. That bag's 43 rectified images (8.06 Hz) and 18 detections (3.37 Hz) are **not** detector-performance results because writing full images materially loaded this development host. Separately, `ros2 topic hz` reported detected output of approximately **11.86 Hz** over a 111-sample window; an 8.068 s metadata-only rosbag captured 74 detection/TF messages, or **9.17 Hz**. Treat those detector samples as development-host bench evidence, not proof of a sustained 10–15 Hz target. A host `ps` snapshot reported camera/rectifier/detector RSS of 80,896 / 80,088 / 113,388 KiB and CPU of 8.8% / 13.1% / 16.7%, respectively. |

## Evidence to preserve

Store or link the following with the completed result: terminal output, the
effective calibration/detector YAML, `ros2 topic info` output, TF frame output,
a short rosbag of camera-info/detection/TF data, a screenshot showing the tag
and detector output, and host CPU/memory observations where available. Record
the physical tag distance and viewing angle separately before making any
metric-accuracy statement. Use
`scripts/record_bag.sh --profile webcam_apriltag` to record the source-scoped
raw/rectified images, camera information, detections, and `/tf` for this bench.
The ignored local evidence bags retained for this run are
`bags/webcam_apriltag-20260817T195613Z` (5.335 s, 108.2 MiB, full image and
observation capture) and `bags/webcam_apriltag_metadata-20260817T160500Z`
(8.068 s, 47 KiB, 74 detection and 74 TF messages). The full-image recording
adds substantial host I/O and must not be used as a detector-rate benchmark.

## Known limits at the time of this record

- The 0.250 m edge is user-confirmed, and the calibration is inherited for the
  same webcam, but no taped-distance or angular error study has yet been
  recorded.
- The camera's physical mount and `base_link -> webcam_optical_frame` extrinsic
  are unmeasured. The mock model's camera transform is synthetic and cannot be
  used as a substitute.
- The source-scoped webcam observation is intentionally not a map-frame pose.
  Final multi-tag deployment, tag world poses, quality gates, covariance,
  localizer, and global EKF remain future work.
- No conclusion about the Humble/Jetson/Orin runtime, ZED interoperability, or
  competition resource use follows from a development-host result.
- The generic Compose profile has no V4L2 device or `/dev/v4l` mapping. It is a
  build/development profile; a least-privilege target-runtime mapping must be
  designed and tested before running this camera launch inside a Jetson
  container.
- A TF client can retain the last dynamic observation after tag loss. Future
  consumers must reject stale transform timestamps rather than infer that a
  cached transform is a current observation.
- The observed camera-relative detection rate varied between sampling methods.
  Repeat the throughput test on the selected Jetson with a measured tag range,
  viewing angle, lighting condition, and CPU/load record before relying on the
  10–15 Hz initial target.

**Implementation Git commit:** `963e64c` (`feat: implement phase 4.1 webcam
apriltag bench`).
