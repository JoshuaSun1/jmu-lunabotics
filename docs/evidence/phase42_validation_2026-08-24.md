# Phase 4.2 validation evidence — 2026-08-24

## Scope

This evidence covers the deterministic, software-only Phase 4.2 known-tag
localizer implementation. It does not claim live AprilTag pose accuracy, a
surveyed map, a measured camera extrinsic, a functioning ZED, a running global
EKF, or target Jetson/Humble validation.

## Development-host environment

- Host: x86_64 Ubuntu 24.04 with ROS 2 Jazzy.
- Target remains Jetson Orin Nano, Ubuntu 22.04, ROS 2 Humble, per the
  repository platform lock.
- `robot_localization` was not installed on this development host, so the
  nonlaunchable global-EKF template was checked statically only.

## Commands

```bash
LUNABOT_ROS_DISTRO=jazzy ./scripts/build.sh
LUNABOT_ROS_DISTRO=jazzy ./scripts/test.sh --skip-build
```

The first attempted build exposed the expected CMake linkage distinction
between the ROS `yaml_cpp_vendor` package and its `yaml-cpp` library target.
The implementation was corrected to find and link `yaml-cpp` explicitly. The
final structural build completed all 10 packages. The full test command passed
**87 tests with 0 errors, 0 failures, and 1 expected conditional skip**.

One parallel incremental rebuild was externally interrupted while compiling the
localizer node; its logs contain no compiler error or test result. A subsequent
serial `cmake --build` of `lb_localization`, followed by the full 10-package
build above, completed successfully. The completed build/test results are the
authoritative validation record.

The public launch was also invoked with no configuration arguments. It exited
before starting a node with: `tag_map_file must be an explicit file; no
production default exists`.

With the synthetic fixtures supplied explicitly and
`allow_synthetic_test_data:=true`, the node initialized with its two configured
source subscriptions and was then cleanly stopped by a five-second test timeout.
The restricted sandbox blocked DDS network sockets, so this establishes only
configuration/startup behavior—not a live detector, TF, or pose-publication
test.

## Deterministic coverage

The Phase 4.2 C++ and static-contract tests exercise:

- Parsing a synthetic-only map with planned `tag36h11` IDs 1–3 at 0.300 m,
  while proving ID 0 is excluded.
- Refusal of the synthetic map/config without the explicit test-only opt-in.
- The exact transform composition from known tag, detector observation, and
  base-to-camera extrinsic, including rotated and full 3-D geometry.
- Equivalent recovery from independently configured webcam and ZED extrinsics.
- Range, hamming, decision-margin, tag-size, covariance-growth, fusion,
  wrapped-yaw, duplicate-landmark, inter-tag-disagreement, jump gates, REP-103
  map-convention rejection, and explicit unobserved roll/pitch covariance.
- Static checks that the node requests exact-time TF lookup and has no
  transform broadcaster or `lb_launch` integration. It also uses a dedicated
  TF listener thread so its configured nonzero lookup timeout is valid on
  Humble/Jazzy. Target-runtime behavior with cached detector TF data remains
  unvalidated.

The synthetic tag poses, camera extrinsics, quality thresholds, and covariance
coefficients are test data. They must not be copied into a physical field map
or operational profile.

## Runtime boundary verified by inspection

`tag_localizer.launch.py` requires explicit files. The core accepts only a
fully surveyed map/config/extrinsic set by default. It accepts a fully
synthetic set only when `allow_synthetic_test_data:=true` is explicitly passed.
No usable configuration is included in public bringup, and no localizer code
imports or creates a `TransformBroadcaster`.

The remaining live validation is intentionally open: physical camera mounting,
exact detector-TF timestamp behavior on the selected Humble/Jetson image,
multi-camera observation collision handling, recorded-data gate tuning,
global-EKF authority, and map-frame accuracy all require hardware or Phase 3
inputs not available in this phase.
