# Implementation log

This is an append-only record of material repository changes. The canonical
ongoing development record is now [`../resources/devlog.md`](../resources/devlog.md),
which includes the traceability fields required by the project instructions.
Each later entry must identify scope, validation, assumptions, TBDs opened or
closed, and remaining blockers.

## 2026-08-04 — Prototype transition and Phase 0 start

- Preserved the prototype at Git tag `prototype-before-phase0-2026-08-04` and
  branch `archive/prototype-before-phase0-2026-08-04`.
- Created local reference copy `../old-lunabotics` with `COLCON_IGNORE`.
- Committed removal of the prototype working tree as `bd2d2c8`.
- Began Phase 0 on branch `phase0-scaffold`.
- Carried forward the BOM and proposed platform-lock note as source resources.
- Opened the TBDs in `docs/tbd_register.md`; no hardware values were invented.
- Remaining blocker: arm64 Jetson/JetPack/ZED container validation requires the
  physical target.

## 2026-08-04 — Phase 0 scaffold completed

- Added the specification-aligned repository layout, ten empty ROS 2 packages,
  version-lock container scaffold, resources, architecture documents, TBD
  register, quality configuration, and CI workflow.
- Recreated the bootstrap workflow as a safe, check-first script. It does not
  alter shell profiles, ROS repositories, JetPack, ZED SDK, or hardware unless
  a separately named, explicit action is requested.
- Added repository-scoped build, test, lint, bag-recording, performance, and
  TF-contract utility scripts.
- Validation on the available x86_64 Ubuntu 24.04 / ROS 2 Jazzy host:
  `LUNABOT_ROS_DISTRO=jazzy ./scripts/test.sh` passed 10 packages and 44 tests;
  `./scripts/lint.sh` passed.
- This validation is structural only. Jetson arm64/Humble container build, L4T,
  CUDA, ZED SDK/wrapper, camera, and hardware validation remain blocked on the
  physical target and are not represented as complete.

## 2026-08-24 — Phase 4.2 software-only known-tag localizer

- Added `lb_localization`'s C++ `tag_localizer` core/node, fail-closed launch,
  tag-map/localizer/EKF templates, and synthetic-only fixtures.
- The core loads a known tag map and camera-source contracts, composes
  `map -> base_link` measurements from exact-time detector observations plus a
  measured camera extrinsic, gates/fuses distinct physical tags, and publishes
  only `/localization/apriltag_pose` and `/diagnostics`.
- The implementation has no TF broadcaster and cannot publish `map -> odom`;
  its public launch is excluded from system bringup and rejects absent,
  unverified, or synthetic-without-explicit-opt-in data.
- Its planar fused output gives roll/pitch an explicit large unobserved
  covariance, validates the map's REP-103 convention, and uses a dedicated TF
  listener thread so nonzero lookup timeouts work on Humble/Jazzy.
- Recorded the planned future field inventory as `tag36h11` IDs 1–3 at nominal
  0.300 m detector-corner edges. It remains an unmeasured, unsurveyed plan;
  Phase 4.1's ID-0 / 0.250 m tag remains bench-only.
- x86_64/Jazzy structural validation built 10 packages and passed 87 tests
  with no errors/failures and one expected conditional skip. A public launch
  with no map/config failed before creating a node, as designed; explicit
  synthetic fixtures booted the node and were cleanly stopped under timeout.
- Required physical work remains the tag/map survey, camera extrinsics,
  calibration and detector-TF timestamp validation, ZED integration, gate and
  covariance tuning, Phase 3 local odometry, and global-EKF TF-authority test.
