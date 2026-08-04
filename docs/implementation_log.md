# Implementation log

This is an append-only record of material repository changes. Each later entry
must identify scope, validation, assumptions, TBDs opened or closed, and
remaining blockers.

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
