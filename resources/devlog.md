# Lunabotics software development log

This is the canonical, append-only software development record required for
competition deliverables. It complements detailed documentation under `docs/`;
supporting evidence is linked from each entry rather than duplicated here.

## 2026-08-04 — Prototype transition and Phase 0 scaffold

**Purpose and scope:** Preserved the prior prototype, then established the
reviewable ROS 2 repository scaffold without implementing motor, sensor,
localization, navigation, mission, or safety behavior.

**Components and files:** Added the ten package boundaries under `src/`,
repository quality tooling, scoped developer scripts, Jetson/container version
lock, architecture and safety documents, the BOM resource, and the TBD
register. Package purpose and future ownership are defined in
[`docs/architecture.md`](../docs/architecture.md).

**ROS interfaces and TF:** No live topics, services, actions, or hardware
commands were introduced. The documented authority contract reserves
`map -> odom` for the global EKF, `odom -> base_link` for the local EKF, and
rigid `base_link` transforms for `robot_state_publisher`.

**Parameters and dependencies:** Hardware dimensions, drive kinematics,
sensor mounting transforms, safety limits, protocol fields, and platform
runtime details remain `TBD` in [`docs/tbd_register.md`](../docs/tbd_register.md).
The proposed software baseline is JetPack 6.2.2/L4T 36.5.0, Ubuntu 22.04,
ROS 2 Humble, ZED SDK 5.2.3, and ZED wrapper v5.2.2; it is proposed rather
than target-validated.

**Safety and failure behavior:** All motors remain disabled. The scaffold
contains no direct drive, power, E-stop, or controller implementation.

**Verification and evidence:** On the x86_64 Ubuntu 24.04/ROS 2 Jazzy host,
`LUNABOT_ROS_DISTRO=jazzy ./scripts/test.sh` passed 10 packages and 44 tests;
`./scripts/lint.sh` passed. This is structural evidence only. The target arm64
Jetson/Humble/ZED container build and hardware validation remain open.

**Git evidence:** `bd2d2c8` preserved the active-repository transition;
`bbd94d9` added the Phase 0 scaffold. The earlier prototype is retained at
tag and branch `prototype-before-phase0-2026-08-04`.

## 2026-08-10 — Four-wheel chassis and digging-arm allocation clarified

**Purpose and decision:** The team confirmed a four-wheel-drive chassis with
a large scooping bucket. The six-motor BOM includes motors planned for the
digging arm; it must not be interpreted as a six-wheel drivetrain.

**Architecture and alternatives:** The robot will be documented as a
four-wheel-drive chassis. Steering/kinematic topology remains `TBD`: do not
infer skid-steer from the bulldozer-like form factor. The decision and its
consequences are in
[`docs/decisions/0001_four_wheel_chassis.md`](../docs/decisions/0001_four_wheel_chassis.md).

**Parameters and interfaces:** No physical dimensions, wheel placement,
bucket envelope, actuator protocol, safety limit, or ROS interface has been
invented. `DRIVE-01` is narrowed to the steering/turning model; `GEOM-01`,
`DRIVE-02`, `DRIVE-03`, `SAFE-01`, and `MISS-01` remain open.

**Safety and verification:** This documentation-only change adds no command
path or runtime behavior. It was checked by the repository scaffold test; no
hardware test is applicable.

**Git evidence:** This entry will be associated with the documentation commit
that records it.
