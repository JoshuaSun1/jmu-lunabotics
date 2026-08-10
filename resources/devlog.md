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

**Git evidence:** `d3cb02b` (`docs: record four-wheel chassis decision`)
records this documentation update.

## 2026-08-10 — Source-directory and ROS-package migration

**Purpose and decision:** Reorganized the ROS workspace into concise source
directories while retaining unique ROS package names with the `lb_` prefix.
The mapping is documented in [`docs/architecture.md`](../docs/architecture.md):
`launch/lb_launch`, `model/lb_model`, `hardware/lb_hardware`,
`interfaces/lb_interfaces`, `localization/lb_localization`,
`state_manager/lb_state_manager`, `navigation/lb_navigation`,
`sensors/lb_sensors`, `safety/lb_safety`, and `sim/lb_sim`.

**Dependency and compatibility assessment:** The scaffold has no direct
package-to-package dependencies or runtime interfaces yet; each manifest
declares only its ament build and lint dependencies. The ROS 2 underlay already
contains the core package `launch`, so `lb_launch` is used for `src/launch` to
avoid shadowing ROS launch tooling. No topics, services, actions, TF ownership,
parameters, safety behavior, or hardware interfaces changed.

**Files and follow-up:** Renamed all source directories, package manifests, and
CMake project names; updated the scaffold contract, lint path, architecture,
canonical specification, and chassis decision record. Future launch commands
will use `ros2 launch lb_launch ...`; source-directory names are not ROS package
names.

**Verification and evidence:** A clean Jazzy structural build discovered the
ten intended `lb_*` packages. The command
`LUNABOT_ROS_DISTRO=jazzy ./scripts/test.sh --skip-build` passed 44 tests after
that build. This remains a non-target host check; Humble/arm64 target
validation is still open.

**Git evidence:** `cc2ec8e` (`refactor: rename ROS package architecture`)
records the package-architecture migration.

## 2026-08-10 — Source directories aligned with ROS package names

**Purpose and decision:** The source-directory name now matches its ROS
package identity exactly. For example, `src/lb_launch` contains package
`lb_launch`; this removes the prior two-name source/package mapping.

**Dependency and compatibility assessment:** Package identities, declared
dependencies, topics, services, actions, TF ownership, parameters, safety
behavior, and hardware interfaces are unchanged. `lb_launch` remains the safe
identity because the ROS underlay owns the unprefixed `launch` package.

**Files and verification:** Renamed all ten source directories and updated the
source-path contract, lint script, architecture document, and canonical
specification. A clean Jazzy structural build discovered the ten intended
packages; its test run passed 44 tests and the repository lint run passed.
This remains a non-target host check; Humble/arm64 target validation is open.

**Git evidence:** `dffd236` (`refactor: align source directories with package
names`) records this source-directory alignment.

## 2026-08-10 — Workspace layout flattened to `dev_ws`

**Purpose and decision:** `dev_ws` is now the sole ROS workspace. The Git
repository remains at `dev_ws/src/jmu-lunabotics`, with its ten `lb_*` ROS
packages directly at repository root. Repository-local `src/`, `build/`,
`install/`, and `log/` directories are no longer part of the active layout.

**Components and dependencies:** `scripts/build.sh` and `scripts/test.sh` now
build only this repository's packages while writing artifacts to
`dev_ws/build`, `dev_ws/install`, and `dev_ws/log`. The CI job provides an
explicit workspace-root override, and the container creates the same layout at
`/workspace/src/jmu-lunabotics`. No ROS package dependencies, interfaces, TF
ownership, parameters, safety behavior, or hardware behavior changed.

**Files and documentation:** Moved all package directories to repository root;
updated build/test/lint/bootstrap scripts, scaffold test, Docker/Compose, CI,
README, architecture, operations, testing documentation, and the canonical
specification. Added `.dockerignore` so generated artifacts are excluded from
the container build context.

**Verification and evidence:** A clean x86_64/Jazzy structural build wrote
only to `dev_ws/build`, `dev_ws/install`, and `dev_ws/log`; it discovered ten
packages and passed 44 tests. The generated artifacts under both the repository
and `dev_ws/src` were removed after validation. This does not replace the
pending Humble/arm64 Jetson validation.

**Known validation limitation:** The pre-commit hooks through YAML validation
passed, but the cached Black mirror hook stalled on this host and was stopped;
the system Black executable is not installed. Shell syntax, Python compilation,
whitespace, end-of-file, and YAML checks passed. Restore a working Black hook
environment before treating the full lint gate as revalidated.

**Git evidence:** This entry is associated with the workspace-flattening commit
recorded in repository history.
