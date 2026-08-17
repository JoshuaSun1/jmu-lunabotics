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

**Git evidence:** `6f88f8f` (`refactor: flatten ROS workspace layout`) records
the workspace-flattening migration.

## 2026-08-10 — Phase 1 mock description, TF contract, and ros2_control wiring

**Purpose and scope:** Implemented the Phase 1 no-hardware model boundary:
parameterized Xacro, four-wheel skid-steer mock tree, sensor/mount frames,
upstream ros2_control mock hardware, joint-state publishing, RViz profile, and
automated TF-authority checks. The user-provided
[`Current Lunabot Robot Design Codex Handoff`](../docs/resources/Current_Lunabot_Robot_Design_Codex_Handoff.md)
is retained as the design-evidence source.

**Components and files:** `lb_model` now owns
`urdf/lb_mock.urdf.xacro`, the explicitly synthetic
`config/mock_geometry.yaml`, standalone `description.launch.py`, and the RViz
profile. `lb_sim` owns `mock_controllers.yaml`, `mock_robot.launch.py`, and the
headless launch test. `lb_launch/launch/bringup.launch.py` is a thin Phase 1
mock include; `lb_hardware` remains intentionally empty until the Phase 2
transport/plugin work. The model contract and detailed frame/provenance record
are in [`docs/phase1_mock_model.md`](../docs/phase1_mock_model.md).

**ROS interfaces and TF:** `joint_state_broadcaster` is the sole mock
publisher of `/joint_states` (`sensor_msgs/msg/JointState`), consumed by
`robot_state_publisher`. `robot_state_publisher` is the sole publisher of
`/tf` and `/tf_static` (`tf2_msgs/msg/TFMessage`) for the `base_link` subtree.
That subtree contains four continuous wheel joints, a fixed conceptual
`scoop_link`, ZED/webcam mock mounts and optical frames, `imu_link`, and a
generic `lidar_mount_link`. The mock contains no `map`, `odom`,
`base_footprint`, `/cmd_vel`, `/odom/wheel`, or real sensor output. Future
global/local EKFs retain `map -> odom`/`odom -> base_link` ownership.

**Parameters and evidence:** Every value in `mock_geometry.yaml` is marked
`synthetic_mock_only` and prohibited from physical use; its replacement source
is a reviewed CAD or measured calibration record. The mock controller update
rate is a visualization/test cadence, not a physical control or safety limit.
Four-wheel skid-steer is now a confirmed topology (`DRIVE-01` closed), based on
the team confirmation and handoff. Wheel radius/placement, track, chassis
origin, sensor mounts, linkage, reductions, encoder signs, protocol, and
safety limits remain open as `DRIVE-02`, `DRIVE-03`, `GEOM-01`, `ZED-01`,
`LIDAR-01`, `TAG-01`, `MISS-01`, and `SAFE-01`.

**Architecture and alternatives:** The mock uses upstream
`mock_components/GenericSystem` rather than inventing an `lb_hardware` plugin.
It exposes wheel velocity/state interfaces but loads only
`joint_state_broadcaster`; no `diff_drive_controller` or command controller is
introduced before Phase 2. The scoop is a fixed proxy because the lift/dump
kinematics are unresolved. Camera optical frames are mock-only URDF-owned;
real wrapper ownership must be selected before integration to avoid duplicate
TF publishers.

**Safety and failure behavior:** The Phase 1 mock has no real transport,
motor-enable path, `/cmd_vel` consumer, odometry publisher, or mechanism
command path. `enable_motors` remains false by default and is intentionally
inert if set. The mock therefore cannot move or enable the physical robot.

**Verification and evidence:** `LUNABOT_ROS_DISTRO=jazzy ./scripts/test.sh`
built all ten packages and passed 53 tests with zero failures (one runtime
launch test conditionally skipped because the host lacks Xacro/ros2_control).
`./scripts/lint.sh` passed in an unrestricted local execution. A temporary,
non-installed extraction of the Jazzy Xacro package expanded the model to 14
links and 13 joints; `check_urdf` parsed the output with `base_link` as root.
The standalone description launch also initialized `robot_state_publisher`.
The concise command/results record is
[`docs/evidence/phase1_validation_2026-08-10.md`](../docs/evidence/phase1_validation_2026-08-10.md).
The restricted sandbox blocks DDS socket creation, and the host lacks
controller-manager and joint-state-broadcaster executables, so the complete
mock runtime/TF-authority test remains pending Humble CI/container. The Black
pre-commit hook itself is healthy but cannot run multi-file inside the restricted
sandbox because its multiprocessing socket is blocked; it succeeds in an
unrestricted environment/CI.

**Git evidence:** `0d78d29` (`feat: implement phase 1 mock robot`) records
this implementation and evidence update.

## 2026-08-11 — Phase 2 mock-first drive hardware interface

**Purpose and scope:** Implemented the Phase 2 software boundary between
`diff_drive_controller` and a future drive MCU without choosing an electrical
protocol. This is a mock/fail-closed implementation only: it does not command
motors, open a CAN/serial/USB device, define an encoder scale, or implement a
physical E-stop.

**Components and files:** `lb_hardware` now owns the protocol-neutral
`DriveTransport` contract, `MockDriveTransport`, `RealDriveTransport`,
`LunabotDriveHardware` Humble system-hardware plugin, plugin export file,
mock-only controller configuration, hardware launch, C++ transport unit test,
and Phase 2 static contracts. `lb_launch` owns
`launch/bench_test.launch.py` and now routes public bringup through that safe
bench profile. The detailed component/interface record is
[`docs/phase2_drive_interface.md`](../docs/phase2_drive_interface.md).

**ROS interfaces and TF:** The declared mock command boundary is `/cmd_vel`
(`geometry_msgs/msg/TwistStamped`) remapped to the controller's native
`~/cmd_vel`; wheel commands are ROS-control velocity interfaces in rad/s.
`joint_state_broadcaster` produces `/joint_states`
(`sensor_msgs/msg/JointState`), and the controller declares a mock
`~/odom` to `/odom/wheel` (`nav_msgs/msg/Odometry`) remap. The controller has
`enable_odom_tf: false`; `robot_state_publisher` retains sole Phase 1 ownership
of the `base_link` subtree, while Phase 3 retains `odom -> base_link` ownership.
The Humble in-process remaps and runtime topics remain target-validation work.

**Parameters and provenance:** The controller uses confirmed four mock wheel
joints in left/right groups. Its 0.12 m radius, 0.68 m separation, rates,
covariance, timeout, and velocity/acceleration/jerk limits are all labelled
synthetic mock values, sourced only from the prior mock profile or test needs;
they are prohibited for physical use. `mock_communication_loss_after_reads`
and `mock_command_timeout_s` are deterministic test inputs. `DRIVE-02`,
`DRIVE-03`, `GEOM-01`, and `SAFE-01` remain open.

**Architecture and alternatives:** Four wheel-joint values are kept at the
software boundary rather than assuming two versus four physical motor channels.
The real skeleton deliberately avoids selecting the BOM's USB-to-CAN adapter
as a direct Jetson-to-controller design, because the specification requires an
MCU safety boundary and the actual topology/protocol remains unresolved.

**Safety and failure behavior:** The mock begins disabled. The real skeleton
always rejects connection and output. The plugin zeros command interfaces and
requests `stop()` plus `set_enabled(false)` before returning an error on state
read/write failure or non-finite command. This is fail-closed software behavior
only; it does not provide an independent physical stop, controller fault
latching, or MCU watchdog.

**Dependencies and verification:** Target dependencies are `hardware_interface`,
`pluginlib`, `rclcpp`, `rclcpp_lifecycle`, controller manager, joint-state
broadcaster, diff-drive controller, Xacro, and robot-state publisher. On the
available x86_64/Jazzy structural host,
`LUNABOT_ROS_DISTRO=jazzy ./scripts/test.sh` built ten packages and passed 63
tests with zero failures and one expected Xacro-related skip. The
ROS-independent mock transport compiled and passed its C++ unit test; static
contracts passed. `./scripts/lint.sh` passed in unrestricted execution; the
restricted sandbox still blocks Black worker processes. Evidence and command details are in
[`docs/evidence/phase2_validation_2026-08-11.md`](../docs/evidence/phase2_validation_2026-08-11.md).

**Known limitations and remaining work:** The host lacks target ROS-control and
Xacro runtime dependencies, so the custom plugin was not compiled or loaded
there; Humble/arm64 must verify plugin build, controller activation, wheel
motion, `/cmd_vel` and `/odom/wheel` remaps, command timeout, and no
`odom -> base_link` TF. No physical drive test is authorized until electrical,
geometry, encoder, protocol, and safety prerequisites are reviewed.

**Git evidence:** `3f12a2a` (`feat: implement phase 2 drive interface`) records this implementation.

## 2026-08-17 — Phase 4.1 webcam AprilTag bench discovery

**Purpose and scope:** Performed a non-actuating, webcam-only bench check before
Phase 4.1 implementation. This confirms image capture and fiducial detection;
it does not validate robot-relative or global pose estimation.

**Observed hardware and interfaces:** The connected external Logitech UVC camera
(`046d:0825`) was available as a V4L2 capture source at 640 x 480 YUYV, 30 Hz.
Its current `/dev/video2` enumeration is host-session-specific and must not be
used as a persistent launch identifier; a stable `/dev/v4l/by-id` path will be
selected during implementation. A temporary `v4l2_camera` node published
`/image_raw` (`sensor_msgs/msg/Image`) and `/camera_info`
(`sensor_msgs/msg/CameraInfo`). A temporary `apriltag_ros` node consumed that
stream and published `/phase41/tag_detections` as an
`apriltag_msgs/msg/AprilTagDetectionArray`.

**Measured result:** The visible tag was detected as family `tag36h11`, ID `0`,
with Hamming error `0` and decision margin `115.7505`. This is direct evidence
that the webcam, selected tag family, and CPU detector operate together on the
development host. The test camera stream had no loaded calibration, so its
pose-estimation warning was expected and no metric pose, covariance, TF, or
global-localization claim is made.

**Confirmed parameter provenance:** The user confirmed that the physical tag
edge size is 0.250 m. This is a user-provided physical value, to be treated as
the detector-corner edge length and independently rechecked before competition
use. The user also confirmed that the archived
`old-lunabotics/config/webcam_calibration.yaml` applies to this same camera.
That calibration is 640 x 480 with `plumb_bob` distortion; it is an archived,
user-confirmed calibration rather than a newly measured result. Phase 4.1 will
load it and later revalidate metric pose against a measured target distance.

**Architecture and remaining work:** Phase 4.1 will retain camera-independent
detection outputs so the webcam and later ZED RGB camera can publish separate
detection topics into one `lb_localization/tag_localizer`. The localizer will
use calibrated camera optical frames and a surveyed tag map, not detector TF
aliases, and only the later global EKF may publish `map -> odom`. Open items
remain TAG-01 (additional IDs, final tag size/placement and tag map), ZED-01
(ZED integration/frame authority), and the physical `base_link`-to-webcam
extrinsic transform.

**Git evidence:** `f219d12` (`docs: record Phase 4.1 webcam discovery`) records this bench-discovery evidence.
