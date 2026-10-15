# Testing

## Phase 0 through Phase 4.2 checks

| Check | Command | What it establishes |
|---|---|---|
| Bootstrap readiness check | `./scripts/bootstrap_dev.sh --check` | Non-mutating report of the selected ROS environment, tools, declared runtime packages, and built workspace. It should fail usefully before first provisioning. |
| One-time Jammy development provisioning | `./scripts/bootstrap_dev.sh --install --profile development` | On a supported Ubuntu 22.04 development host, installs the repository's ROS/development dependencies and prepares the workspace for mock and webcam-AprilTag work. It is not a routine per-session test. |
| One-time Jetson provisioning | `./scripts/bootstrap_dev.sh --install --profile jetson` | On the manually verified arm64 Orin Nano / JetPack 6.2.2 / L4T 36.5 target, prepares the target ROS workspace dependencies. It does not flash or validate JetPack, ZED, cameras, robot hardware, or safety hardware. |
| Per-terminal activation | `source scripts/activate.sh` | Sources the ROS and workspace overlays in the current shell without editing a persistent shell profile. |
| Bootstrap contract | `scripts/test.sh` | Statically verifies the explicit installer interface, safe default, workspace validation, Humble/rosdep authority, ROS-source ordering, activation helper, and target-only omissions without changing a host. |
| Scaffold contract | `scripts/test.sh` | Package manifests and required Phase 0 files are present. |
| ROS package build | `scripts/build.sh` | The empty package scaffold configures and builds. |
| Style and hygiene | `scripts/lint.sh` | Pre-commit hooks and local syntax checks pass. |
| Xacro contract | `scripts/test.sh` | Expands the synthetic model when `xacro` is installed; checks the nonphysical profile and required frame names. |
| Mock launch | `scripts/test.sh` on a provisioned Humble host or in CI | Starts `GenericSystem`, confirms an active `joint_state_broadcaster`, required TFs, single publishers, and no `map`/`odom`. |
| Live TF authority | `scripts/check_tf_authority.py --runtime` | Checks a separately launched `lb_sim mock_robot.launch.py` graph without commanding hardware. |
| Manual RViz smoke | `ros2 launch lb_sim mock_robot.launch.py use_rviz:=true` | Displays the synthetic four-wheel, scoop proxy, and sensor-mount TF tree; no motor hardware is needed. |
| Phase 2 transport unit test | `scripts/test.sh` | Verifies disabled mock startup, explicit mock motion, zero stop, command timeout, communication loss, and real-skeleton rejection. |
| Phase 2 static controller contract | `scripts/test.sh` | Verifies four wheel groups, synthetic-only geometry, stamped command configuration, limits, and disabled odometry TF. |
| Phase 2 mock bench | `ros2 launch lb_launch bench_test.launch.py` | Starts headless, disabled mock hardware. It must not be used for physical propulsion. |
| Phase 2 enabled mock exercise | `ros2 launch lb_launch bench_test.launch.py enable_motors:=true` | Enables in-memory mock output only; target Humble runtime test must verify wheel motion, wheel odometry, command timeout, and no `odom -> base_link` TF. |
| Phase 4.1 webcam contract | `scripts/test.sh` | Verifies source-scoped launch/configuration, 640 x 480 inherited calibration profile, V4L2 15 Hz preflight, `tag36h11` ID 0/0.250 m/hamming-0 baseline, unique webcam observation frame, and absence of a localizer/EKF/static camera TF. |
| Phase 4.1 calibrated webcam bench | `ros2 launch lb_sensors webcam_apriltag.launch.py` | Runs fail-closed V4L2 rate preflight, `v4l2_camera -> image_proc -> apriltag_ros`, without a motor, localizer, or global TF path. Requires the webcam, calibrated 640 x 480 profile, and visible test tag. |
| Phase 4.1 detection inspection | `ros2 topic echo /sensors/webcam/tag_detections --once` | Confirms the source-scoped detection's family, ID, hamming, decision margin, and camera header frame. It does not measure pose accuracy. |
| Phase 4.1 observation-TF inspection | `ros2 run tf2_ros tf2_echo webcam_optical_frame webcam_observation_tag_0` | Confirms newly stamped dynamic camera-relative observations while the tag is visible; a cached transform after loss is stale and must not be interpreted as a base-relative or map-frame pose. |
| Phase 4.1 rate/resource baseline | `ros2 topic hz /sensors/webcam/image_raw` and `/sensors/webcam/tag_detections` | Records actual host rates after V4L2 verifies the configured 15 Hz source rate; measure CPU/memory before making target performance claims. |
| Phase 4.2 localizer core/contract tests | `LUNABOT_ROS_DISTRO=jazzy ./scripts/test.sh` | Exercises synthetic tag-map parsing, explicit test-only opt-in, composition, quality/stale-TF contracts, covariance/fusion, duplicate-tag protection, jump gates, and the no-TF-broadcaster boundary. A synthetic node startup is also checked; it does not exercise a running camera/TF graph. |
| Tag-1 bag replay | `ros2 launch lb_sim bag_replay.launch.py bag_path:=$PWD/bags/test1_AT1 playback_rate:=0.25` | Replays the explicit Jazzy MCAP with `/clock`, opens the camera-relative RViz profile, bounds the current path, clears it on stale observations, and starts a new segment after reacquisition. It starts no camera, detector, map server, Nav2, or hardware. |
| Provisional arena-pose replay | `ros2 launch lb_sim arena_pose_replay.launch.py bag_path:=$PWD/bags/test1_AT1 playback_rate:=0.25` | Shows a raw 6-DoF camera pose, planar robot proxy, synthetic chassis/footprint, colocated rear webcam, and forward ZED over the static arena. Verify both view arrows, marker disappearance during tag loss, separate path after reacquisition, and movement direction. Geometry is estimated and output is visualization-only. |
| Arena raster contract | `colcon test --packages-select lb_navigation` | Verifies the 139 x 102 raster, 137 x 100 free interior, occupied one-cell perimeter, 0.05 m resolution, `[-0.05, -0.05, 0.0]` image origin, and omission of the unsurveyed center column. |
| Static map lifecycle smoke | `ros2 launch lb_navigation map.launch.py use_rviz:=false`, then inspect `/map_server` and `/map` | Requires `nav2_map_server` and `nav2_lifecycle_manager`. Verifies active lifecycle state, transient-local `/map`, `frame_id=map`, 0.05 m resolution, 139 x 102 dimensions, and the configured origin. It does not establish a robot pose or TF. |
| Static arena-scene smoke | `ros2 launch lb_navigation arena_scene.launch.py`, then `ros2 topic echo /visualization/arena/markers --once` | Verifies a transient-local, visualization-only scene containing the arena outline, excavation boundary, dashed provisional starting zone, berm target, and only configured tags. Confirm no column marker or TF publisher exists. |

An x86/Jazzy result is a development-host result only. It cannot validate the
Humble/JetPack/ZED runtime combination.

Bootstrap's `--install` path is intentionally tested only on an appropriate
Jammy host or the verified Jetson target because it changes that device's
package installation. The default `--check` path is the safe diagnostic to run
elsewhere. Neither path starts a hardware launch, enables a motor, installs the
ZED SDK or wrapper, edits a shell profile, or proves a camera feed; a
successfully provisioned host is mock/webcam-AprilTag-ready only as its actual
connected hardware and later bench preflights permit.

## Future required tests

The Phase 1 runtime launch test is enabled only when `xacro`, ros2_control, and
its test dependencies are installed. Phase 2 always runs the ROS-independent
transport unit and static controller/launch contracts. A host lacking the
target Humble controller stack cannot claim plugin loading, controller,
topic-remap, wheel-odometry, timeout, or TF runtime validation. Later phases
will add unit tests for localization, terrain hazards, mission, safety, and
protocol encoding; launch/integration tests for costmaps, Nav2, and safety
locks; and ordered physical hardware tests. The full sequence is defined in the
canonical specification.

Phase 4.1 must preserve the camera calibration/detector YAML, device identity,
V4L2 rate-preflight output, terminal output, observation-TF output, and a
short rosbag or equivalent evidence for each meaningful bench run. A
development-host pass is not a Humble/Jetson/Orin validation and does not
establish metric pose accuracy. Phase 4.2 adds offline tag-map/localizer,
covariance/quality-gate, and multi-source collision contracts, but physical
localization still needs measured camera-to-tag accuracy, a surveyed physical
camera extrinsic, detector-TF timestamp validation, recorded-data tuning, and
runtime TF-authority integration tests for the local and global EKFs.

Use `scripts/record_bag.sh --profile webcam_apriltag` to capture the
source-scoped raw/rectified images, camera information, detections, and `/tf`
for a Phase 4.1 bench run.
