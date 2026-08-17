# Testing

## Phase 0 through Phase 4.1 checks

| Check | Command | What it establishes |
|---|---|---|
| Scaffold contract | `scripts/test.sh` | Package manifests and required Phase 0 files are present. |
| ROS package build | `scripts/build.sh` | The empty package scaffold configures and builds. |
| Style and hygiene | `scripts/lint.sh` | Pre-commit hooks and local syntax checks pass. |
| Target container | `docker compose --env-file docker/versions.env -f docker/docker-compose.yml build` | Must be run natively on the arm64 Jetson. |
| Xacro contract | `scripts/test.sh` | Expands the synthetic model when `xacro` is installed; checks the nonphysical profile and required frame names. |
| Mock launch | `scripts/test.sh` in Humble CI/container | Starts `GenericSystem`, confirms an active `joint_state_broadcaster`, required TFs, single publishers, and no `map`/`odom`. |
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

An x86/Jazzy result is a development-host result only. It cannot validate the
Humble/JetPack/ZED runtime combination.

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
establish metric pose accuracy. Before Phase 4 localization, add measured
camera-to-tag accuracy tests, a surveyed physical camera extrinsic,
multiple-tag/source collision tests, tag map/localizer unit tests,
covariance/quality-gate tests, and TF-authority integration tests for the
local and global EKFs.

Use `scripts/record_bag.sh --profile webcam_apriltag` to capture the
source-scoped raw/rectified images, camera information, detections, and `/tf`
for a Phase 4.1 bench run.
