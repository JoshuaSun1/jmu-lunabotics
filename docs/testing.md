# Testing

## Phase 0 through Phase 2 checks

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
