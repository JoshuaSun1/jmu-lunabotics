# Requirements traceability

| Specification requirement | Evidence | Status |
|---|---|---|
| Repository structure and package ownership | Root `lb_*` package layout and `architecture.md` | Implemented |
| Native runtime version lock | Platform-lock resource and bootstrap target checks | Platform versions proposed; exact installed Debian/ROS manifest pending physical validation |
| Formatting/lint/test configuration | `.pre-commit-config.yaml`, scripts, package tests | Implemented; restricted-sandbox Black limitation recorded in devlog |
| Phase 1 parameterized Xacro | `lb_model/urdf/lb_mock.urdf.xacro`, `lb_model/config/mock_geometry.yaml`, model contract test | Implemented as synthetic mock only |
| Phase 1 sensor and mechanism frames | Xacro links, `docs/phase1_mock_model.md`, mock TF test | Implemented; physical mounts and linkage TBD |
| Phase 1 ros2_control mock hardware | Upstream `mock_components/GenericSystem`, `lb_sim/config/mock_controllers.yaml` | Implemented; no real transport or drive controller |
| Phase 1 joint-state and robot-state publishing | `lb_sim/launch/mock_robot.launch.py`, `lb_model/launch/description.launch.py`, runtime test | Implemented; Humble runtime validation pending locally |
| Phase 1 TF authority test | `lb_sim/test/test_mock_robot_launch.py`, `scripts/check_tf_authority.py` | Implemented; Humble runtime validation pending locally |
| Phase 2 transport abstraction | `lb_hardware/include/lb_hardware/drive_transport.hpp`, transport unit test | Implemented with mock/unavailable transport only; physical protocol pending |
| Phase 2 mock transport and safe failure | `MockDriveTransport`, `LunabotDriveHardware`, `mock_drive_transport_test` | Implemented and structurally tested; Humble plugin runtime validation pending |
| Phase 2 real transport skeleton | `lb_hardware/src/real_drive_transport.cpp` | Implemented fail-closed; no device/protocol selected |
| Phase 2 four-wheel diff-drive configuration | `lb_hardware/config/mock_drive_controllers.yaml` | Implemented with synthetic mock geometry only; `DRIVE-02` pending |
| Phase 2 bench-test launch | `lb_launch/launch/bench_test.launch.py`, `lb_hardware/launch/hardware.launch.py` | Implemented; Humble runtime launch/remap validation pending |
| Phase 4.1 calibrated webcam acquisition and rectification | `lb_sensors/launch/webcam_apriltag.launch.py`, `lb_sensors/config/webcam_calibration.yaml` | Implemented as non-actuating 640 x 480 V4L2 acquisition after a fail-closed 15 Hz preflight plus rectification; user-confirmed inherited calibration requires measured revalidation and target runtime validation |
| Phase 4.1 AprilTag detection baseline | `lb_sensors/config/webcam_apriltag.yaml`, `lb_sensors/launch/webcam_apriltag.launch.py`, `lb_sensors/test/test_phase41_webcam_contract.py` | Implemented for `tag36h11` ID 0, 0.250 m detector-corner edge, and hamming 0; development-host output samples were 9.17–11.86 Hz, so the specification's 10–15 FPS detector target is not yet demonstrated; target profiling pending |
| Phase 4.1 camera-scoped observation TF | `webcam_optical_frame -> webcam_observation_tag_0`, `docs/phase41_webcam_apriltag.md` | Implemented as dynamic detector observation only; no physical base-camera extrinsic, tag map, robot pose, localizer, EKF, `map -> odom`, or `odom -> base_link` |
| Phase 4.2 tag-map schema and planned field inventory | `lb_localization/config/tag_map.template.yaml`, `docs/phase42_tag_localizer.md` | Implemented as an unverified template for `tag36h11` IDs 1–3, nominal 0.300 m; no field poses/survey or physical claim |
| Phase 4.2 known-tag localizer core | `lb_localization/src/tag_localizer_core.cpp`, `tag_localizer_node.cpp`, unit/contract tests | Implemented and structurally tested with synthetic fixtures; publishes only map-frame absolute pose and diagnostics, never TF; physical use and the specification's localization-validity monitoring remain pending `CAM-01`, `TAG-01`, and `LOC-01` |
| Global EKF correction and `map -> odom` authority | `lb_localization/config/ekf_global.template.yaml` | Pending Phase 3 `/odometry/local`, measured tuning, `robot_localization`, and target runtime TF-authority evidence; template is not launched |
| Native target provisions on target architecture | `bootstrap_dev.sh --install --profile jetson` | Pending physical arm64 validation |
| `colcon build` / `colcon test` | Build/test scripts and package tests | Pending validation in locked Humble environment |

The Phase 4.1 detector baseline and Phase 4.2 synthetic localizer are narrower
than completed field localization. Requirements outside the completed
mock-drive, camera-observation, and software-only localizer boundaries remain
deliberately unimplemented. See [`../resources/devlog.md`](../resources/devlog.md)
for validation evidence as it becomes available.
