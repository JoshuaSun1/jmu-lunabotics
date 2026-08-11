# Requirements traceability

| Specification requirement | Evidence | Status |
|---|---|---|
| Repository structure and package ownership | Root `lb_*` package layout and `architecture.md` | Implemented |
| Version lock | `docker/versions.env`, platform-lock resource | Proposed; physical validation pending |
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
| Container builds on target architecture | `docker/` Phase 1 dependency set | Pending physical arm64 validation |
| `colcon build` / `colcon test` | Build/test scripts and package tests | Pending validation in locked Humble environment |

Requirements outside Phase 1 remain deliberately unimplemented. See
[`../resources/devlog.md`](../resources/devlog.md) for validation evidence as
it becomes available.
