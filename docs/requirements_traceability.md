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
| Container builds on target architecture | `docker/` Phase 1 dependency set | Pending physical arm64 validation |
| `colcon build` / `colcon test` | Build/test scripts and package tests | Pending validation in locked Humble environment |

Requirements outside Phase 1 remain deliberately unimplemented. See
[`../resources/devlog.md`](../resources/devlog.md) for validation evidence as
it becomes available.
