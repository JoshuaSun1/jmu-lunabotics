# Bootstrap validation evidence — 2026-08-24

## Scope

This evidence covers the repository's one-time Humble provisioning workflow
and per-terminal activation helper. It does not claim that the installer was
executed on Ubuntu 22.04, the Jetson Orin Nano, or physical robot hardware.
Those actions would modify a target host and require the unavailable hardware.

## Development-host environment

- Host: x86_64 Ubuntu 24.04 with ROS 2 Jazzy.
- Locked target: Jetson Orin Nano 8 GB, JetPack 6.2.2 / L4T 36.5, Ubuntu
  22.04, and ROS 2 Humble.
- Result: this host is deliberately unsuitable for `--install`; it can only
  validate structural behavior under an explicit Jazzy override.

## Commands and results

```bash
bash -n scripts/bootstrap_dev.sh scripts/activate.sh scripts/lint.sh
python3 -m pytest -q lb_launch/test/test_bootstrap_contract.py
LUNABOT_ROS_DISTRO=jazzy ./scripts/build.sh
LUNABOT_ROS_DISTRO=jazzy ./scripts/test.sh --skip-build
./scripts/lint.sh
```

The standalone bootstrap contract suite passed 7 tests. The final structural
test suite built all 10 packages and reported **95 tests, 0 errors, 0
failures, and 1 expected conditional skip**. Repository lint passed all
configured hooks.

The following non-mutating negative checks behaved as intended:

- Default `./scripts/bootstrap_dev.sh --check` returned nonzero because this
  host lacks `/opt/ros/humble/setup.bash`.
- `LUNABOT_ROS_DISTRO=jazzy ./scripts/bootstrap_dev.sh --check` returned
  nonzero while precisely identifying the absent Xacro/ros2_control packages;
  it confirmed the built overlay and already-present webcam/AprilTag packages.
- The Jetson profile rejected Ubuntu 24.04, x86_64, absent L4T 36.5, and no
  Orin model string.
- A nonexistent `LUNABOT_WORKSPACE_ROOT` failed rather than being accepted.
- Conflicting `--check --install` flags failed before any host work.
- Non-target activation was rejected until both
  `LUNABOT_ROS_DISTRO=jazzy` and
  `LUNABOT_ALLOW_NON_TARGET_ACTIVATION=1` were explicitly set.

No `apt`, `sudo`, `rosdep update`, build-clean, JetPack, ZED, camera, motor,
or hardware-launch operation was performed by this validation.

## Coverage and limitations

The contract test protects the explicit installer/default-check split,
workspace layout validation, Humble plus `rosdep` dependency authority,
ordering of the ROS apt source before ROS-provided host tooling, source-only
activation, action-conflict rejection, and explicit JetPack/ZED exclusions.

The actual installer remains unexecuted on its supported Jammy/Orin platform.
It must still be validated there with package-download logs, the final
readiness check, and a mock/webcam bench run as applicable. The strict target
preflight checks Ubuntu, arm64, L4T, and Orin Nano model text, but the physical
8 GB SKU must be confirmed separately because the device-tree model does not
encode memory capacity.
