# jmu-lunabotics

ROS 2 workspace source repository for Lunabotics autonomy and controls.

## Quick Start

Project baseline:
- Ubuntu 24.04 LTS
- ROS 2 Jazzy

From this repository root, run:

```bash
bash scripts/bootstrap_dev.sh
```

Then in a new shell:

```bash
cd ~/dev_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Primary robot bringup now goes through ROS 2 launch:

```bash
ros2 launch jmu_lunabotics robot.launch.py
```

Optional features can be toggled at launch time:

```bash
ros2 launch jmu_lunabotics robot.launch.py enable_rviz:=true enable_view_image_raw:=true
```

Default robot bringup values now live in [config/robot_defaults.yaml](/home/jsun3/dev_ws/src/jmu-lunabotics/config/robot_defaults.yaml:1), so camera, AprilTag, and visualization settings have one ROS-native home.

Current source layout is intentionally small: `launch/` owns bringup entry points, `config/` owns runtime defaults and calibration, and `nodes/` owns developed ROS nodes grouped by subsystem.

Full setup and troubleshooting are documented in `docs/setup.md`.

## Development Tooling

This repo includes:
- `.clang-format` for C/C++ formatting
- `.pre-commit-config.yaml` for commit-time checks (`black`, `clang-format`, and file hygiene checks)

If you need to re-install hooks manually:

```bash
export PATH="$HOME/.local/bin:$PATH"
PRE_COMMIT_HOME=.pre-commit-cache pre-commit install -f --install-hooks
PRE_COMMIT_HOME=.pre-commit-cache pre-commit run --all-files
```
