# Setup Guide

This project targets:
- Ubuntu 24.04 LTS
- ROS 2 Jazzy

## One-command setup

From the repository root:

```bash
bash scripts/bootstrap_dev.sh
```

The script will:
- install ROS 2 Jazzy and development dependencies
- initialize and update `rosdep`
- install pre-commit and git hooks
- resolve ROS package dependencies from `src/`
- build the workspace

## Daily workflow

```bash
cd ~/dev_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install
```

Run your launch command in the same sourced shell:

```bash
ros2 launch jmu_lunabotics master.launch.py
```

The default launch settings are stored in `src/jmu-lunabotics/config/robot_defaults.yaml`.

## Troubleshooting

If `pre-commit` is not found after bootstrap:

```bash
export PATH="$HOME/.local/bin:$PATH"
pre-commit --version
```

If dependency resolution fails:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/dev_ws
rosdep install --from-paths src --ignore-src -r -y
```
