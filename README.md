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
