# jmu-lunabotics

ROS 2 workspace source repository for Lunabotics autonomy and controls.

## Development tooling

This repo includes:
- `.clang-format` for C/C++ formatting
- `.pre-commit-config.yaml` for pre-commit checks (`black`, `clang-format`, and basic file hygiene checks)

Install and enable hooks:

```bash
sudo apt-get update
sudo apt-get install -y python3-pip clang-format
python3 -m pip install --user pre-commit
~/.local/bin/pre-commit install
~/.local/bin/pre-commit run --all-files
```
