# Testing

## Phase 0 checks

| Check | Command | What it establishes |
|---|---|---|
| Scaffold contract | `scripts/test.sh` | Package manifests and required Phase 0 files are present. |
| ROS package build | `scripts/build.sh` | The empty package scaffold configures and builds. |
| Style and hygiene | `scripts/lint.sh` | Pre-commit hooks and local syntax checks pass. |
| Target container | `docker compose --env-file docker/versions.env build` | Must be run natively on the arm64 Jetson. |

An x86/Jazzy result is a development-host result only. It cannot validate the
Humble/JetPack/ZED runtime combination.

## Future required tests

Later phases will add unit tests for localization, terrain hazards, mission,
safety, and protocol encoding; launch/integration tests for transform
authority, mock bringup, costmaps, Nav2, and safety locks; and ordered physical
hardware tests. The full sequence is defined in the canonical specification.
