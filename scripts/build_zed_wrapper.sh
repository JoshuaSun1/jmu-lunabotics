#!/usr/bin/env bash
# Build the ZED ROS 2 wrapper with one package and one compiler job at a time.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="${LUNABOT_WORKSPACE_ROOT:-$(cd "${REPOSITORY_ROOT}/../.." && pwd)}"
WRAPPER_ROOT="${WORKSPACE_ROOT}/src/zed-ros2-wrapper"

usage() {
  cat <<'EOF'
Usage: scripts/build_zed_wrapper.sh

Build zed_wrapper and its dependencies in the enclosing ROS 2 Jazzy workspace.
Limits colcon to one package and CMake/Make/Ninja to one compiler job. Set
LUNABOT_WORKSPACE_ROOT if the workspace is not two directories above this repo.
EOF
}

if [[ "$#" -gt 0 ]]; then
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
fi

if [[ ! -f "${WRAPPER_ROOT}/zed_wrapper/package.xml" ]]; then
  echo "ZED wrapper checkout not found at ${WRAPPER_ROOT}." >&2
  exit 1
fi

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed at /opt/ros/jazzy." >&2
  exit 1
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon is not installed or is not on PATH." >&2
  exit 1
fi

# --parallel-workers limits packages only. colcon-cmake otherwise defaults to
# one Make job per CPU core; both variables also cover direct CMake builds.
export MAKEFLAGS="-j1 -l1"
export CMAKE_BUILD_PARALLEL_LEVEL=1

# ROS setup scripts access optional variables that may be unset under `set -u`.
set +u
source /opt/ros/jazzy/setup.bash
set -u

echo "Building ZED wrapper in ${WORKSPACE_ROOT} with one package and one compiler job."
exec colcon --log-base "${WORKSPACE_ROOT}/log" build \
  --base-paths "${WORKSPACE_ROOT}/src" \
  --build-base "${WORKSPACE_ROOT}/build" \
  --install-base "${WORKSPACE_ROOT}/install" \
  --symlink-install \
  --executor sequential \
  --packages-up-to zed_wrapper \
  --packages-skip zed_debug \
  --event-handlers console_direct+
