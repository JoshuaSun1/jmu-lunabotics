#!/usr/bin/env bash
# Build and test only the ROS 2 packages owned by this repository.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_TO_USE="${LUNABOT_ROS_DISTRO:-humble}"
SKIP_BUILD="false"

usage() {
  cat <<'EOF'
Usage: scripts/test.sh [--skip-build]

Runs a repository-scoped colcon build followed by colcon test. The runtime
target is Humble; an explicit LUNABOT_ROS_DISTRO override is structural-only.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --skip-build)
      SKIP_BUILD="true"
      ;;
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
  shift
done

if [[ "${SKIP_BUILD}" != "true" ]]; then
  "${SCRIPT_DIR}/build.sh"
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO_TO_USE}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO_TO_USE} is not installed at /opt/ros/${ROS_DISTRO_TO_USE}." >&2
  exit 1
fi

# ROS setup scripts access optional variables that may be unset under `set -u`.
set +u
source "/opt/ros/${ROS_DISTRO_TO_USE}/setup.bash"
source "${REPOSITORY_ROOT}/install/setup.bash"
set -u
colcon --log-base "${REPOSITORY_ROOT}/log" test \
  --base-paths "${REPOSITORY_ROOT}/src" \
  --build-base "${REPOSITORY_ROOT}/build" \
  --install-base "${REPOSITORY_ROOT}/install" \
  --event-handlers console_direct+ \
  --return-code-on-test-failure
colcon test-result --test-result-base "${REPOSITORY_ROOT}/build" --verbose
