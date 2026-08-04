#!/usr/bin/env bash
# Build only the ROS 2 packages owned by this repository.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_TO_USE="${LUNABOT_ROS_DISTRO:-humble}"
CLEAN="false"

usage() {
  cat <<'EOF'
Usage: scripts/build.sh [--clean]

Builds only ./src into repository-local build/, install/, and log/ directories.
The runtime target is Humble; LUNABOT_ROS_DISTRO may be set only for a clearly
labelled non-target structural check on another ROS distribution.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --clean)
      CLEAN="true"
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

if [[ ! -d "${REPOSITORY_ROOT}/src" || ! -f "${REPOSITORY_ROOT}/README.md" ]]; then
  echo "Unable to identify repository root: ${REPOSITORY_ROOT}" >&2
  exit 1
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO_TO_USE}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO_TO_USE} is not installed at /opt/ros/${ROS_DISTRO_TO_USE}." >&2
  exit 1
fi

if [[ "${ROS_DISTRO_TO_USE}" != "humble" ]]; then
  echo "warning: ${ROS_DISTRO_TO_USE} is a non-target structural-check override" >&2
fi

if [[ "${CLEAN}" == "true" ]]; then
  rm -rf "${REPOSITORY_ROOT}/build" "${REPOSITORY_ROOT}/install" "${REPOSITORY_ROOT}/log"
fi

# ROS setup scripts access optional variables that may be unset under `set -u`.
set +u
source "/opt/ros/${ROS_DISTRO_TO_USE}/setup.bash"
set -u
exec colcon --log-base "${REPOSITORY_ROOT}/log" build \
  --base-paths "${REPOSITORY_ROOT}/src" \
  --build-base "${REPOSITORY_ROOT}/build" \
  --install-base "${REPOSITORY_ROOT}/install" \
  --symlink-install \
  --event-handlers console_direct+
