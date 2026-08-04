#!/usr/bin/env bash
# Record a documented ROS 2 topic group after an explicit operator request.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_TO_USE="${LUNABOT_ROS_DISTRO:-humble}"
PROFILE=""
DRY_RUN="false"

usage() {
  cat <<'EOF'
Usage: scripts/record_bag.sh --profile PROFILE [--dry-run]

Profiles: localization, perception, navigation, hardware, mission

The bag output is written below ./bags/ and is ignored by Git. This script
never enables motors and only records the canonical topic groups documented in
docs/lunabotics_autonomy_software_spec.md.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="${2:-}"
      shift
      ;;
    --dry-run)
      DRY_RUN="true"
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

if [[ -z "${PROFILE}" ]]; then
  echo '--profile is required.' >&2
  usage >&2
  exit 2
fi

case "${PROFILE}" in
  localization)
    TOPICS=(/odom/wheel /imu/data /odometry/local /odometry/global /tag_detections /localization/apriltag_pose /tf /tf_static)
    ;;
  perception)
    TOPICS=(/scan /perception/depth_obstacles /perception/terrain_hazards /diagnostics)
    ;;
  navigation)
    TOPICS=(/plan /local_plan /cmd_vel_nav /cmd_vel /diagnostics)
    ;;
  hardware)
    TOPICS=(/joint_states /odom/wheel /cmd_vel /diagnostics)
    ;;
  mission)
    TOPICS=(/mission/state /diagnostics)
    ;;
  *)
    echo "Unknown profile: ${PROFILE}" >&2
    usage >&2
    exit 2
    ;;
esac

OUTPUT_DIRECTORY="${REPOSITORY_ROOT}/bags/${PROFILE}-$(date -u +%Y%m%dT%H%M%SZ)"
printf 'profile: %s\noutput: %s\ntopics: %s\n' "${PROFILE}" "${OUTPUT_DIRECTORY}" "${TOPICS[*]}"

if [[ "${DRY_RUN}" == "true" ]]; then
  exit 0
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO_TO_USE}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO_TO_USE} is not installed." >&2
  exit 1
fi

# ROS setup scripts access optional variables that may be unset under `set -u`.
set +u
source "/opt/ros/${ROS_DISTRO_TO_USE}/setup.bash"
set -u
mkdir -p "${REPOSITORY_ROOT}/bags"
exec ros2 bag record --output "${OUTPUT_DIRECTORY}" "${TOPICS[@]}"
