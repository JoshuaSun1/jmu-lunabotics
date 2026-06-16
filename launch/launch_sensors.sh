#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WEBCAM_DIR="${REPO_ROOT}/sensor/webcam"
ROBOT_CONFIG_FILE="${ROBOT_CONFIG_FILE:-${SCRIPT_DIR}/robot_config.sh}"

if [[ -f "${ROBOT_CONFIG_FILE}" ]]; then
  # shellcheck source=robot_config.sh
  source "${ROBOT_CONFIG_FILE}"
else
  echo "Robot config file not found: ${ROBOT_CONFIG_FILE}" >&2
  exit 1
fi

: "${ROS_DISTRO:=jazzy}"
: "${IMAGE_TOPIC:=/camera/image_raw}"

is_enabled() {
  case "${1,,}" in
    true|1|yes|on|enabled)
      return 0
      ;;
    false|0|no|off|disabled)
      return 1
      ;;
    *)
      echo "Invalid boolean setting: ${1}" >&2
      echo "Use true or false in ${ROBOT_CONFIG_FILE}." >&2
      exit 1
      ;;
  esac
}

CAMERA_PID=""
APRILTAG_PID=""

export VIDEO_DEVICE
export CAMERA_NAME
export CAMERA_INFO_FILE
export CAMERA_INFO_URL
export APRILTAG_TAG_ID
export APRILTAG_TAG_FAMILY
export APRILTAG_TAG_SIZE_METERS
export APRILTAG_POSE_TOPIC
export APRILTAG_DISTANCE_TOPIC

if ! is_enabled "${ENABLE_CAMERA}"; then
  echo "Camera launch disabled by ${ROBOT_CONFIG_FILE}."
  exit 0
fi

if ! command -v ros2 >/dev/null 2>&1; then
  ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
  if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "Could not find ros2 or ${ROS_SETUP}." >&2
    echo "Install ROS 2 ${ROS_DISTRO}, then try again." >&2
    exit 1
  fi

  # shellcheck source=/dev/null
  source "${ROS_SETUP}"
fi

"${WEBCAM_DIR}/publish_camera.sh" &
CAMERA_PID="$!"

if is_enabled "${ENABLE_APRILTAG}"; then
  "${WEBCAM_DIR}/publish_apriltag.sh" &
  APRILTAG_PID="$!"
fi

cleanup() {
  if [[ -n "${CAMERA_PID}" ]] && kill -0 "${CAMERA_PID}" >/dev/null 2>&1; then
    kill "${CAMERA_PID}" >/dev/null 2>&1 || true
    wait "${CAMERA_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${APRILTAG_PID}" ]] && kill -0 "${APRILTAG_PID}" >/dev/null 2>&1; then
    kill "${APRILTAG_PID}" >/dev/null 2>&1 || true
    wait "${APRILTAG_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

if ! is_enabled "${ENABLE_VIEW_IMAGE_RAW}"; then
  echo "Image viewer disabled by ${ROBOT_CONFIG_FILE}."
  echo "Sensor publishers are running. Press Ctrl-C to stop."
  wait "${CAMERA_PID}"
  exit $?
fi

echo "Waiting for ${IMAGE_TOPIC}..."
for _ in {1..60}; do
  if ros2 topic list | grep -qx "${IMAGE_TOPIC}"; then
    "${WEBCAM_DIR}/view_image_raw.sh"
    exit 0
  fi

  if ! kill -0 "${CAMERA_PID}" >/dev/null 2>&1; then
    echo "Camera publisher exited before ${IMAGE_TOPIC} became available." >&2
    exit 1
  fi

  sleep 0.5
done

echo "Timed out waiting for ${IMAGE_TOPIC}." >&2
exit 1
