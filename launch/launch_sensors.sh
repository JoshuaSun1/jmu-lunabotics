#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/image_raw}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WEBCAM_DIR="${REPO_ROOT}/sensor/webcam"

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

cleanup() {
  if kill -0 "${CAMERA_PID}" >/dev/null 2>&1; then
    kill "${CAMERA_PID}" >/dev/null 2>&1 || true
    wait "${CAMERA_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

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
