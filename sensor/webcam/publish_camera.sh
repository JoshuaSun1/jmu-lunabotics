#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WEBCAM_DRIVER="${WEBCAM_DRIVER:-v4l2_camera}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
CAMERA_NS="${CAMERA_NS:-/camera}"
CAMERA_NAME="${CAMERA_NAME:-webcamera1}"
FRAME_ID="${FRAME_ID:-camera_link}"

IMAGE_TOPIC="${IMAGE_TOPIC:-${CAMERA_NS}/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-${CAMERA_NS}/camera_info}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_CAMERA_INFO_FILE="${SCRIPT_DIR}/webcam_calibration.yaml"
CAMERA_INFO_FILE="${CAMERA_INFO_FILE:-${DEFAULT_CAMERA_INFO_FILE}}"
CAMERA_INFO_URL="${CAMERA_INFO_URL:-}"

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

if [[ ! -e "${VIDEO_DEVICE}" ]]; then
  echo "Camera device ${VIDEO_DEVICE} was not found." >&2
  echo "Plug in the webcam or set VIDEO_DEVICE=/dev/videoN." >&2
  exit 1
fi

if [[ -n "${CAMERA_INFO_URL}" ]]; then
  RESOLVED_CAMERA_INFO_URL="${CAMERA_INFO_URL}"
elif [[ -f "${CAMERA_INFO_FILE}" ]]; then
  RESOLVED_CAMERA_INFO_URL="file://$(readlink -f "${CAMERA_INFO_FILE}")"
else
  RESOLVED_CAMERA_INFO_URL=""
fi

if [[ -n "${RESOLVED_CAMERA_INFO_URL}" ]]; then
  echo "Using camera calibration: ${RESOLVED_CAMERA_INFO_URL}"
else
  echo "Camera calibration file not found; camera_info may remain uncalibrated." >&2
  echo "Expected file: ${CAMERA_INFO_FILE}" >&2
fi

case "${WEBCAM_DRIVER}" in
  v4l2_camera)
    if ! ros2 pkg prefix v4l2_camera >/dev/null 2>&1; then
      echo "Missing ROS package: v4l2_camera" >&2
      echo "Install it with: sudo apt-get install -y ros-${ROS_DISTRO}-v4l2-camera" >&2
      exit 1
    fi

    echo "Publishing ${VIDEO_DEVICE} with v4l2_camera"
    echo "  image:       ${IMAGE_TOPIC}"
    echo "  camera info: ${CAMERA_INFO_TOPIC}"
    echo "  camera name: ${CAMERA_NAME}"
    exec ros2 run v4l2_camera v4l2_camera_node \
      --ros-args \
      -p video_device:="${VIDEO_DEVICE}" \
      -p camera_name:="${CAMERA_NAME}" \
      -p camera_info_url:="${RESOLVED_CAMERA_INFO_URL}" \
      -p camera_frame_id:="${FRAME_ID}" \
      -r image_raw:="${IMAGE_TOPIC}" \
      -r camera_info:="${CAMERA_INFO_TOPIC}"
    ;;

  usb_cam)
    if ! ros2 pkg prefix usb_cam >/dev/null 2>&1; then
      echo "Missing ROS package: usb_cam" >&2
      echo "Install it with: sudo apt-get install -y ros-${ROS_DISTRO}-usb-cam" >&2
      exit 1
    fi

    echo "Publishing ${VIDEO_DEVICE} with usb_cam"
    echo "  image:       ${IMAGE_TOPIC}"
    echo "  camera info: ${CAMERA_INFO_TOPIC}"
    echo "  camera name: ${CAMERA_NAME}"
    exec ros2 run usb_cam usb_cam_node_exe \
      --ros-args \
      -p video_device:="${VIDEO_DEVICE}" \
      -p camera_name:="${CAMERA_NAME}" \
      -p camera_info_url:="${RESOLVED_CAMERA_INFO_URL}" \
      -p frame_id:="${FRAME_ID}" \
      -r image_raw:="${IMAGE_TOPIC}" \
      -r camera_info:="${CAMERA_INFO_TOPIC}"
    ;;

  *)
    echo "Unsupported WEBCAM_DRIVER=${WEBCAM_DRIVER}" >&2
    echo "Use WEBCAM_DRIVER=v4l2_camera or WEBCAM_DRIVER=usb_cam." >&2
    exit 1
    ;;
esac
