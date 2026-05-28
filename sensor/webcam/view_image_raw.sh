#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/image_raw}"

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

if ! ros2 pkg prefix rqt_image_view >/dev/null 2>&1; then
  echo "Missing ROS package: rqt_image_view" >&2
  echo "Install it with: sudo apt-get install -y ros-${ROS_DISTRO}-rqt-image-view" >&2
  exit 1
fi

echo "Opening ${IMAGE_TOPIC} in rqt_image_view"
exec ros2 run rqt_image_view rqt_image_view "${IMAGE_TOPIC}"
