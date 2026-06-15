#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
IMAGE_PROC_PKG="${IMAGE_PROC_PKG:-image_proc}"
RECTIFY_NODE="${RECTIFY_NODE:-rectify_node}"
APRILTAG_PKG="${APRILTAG_PKG:-apriltag_ros}"
APRILTAG_NODE="${APRILTAG_NODE:-apriltag_node}"
APRILTAG_NS="${APRILTAG_NS:-/apriltag}"
POSE_PUBLISHER_SCRIPT="${POSE_PUBLISHER_SCRIPT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/publish_apriltag_pose.py}"

IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/camera_info}"
RECTIFIED_IMAGE_TOPIC="${RECTIFIED_IMAGE_TOPIC:-/camera/image_rect}"
FRAME_ID="${FRAME_ID:-camera_link}"

APRILTAG_TAG_ID="${APRILTAG_TAG_ID:-0}"
APRILTAG_TAG_FAMILY="${APRILTAG_TAG_FAMILY:-36h11}"
APRILTAG_TAG_SIZE_METERS="${APRILTAG_TAG_SIZE_METERS:-0.162}"
APRILTAG_FRAME_NAME="${APRILTAG_FRAME_NAME:-apriltag_${APRILTAG_TAG_ID}}"
APRILTAG_POSE_TOPIC="${APRILTAG_POSE_TOPIC:-${APRILTAG_NS}/camera_pose}"
APRILTAG_DISTANCE_TOPIC="${APRILTAG_DISTANCE_TOPIC:-${APRILTAG_NS}/camera_distance}"
APRILTAG_POSE_ESTIMATION_METHOD="${APRILTAG_POSE_ESTIMATION_METHOD:-pnp}"

APRILTAG_THREADS="${APRILTAG_THREADS:-1}"
APRILTAG_DECIMATE="${APRILTAG_DECIMATE:-2.0}"
APRILTAG_BLUR="${APRILTAG_BLUR:-0.0}"
APRILTAG_REFINE="${APRILTAG_REFINE:-1}"
APRILTAG_SHARPENING="${APRILTAG_SHARPENING:-0.25}"
APRILTAG_DEBUG="${APRILTAG_DEBUG:-0}"
APRILTAG_MAX_HAMMING="${APRILTAG_MAX_HAMMING:-0}"

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

if ! ros2 pkg prefix "${APRILTAG_PKG}" >/dev/null 2>&1; then
  echo "Missing ROS package: ${APRILTAG_PKG}" >&2
  echo "Install it with: sudo apt-get install -y ros-${ROS_DISTRO}-apriltag-ros" >&2
  exit 1
fi

if ! ros2 pkg prefix "${IMAGE_PROC_PKG}" >/dev/null 2>&1; then
  echo "Missing ROS package: ${IMAGE_PROC_PKG}" >&2
  echo "Install it with: sudo apt-get install -y ros-${ROS_DISTRO}-image-proc" >&2
  exit 1
fi

CONFIG_FILE="$(mktemp /tmp/apriltag_config.XXXXXX.yaml)"
RECTIFY_PID=""
APRILTAG_PID=""
cleanup() {
  if [[ -n "${APRILTAG_PID}" ]] && kill -0 "${APRILTAG_PID}" >/dev/null 2>&1; then
    kill "${APRILTAG_PID}" >/dev/null 2>&1 || true
    wait "${APRILTAG_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${RECTIFY_PID}" ]] && kill -0 "${RECTIFY_PID}" >/dev/null 2>&1; then
    kill "${RECTIFY_PID}" >/dev/null 2>&1 || true
    wait "${RECTIFY_PID}" >/dev/null 2>&1 || true
  fi
  rm -f "${CONFIG_FILE}"
}
trap cleanup EXIT INT TERM

cat > "${CONFIG_FILE}" <<EOF
apriltag:
  ros__parameters:
    image_transport: raw
    family: ${APRILTAG_TAG_FAMILY}
    size: ${APRILTAG_TAG_SIZE_METERS}
    max_hamming: ${APRILTAG_MAX_HAMMING}
    pose_estimation_method: ${APRILTAG_POSE_ESTIMATION_METHOD}
    detector:
      threads: ${APRILTAG_THREADS}
      decimate: ${APRILTAG_DECIMATE}
      blur: ${APRILTAG_BLUR}
      refine: ${APRILTAG_REFINE}
      sharpening: ${APRILTAG_SHARPENING}
      debug: ${APRILTAG_DEBUG}
    tag:
      ids: [${APRILTAG_TAG_ID}]
      frames: [${APRILTAG_FRAME_NAME}]
      sizes: [${APRILTAG_TAG_SIZE_METERS}]
EOF

echo "Publishing AprilTag detections for tag ${APRILTAG_TAG_ID} (${APRILTAG_TAG_FAMILY})"
echo "  image raw:    ${IMAGE_TOPIC}"
echo "  image rect:   ${RECTIFIED_IMAGE_TOPIC}"
echo "  camera info:  ${CAMERA_INFO_TOPIC}"
echo "  detections:   ${APRILTAG_NS}/detections"
echo "  tf child:     ${APRILTAG_FRAME_NAME}"
echo "  pose topic:   ${APRILTAG_POSE_TOPIC}"
echo "  distance:     ${APRILTAG_DISTANCE_TOPIC}"
echo "  tag size (m): ${APRILTAG_TAG_SIZE_METERS}"

ros2 run "${IMAGE_PROC_PKG}" "${RECTIFY_NODE}" \
  --ros-args \
  -r image:="${IMAGE_TOPIC}" \
  -r camera_info:="${CAMERA_INFO_TOPIC}" \
  -r image_rect:="${RECTIFIED_IMAGE_TOPIC}" &
RECTIFY_PID="$!"

echo "Waiting for ${RECTIFIED_IMAGE_TOPIC}..."
for _ in {1..60}; do
  if ros2 topic list | grep -qx "${RECTIFIED_IMAGE_TOPIC}"; then
    break
  fi

  if ! kill -0 "${RECTIFY_PID}" >/dev/null 2>&1; then
    echo "Rectify node exited before ${RECTIFIED_IMAGE_TOPIC} became available." >&2
    exit 1
  fi

  sleep 0.5
done

if ! ros2 topic list | grep -qx "${RECTIFIED_IMAGE_TOPIC}"; then
  echo "Timed out waiting for ${RECTIFIED_IMAGE_TOPIC}." >&2
  exit 1
fi

if [[ ! -f "${POSE_PUBLISHER_SCRIPT}" ]]; then
  echo "Pose publisher script not found: ${POSE_PUBLISHER_SCRIPT}" >&2
  exit 1
fi

ros2 run "${APRILTAG_PKG}" "${APRILTAG_NODE}" \
  --ros-args \
  -r __ns:="${APRILTAG_NS}" \
  -r image_rect:="${RECTIFIED_IMAGE_TOPIC}" \
  -r camera_info:="${CAMERA_INFO_TOPIC}" \
  -r detections:="${APRILTAG_NS}/detections" \
  --params-file "${CONFIG_FILE}" &
APRILTAG_PID="$!"

sleep 1
if ! kill -0 "${APRILTAG_PID}" >/dev/null 2>&1; then
  echo "AprilTag detector exited during startup." >&2
  wait "${APRILTAG_PID}"
  exit 1
fi

export APRILTAG_FRAME_NAME
export APRILTAG_POSE_TOPIC
export APRILTAG_DISTANCE_TOPIC
export FRAME_ID

exec python3 "${POSE_PUBLISHER_SCRIPT}"
