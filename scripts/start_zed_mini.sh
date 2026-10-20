#!/usr/bin/env bash
# Entry point for the Jetson boot service. Keep camera and ROS on this host.
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="$(cd "${repository_root}/../.." && pwd)"

# ROS setup scripts inspect optional variables; source without nounset.
set +u
source /opt/ros/jazzy/setup.bash
source "${workspace_root}/install/setup.bash"
set -u

terrain_args=()
if [[ -n "${TERRAIN_PARAMS_FILE:-}" ]]; then
  terrain_args+=(terrain_params_file:="${TERRAIN_PARAMS_FILE}")
fi
exec ros2 launch lb_sensors zed_mini.launch.py \
  camera_name:=zed serial_number:="${ZED_SERIAL_NUMBER:-0}" \
  enable_depth_perception:="${ENABLE_DEPTH_PERCEPTION:-false}" \
  "${terrain_args[@]}"
