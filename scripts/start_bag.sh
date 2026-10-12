#!/usr/bin/env bash
# Activate the built workspace and record an already-running ROS graph.
set -euo pipefail

LB_BAG_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LB_BAG_PROFILE="webcam_apriltag"
LB_BAG_DRY_RUN="false"
LB_BAG_NAME_ARGS=()

usage() {
  cat <<'EOF'
Usage: scripts/start_bag.sh [--profile PROFILE] [--ros-distro DISTRO] [--name NAME] [--dry-run]

Start your launch first, then run this script in another terminal. It sources
the ROS underlay and built workspace overlay and starts recording immediately.
The default profile is webcam_apriltag; the default ROS distribution is Humble
(or LUNABOT_ROS_DISTRO). Explicit --ros-distro jazzy opts into non-target use.

Profiles: localization, perception, navigation, hardware, mission, webcam_apriltag

Examples:
  scripts/start_bag.sh
  scripts/start_bag.sh --ros-distro jazzy
  scripts/start_bag.sh --profile hardware
  scripts/start_bag.sh --ros-distro jazzy --name tag_test_01

Bags are saved under the repository's bags/ directory. Press Ctrl+C to finish.
--name selects bags/NAME; omit it for a profile and UTC timestamp name.
Use letters, digits, underscores, or hyphens; existing outputs are rejected.
Use the same ROS_DOMAIN_ID and middleware/discovery settings as your launch.
--dry-run prints the recording plan without activation or recording.
This script does not build, install dependencies, or start robot nodes.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --profile|--ros-distro|--name)
      if [[ "$#" -lt 2 || -z "$2" || "$2" == --* ]]; then
        echo "Missing value for $1." >&2
        exit 2
      fi
      if [[ "$1" == "--profile" ]]; then
        LB_BAG_PROFILE="$2"
      elif [[ "$1" == "--name" ]]; then
        LB_BAG_NAME_ARGS=(--name "$2")
      else
        export LUNABOT_ROS_DISTRO="$2"
        # Naming a non-target distro explicitly is the operator's opt-in.
        if [[ "$2" != "humble" ]]; then
          export LUNABOT_ALLOW_NON_TARGET_ACTIVATION=1
        fi
      fi
      shift 2
      ;;
    --dry-run)
      LB_BAG_DRY_RUN="true"
      shift
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
done

case "${LB_BAG_PROFILE}" in
  localization|perception|navigation|hardware|mission|webcam_apriltag) ;;
  *)
    echo "Unknown profile: ${LB_BAG_PROFILE}" >&2
    exit 2
    ;;
esac

if [[ "${LB_BAG_DRY_RUN}" == "true" ]]; then
  exec "${LB_BAG_SCRIPT_DIR}/record_bag.sh" --profile "${LB_BAG_PROFILE}" "${LB_BAG_NAME_ARGS[@]}" --dry-run
fi

# shellcheck disable=SC1091
source "${LB_BAG_SCRIPT_DIR}/activate.sh"
echo 'Starting bag recording. Press Ctrl+C to finish and write metadata.'
exec "${LB_BAG_SCRIPT_DIR}/record_bag.sh" --profile "${LB_BAG_PROFILE}" "${LB_BAG_NAME_ARGS[@]}"
