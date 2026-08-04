#!/usr/bin/env bash
# Safe development-environment checks for the Lunabotics repository.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TARGET_ROS_DISTRO="humble"
REQUESTED_ROS_DISTRO="${LUNABOT_ROS_DISTRO:-${TARGET_ROS_DISTRO}}"

usage() {
  cat <<'EOF'
Usage: scripts/bootstrap_dev.sh [--check] [--strict-target] [--install-hooks]
                                [--install-host-tools]

Default behavior is --check: inspect prerequisites without modifying the host.

  --check               Check the selected ROS environment and development tools.
  --strict-target       Require an arm64 Jetson with L4T 36.5 during --check.
  --install-hooks       Install repository-local pre-commit hooks.
  --install-host-tools  Explicitly install generic Ubuntu development tools after
                         confirmation. Does not install ROS, JetPack, or ZED SDK.
  -h, --help            Show this help text.

The target runtime is ROS 2 Humble. Set LUNABOT_ROS_DISTRO only for a clearly
labelled non-target structural check, such as a Jazzy development workstation.
EOF
}

require_not_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    echo "Run this script as a regular user, not root." >&2
    exit 1
  fi
}

check_command() {
  local command_name="$1"
  if command -v "${command_name}" >/dev/null 2>&1; then
    printf 'ok: %s\n' "${command_name}"
    return 0
  fi
  printf 'missing: %s\n' "${command_name}" >&2
  return 1
}

run_check() {
  local strict_target="$1"
  local failures=0
  local architecture
  architecture="$(uname -m)"

  printf 'repository: %s\n' "${REPOSITORY_ROOT}"
  printf 'requested ROS distribution: %s\n' "${REQUESTED_ROS_DISTRO}"
  printf 'architecture: %s\n' "${architecture}"

  if [[ "${REQUESTED_ROS_DISTRO}" != "${TARGET_ROS_DISTRO}" ]]; then
    printf 'warning: %s is a development-only override; target lock is %s\n' \
      "${REQUESTED_ROS_DISTRO}" "${TARGET_ROS_DISTRO}" >&2
  fi

  if [[ ! -f "/opt/ros/${REQUESTED_ROS_DISTRO}/setup.bash" ]]; then
    printf 'missing ROS setup: /opt/ros/%s/setup.bash\n' "${REQUESTED_ROS_DISTRO}" >&2
    failures=1
  fi

  check_command git || failures=1
  check_command python3 || failures=1
  check_command colcon || failures=1
  check_command pre-commit || failures=1

  if [[ "${architecture}" == "aarch64" ]]; then
    if [[ -r /etc/nv_tegra_release ]] \
      && grep -q 'R36 (release), REVISION: 5.0' /etc/nv_tegra_release; then
      echo 'ok: detected L4T 36.5 target release'
    else
      echo 'warning: L4T 36.5 was not verified from /etc/nv_tegra_release' >&2
      [[ "${strict_target}" == "true" ]] && failures=1
    fi
  else
    echo 'info: non-arm64 host; Jetson, GPU, ZED, and runtime validation are deferred'
    [[ "${strict_target}" == "true" ]] && failures=1
  fi

  return "${failures}"
}

install_host_tools() {
  if ! command -v apt-get >/dev/null 2>&1; then
    echo '--install-host-tools is supported only on apt-based hosts.' >&2
    return 1
  fi

  echo 'This installs generic host tools only. It will not add ROS repositories,'
  echo 'flash JetPack, install the ZED SDK, initialize rosdep, or edit shell profiles.'
  read -r -p 'Continue with sudo apt-get install? [y/N] ' confirmation
  if [[ "${confirmation}" != "y" && "${confirmation}" != "Y" ]]; then
    echo 'Host-tool installation cancelled.'
    return 0
  fi

  sudo apt-get update
  sudo apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-pre-commit \
    python3-rosdep \
    python3-vcstool
}

main() {
  local action="check"
  local strict_target="false"

  require_not_root
  while [[ "$#" -gt 0 ]]; do
    case "$1" in
      --check)
        action="check"
        ;;
      --strict-target)
        strict_target="true"
        ;;
      --install-hooks)
        action="install-hooks"
        ;;
      --install-host-tools)
        action="install-host-tools"
        ;;
      -h|--help)
        usage
        return 0
        ;;
      *)
        echo "Unknown option: $1" >&2
        usage >&2
        return 2
        ;;
    esac
    shift
  done

  case "${action}" in
    check)
      run_check "${strict_target}"
      ;;
    install-hooks)
      check_command pre-commit
      (cd "${REPOSITORY_ROOT}" && pre-commit install --install-hooks)
      ;;
    install-host-tools)
      install_host_tools
      ;;
  esac
}

main "$@"
