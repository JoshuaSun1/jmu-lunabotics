#!/usr/bin/env bash
# One-time ROS development-environment provisioning and read-only checks.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TARGET_ROS_DISTRO="humble"
TARGET_UBUNTU_VERSION="22.04"
TARGET_L4T_RELEASE="R36 (release), REVISION: 5.0"
REQUESTED_ROS_DISTRO="${LUNABOT_ROS_DISTRO:-${TARGET_ROS_DISTRO}}"

infer_workspace_root() {
  local canonical_candidate
  canonical_candidate="$(cd "${REPOSITORY_ROOT}/../.." && pwd)"

  if [[ -d "${canonical_candidate}/src/jmu-lunabotics" ]] \
    && [[ "$(cd "${canonical_candidate}/src/jmu-lunabotics" && pwd)" == "${REPOSITORY_ROOT}" ]]; then
    printf '%s\n' "${canonical_candidate}"
  else
    # This supports a standalone clone and the CI checkout layout. The
    # canonical developer layout remains <workspace>/src/jmu-lunabotics.
    printf '%s\n' "${REPOSITORY_ROOT}"
  fi
}

WORKSPACE_ROOT="${LUNABOT_WORKSPACE_ROOT:-$(infer_workspace_root)}"
ACTION="check"
ACTION_EXPLICIT="false"
PROFILE="development"
STRICT_TARGET="false"
ASSUME_YES="false"
DRY_RUN="false"
WORKSPACE_LAYOUT="unvalidated"

usage() {
  cat <<'EOF'
Usage: scripts/bootstrap_dev.sh [--check | --install] [options]

Default behavior is --check: inspect a configured environment without
modifying the host.

Actions:
  --check                 Check the selected ROS environment and workspace.
  --install               One-time provision of this repository's declared
                          Humble dependencies, then build and test it.
  --install-hooks         Install repository-local pre-commit hooks only.
  --install-host-tools    Legacy generic-tools-only action; does not install
                          ROS or repository dependencies.

Options:
  --profile PROFILE       development (default) or jetson. The development
                          profile includes RViz and rosbag tools; the jetson
                          profile requires the locked Orin/L4T target and
                          remains headless.
  --workspace PATH        ROS workspace root. Recommended layout is
                          PATH/src/jmu-lunabotics. A standalone repository
                          root is accepted for CI and creates build artifacts
                          there.
  --strict-target         Require the locked Orin Nano/L4T 36.5 target during
                          --check. Implied by --profile jetson.
  --yes                   Do not ask for confirmation before a modifying
                          action.
  --dry-run               Print an --install plan without modifying the host.
  -h, --help              Show this help text.

--install is intentionally limited to Ubuntu 22.04 and ROS 2 Humble. It adds
the ROS apt source if needed, installs ROS base plus rosdep-resolved repository
dependencies, builds, and tests. It never edits shell profiles, flashes or
upgrades JetPack, installs the ZED SDK/wrapper, changes device permissions, or
launches hardware.
EOF
}

require_not_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    echo "Run this script as a regular user, not root." >&2
    return 1
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

normalize_workspace_root() {
  local requested_root="$1"
  if [[ ! -d "${requested_root}" ]]; then
    printf 'workspace does not exist: %s\n' "${requested_root}" >&2
    return 1
  fi
  WORKSPACE_ROOT="$(cd "${requested_root}" && pwd)"
}

validate_workspace_layout() {
  local actual_repository
  local canonical_repository

  if [[ ! -d "${WORKSPACE_ROOT}" ]]; then
    printf 'workspace does not exist: %s\n' "${WORKSPACE_ROOT}" >&2
    return 1
  fi
  if [[ ! -f "${REPOSITORY_ROOT}/README.md" || ! -d "${REPOSITORY_ROOT}/lb_launch" ]]; then
    printf 'unable to identify repository root: %s\n' "${REPOSITORY_ROOT}" >&2
    return 1
  fi

  actual_repository="$(cd "${REPOSITORY_ROOT}" && pwd)"
  if [[ -d "${WORKSPACE_ROOT}/src/jmu-lunabotics" ]]; then
    canonical_repository="$(cd "${WORKSPACE_ROOT}/src/jmu-lunabotics" && pwd)"
    if [[ "${actual_repository}" == "${canonical_repository}" ]]; then
      WORKSPACE_LAYOUT="canonical"
      return 0
    fi
  fi

  if [[ "${actual_repository}" == "${WORKSPACE_ROOT}" ]]; then
    WORKSPACE_LAYOUT="standalone"
    return 0
  fi

  cat >&2 <<EOF
workspace layout is ambiguous.
  repository: ${actual_repository}
  workspace:  ${WORKSPACE_ROOT}
Expected ${WORKSPACE_ROOT}/src/jmu-lunabotics, or pass the repository root as
--workspace for the standalone/CI layout.
EOF
  return 1
}

read_os_release() {
  OS_ID="unknown"
  OS_VERSION_ID="unknown"
  OS_CODENAME="unknown"

  if [[ -r /etc/os-release ]]; then
    OS_ID="$(. /etc/os-release; printf '%s' "${ID:-unknown}")"
    OS_VERSION_ID="$(. /etc/os-release; printf '%s' "${VERSION_ID:-unknown}")"
    OS_CODENAME="$(. /etc/os-release; printf '%s' "${UBUNTU_CODENAME:-${VERSION_CODENAME:-unknown}}")"
  fi
}

target_platform_status() {
  local architecture
  local device_model=""
  local failures=0
  architecture="$(uname -m)"
  read_os_release

  if [[ "${OS_ID}" != "ubuntu" || "${OS_VERSION_ID}" != "${TARGET_UBUNTU_VERSION}" ]]; then
    printf 'missing target operating system: expected Ubuntu %s, found %s %s\n' \
      "${TARGET_UBUNTU_VERSION}" "${OS_ID}" "${OS_VERSION_ID}" >&2
    failures=1
  else
    printf 'ok: target operating system Ubuntu %s\n' "${TARGET_UBUNTU_VERSION}"
  fi

  if [[ "${architecture}" != "aarch64" ]]; then
    printf 'missing target architecture: expected aarch64, found %s\n' "${architecture}" >&2
    failures=1
  else
    printf 'ok: target architecture %s\n' "${architecture}"
  fi

  if [[ ! -r /etc/nv_tegra_release ]] \
    || ! grep -Fq "${TARGET_L4T_RELEASE}" /etc/nv_tegra_release; then
    printf 'missing locked L4T release: expected %s\n' "${TARGET_L4T_RELEASE}" >&2
    failures=1
  else
    printf 'ok: locked L4T release %s\n' "${TARGET_L4T_RELEASE}"
  fi

  if [[ -r /proc/device-tree/model ]]; then
    device_model="$(tr -d '\0' </proc/device-tree/model)"
  fi
  if [[ "${device_model,,}" != *"orin nano"* ]]; then
    printf 'missing locked Jetson model: expected Orin Nano, found %s\n' \
      "${device_model:-unavailable}" >&2
    failures=1
  else
    printf 'ok: Jetson model %s\n' "${device_model}"
  fi
  echo 'info: confirm the physical 8 GB SKU separately; the device-tree model does not encode memory capacity'

  return "${failures}"
}

source_ros_underlay() {
  local setup_file="/opt/ros/${REQUESTED_ROS_DISTRO}/setup.bash"
  local source_status
  if [[ ! -f "${setup_file}" ]]; then
    printf 'missing ROS setup: %s\n' "${setup_file}" >&2
    return 1
  fi

  # ROS setup scripts read optional variables that may be unset under `set -u`.
  set +u
  # shellcheck disable=SC1090
  if source "${setup_file}"; then
    source_status=0
  else
    source_status=$?
  fi
  set -u
  return "${source_status}"
}

check_ros_package() {
  local package_name="$1"
  if ros2 pkg prefix "${package_name}" >/dev/null 2>&1; then
    printf 'ok: ROS package %s\n' "${package_name}"
    return 0
  fi

  printf 'missing ROS package: %s\n' "${package_name}" >&2
  return 1
}

run_check() {
  local failures=0
  local workspace_ready="false"
  local ros_ready="false"
  local overlay_ready="false"
  local architecture
  local package_name
  local -a required_ros_packages=(
    launch
    launch_ros
    xacro
    robot_state_publisher
    joint_state_publisher
    hardware_interface
    controller_manager
    joint_state_broadcaster
    diff_drive_controller
    apriltag_ros
    image_proc
    v4l2_camera
    apriltag_msgs
    diagnostic_msgs
    tf2_ros
    yaml_cpp_vendor
    rosbag2
  )
  local -a required_workspace_packages=(
    lb_interfaces
    lb_launch
    lb_model
    lb_hardware
    lb_navigation
    lb_safety
    lb_sim
    lb_sensors
    lb_state_manager
    lb_localization
  )

  read_os_release
  architecture="$(uname -m)"
  printf 'repository: %s\n' "${REPOSITORY_ROOT}"
  printf 'workspace: %s\n' "${WORKSPACE_ROOT}"
  printf 'requested ROS distribution: %s\n' "${REQUESTED_ROS_DISTRO}"
  printf 'profile: %s\n' "${PROFILE}"
  printf 'host: %s %s (%s) on %s\n' \
    "${OS_ID}" "${OS_VERSION_ID}" "${OS_CODENAME}" "${architecture}"

  if ! validate_workspace_layout; then
    failures=1
  else
    workspace_ready="true"
    printf 'ok: workspace layout %s\n' "${WORKSPACE_LAYOUT}"
  fi

  if [[ "${REQUESTED_ROS_DISTRO}" != "${TARGET_ROS_DISTRO}" ]]; then
    printf 'warning: %s is a development-only override; target lock is %s\n' \
      "${REQUESTED_ROS_DISTRO}" "${TARGET_ROS_DISTRO}" >&2
  fi

  if source_ros_underlay; then
    ros_ready="true"
    check_command ros2 || failures=1
  else
    failures=1
  fi

  check_command git || failures=1
  check_command python3 || failures=1
  check_command cmake || failures=1
  check_command colcon || failures=1
  check_command rosdep || failures=1
  check_command v4l2-ctl || failures=1
  if [[ "${PROFILE}" == "development" ]]; then
    check_command pre-commit || failures=1
  fi

  if [[ "${ros_ready}" == "true" ]]; then
    for package_name in "${required_ros_packages[@]}"; do
      check_ros_package "${package_name}" || failures=1
    done
    if [[ "${PROFILE}" == "development" ]]; then
      check_ros_package rviz2 || failures=1
    fi

    if [[ "${workspace_ready}" == "true" \
      && -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
      set +u
      # shellcheck disable=SC1090
      if source "${WORKSPACE_ROOT}/install/setup.bash"; then
        overlay_ready="true"
      else
        printf 'unable to source workspace overlay: %s/install/setup.bash\n' \
          "${WORKSPACE_ROOT}" >&2
        failures=1
      fi
      set -u
      if [[ "${overlay_ready}" == "true" ]]; then
        for package_name in "${required_workspace_packages[@]}"; do
          check_ros_package "${package_name}" || failures=1
        done
      fi
    elif [[ "${workspace_ready}" == "true" ]]; then
      printf 'missing workspace overlay: %s/install/setup.bash\n' \
        "${WORKSPACE_ROOT}" >&2
      failures=1
    fi
  fi

  if [[ "${overlay_ready}" == "true" ]]; then
    echo 'ok: workspace overlay is sourceable'
  fi

  if [[ "${STRICT_TARGET}" == "true" ]]; then
    target_platform_status || failures=1
  elif [[ "${architecture}" != "aarch64" ]]; then
    echo 'info: non-arm64 host; Jetson, GPU, ZED, and target-runtime validation are deferred'
  else
    echo 'info: arm64 host detected; use --strict-target or --profile jetson to validate the lock'
  fi

  echo 'info: JetPack flashing, the ZED SDK/wrapper, device permissions, and hardware launches are intentionally outside bootstrap'
  return "${failures}"
}

run_command() {
  if [[ "${DRY_RUN}" == "true" ]]; then
    printf '+'
    printf ' %q' "$@"
    printf '\n'
    return 0
  fi
  "$@"
}

run_sudo() {
  run_command sudo "$@"
}

ros_apt_source_is_configured() {
  local source_file
  for source_file in /etc/apt/sources.list /etc/apt/sources.list.d/*.list \
    /etc/apt/sources.list.d/*.sources; do
    [[ -f "${source_file}" ]] || continue
    if grep -Fq 'packages.ros.org/ros2/ubuntu' "${source_file}" \
      && grep -Eq "(^Suites:[[:space:]]*${OS_CODENAME}([[:space:]]|$)|[[:space:]]${OS_CODENAME}[[:space:]]+main([[:space:]]|$))" \
        "${source_file}"; then
      return 0
    fi
  done
  return 1
}

ros_apt_source_is_present() {
  local source_file
  for source_file in /etc/apt/sources.list /etc/apt/sources.list.d/*.list \
    /etc/apt/sources.list.d/*.sources; do
    [[ -f "${source_file}" ]] || continue
    if grep -Fq 'packages.ros.org/ros2/ubuntu' "${source_file}"; then
      return 0
    fi
  done
  return 1
}

configure_ros_apt_source() {
  local architecture
  local source_entry

  if ros_apt_source_is_configured; then
    echo 'ok: ROS 2 apt source already configured'
    return 0
  fi
  if ros_apt_source_is_present; then
    printf 'ROS 2 apt source exists but is not configured for Ubuntu %s; repair it before bootstrap.\n' \
      "${OS_CODENAME}" >&2
    return 1
  fi

  architecture="$(dpkg --print-architecture)"
  source_entry="deb [arch=${architecture} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${OS_CODENAME} main"
  echo 'configuring the official ROS 2 apt source'

  if [[ "${DRY_RUN}" == "true" ]]; then
    echo '+ curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor --yes --output /usr/share/keyrings/ros-archive-keyring.gpg'
    printf '+ printf %%s %q | sudo tee /etc/apt/sources.list.d/ros2.list\n' "${source_entry}"
    return 0
  fi

  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | sudo gpg --dearmor --yes --output /usr/share/keyrings/ros-archive-keyring.gpg
  printf '%s\n' "${source_entry}" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
}

confirm_modifying_action() {
  local description="$1"
  local confirmation

  if [[ "${DRY_RUN}" == "true" || "${ASSUME_YES}" == "true" ]]; then
    return 0
  fi

  printf '%s\n' "${description}"
  read -r -p 'Continue? [y/N] ' confirmation
  if [[ "${confirmation}" != "y" && "${confirmation}" != "Y" ]]; then
    echo 'Bootstrap cancelled.'
    return 1
  fi
}

validate_install_platform() {
  read_os_release
  if [[ "${REQUESTED_ROS_DISTRO}" != "${TARGET_ROS_DISTRO}" ]]; then
    printf '%s\n' '--install supports only the locked ROS 2 Humble distribution.' >&2
    return 1
  fi
  if [[ "${OS_ID}" != "ubuntu" || "${OS_VERSION_ID}" != "${TARGET_UBUNTU_VERSION}" ]]; then
    printf 'unsupported installation platform: expected Ubuntu %s, found %s %s\n' \
      "${TARGET_UBUNTU_VERSION}" "${OS_ID}" "${OS_VERSION_ID}" >&2
    return 1
  fi
  if ! command -v apt-get >/dev/null 2>&1 || ! command -v sudo >/dev/null 2>&1; then
    echo '--install requires apt-get and sudo.' >&2
    return 1
  fi
}

install_repository_environment() {
  local -a bootstrap_packages=(
    ca-certificates
    curl
    gnupg
    lsb-release
    software-properties-common
  )
  local -a host_packages=(
    git
    build-essential
    cmake
    python3-colcon-common-extensions
    python3-pre-commit
    python3-rosdep
    python3-vcstool
    v4l-utils
  )
  local -a ros_tool_packages=(
    "ros-${TARGET_ROS_DISTRO}-ros-base"
    "ros-${TARGET_ROS_DISTRO}-rosbag2"
  )

  if [[ "${PROFILE}" == "development" ]]; then
    ros_tool_packages+=("ros-${TARGET_ROS_DISTRO}-rviz2")
  fi

  validate_workspace_layout
  validate_install_platform
  if [[ "${PROFILE}" == "jetson" || "${STRICT_TARGET}" == "true" ]]; then
    target_platform_status
  fi

  cat <<EOF
One-time bootstrap plan:
  workspace: ${WORKSPACE_ROOT} (${WORKSPACE_LAYOUT})
  profile:   ${PROFILE}
  platform:  Ubuntu ${TARGET_UBUNTU_VERSION}, ROS 2 ${TARGET_ROS_DISTRO}
  actions:   apt prerequisites, ROS source/base, rosdep dependencies, build, test
EOF
  confirm_modifying_action 'This will use sudo and modify system package configuration.'

  run_sudo apt-get update
  run_sudo apt-get install -y --no-install-recommends "${bootstrap_packages[@]}"
  run_sudo add-apt-repository -y universe
  run_sudo apt-get update
  configure_ros_apt_source
  run_sudo apt-get update
  run_sudo apt-get install -y --no-install-recommends "${host_packages[@]}"
  run_sudo apt-get install -y --no-install-recommends "${ros_tool_packages[@]}"

  if [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    echo 'ok: rosdep is already initialized'
  else
    run_sudo rosdep init
  fi
  run_command rosdep update
  run_command rosdep install --from-paths "${REPOSITORY_ROOT}" --ignore-src \
    --rosdistro "${TARGET_ROS_DISTRO}" -r -y

  if [[ "${DRY_RUN}" == "true" ]]; then
    printf '+ LUNABOT_WORKSPACE_ROOT=%q LUNABOT_ROS_DISTRO=%q %q\n' \
      "${WORKSPACE_ROOT}" "${TARGET_ROS_DISTRO}" "${SCRIPT_DIR}/build.sh"
    printf '+ LUNABOT_WORKSPACE_ROOT=%q LUNABOT_ROS_DISTRO=%q %q --skip-build\n' \
      "${WORKSPACE_ROOT}" "${TARGET_ROS_DISTRO}" "${SCRIPT_DIR}/test.sh"
    echo 'dry run complete: no package, rosdep, build, or test command was executed'
    return 0
  fi

  LUNABOT_WORKSPACE_ROOT="${WORKSPACE_ROOT}" \
    LUNABOT_ROS_DISTRO="${TARGET_ROS_DISTRO}" \
    "${SCRIPT_DIR}/build.sh"
  LUNABOT_WORKSPACE_ROOT="${WORKSPACE_ROOT}" \
    LUNABOT_ROS_DISTRO="${TARGET_ROS_DISTRO}" \
    "${SCRIPT_DIR}/test.sh" --skip-build

  echo 'Bootstrap completed. For each new terminal, run: source scripts/activate.sh'
  echo 'Optional developer hooks: scripts/bootstrap_dev.sh --install-hooks'
  run_check
}

install_host_tools_only() {
  local -a host_packages=(
    git
    python3-colcon-common-extensions
    python3-pre-commit
    python3-rosdep
    python3-vcstool
  )

  if ! command -v apt-get >/dev/null 2>&1 || ! command -v sudo >/dev/null 2>&1; then
    echo '--install-host-tools is supported only on apt-based hosts with sudo.' >&2
    return 1
  fi
  confirm_modifying_action 'This installs generic host tools only; it does not add ROS sources or install ROS.'
  run_sudo apt-get update
  run_sudo apt-get install -y --no-install-recommends "${host_packages[@]}"
}

install_hooks() {
  check_command pre-commit
  if ! git -C "${REPOSITORY_ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo '--install-hooks requires a Git working tree.' >&2
    return 1
  fi
  confirm_modifying_action 'This installs repository-local pre-commit hooks and may download hook environments.'
  if [[ "${DRY_RUN}" == "true" ]]; then
    printf '+ (cd %q && pre-commit install --install-hooks)\n' "${REPOSITORY_ROOT}"
    return 0
  fi
  (cd "${REPOSITORY_ROOT}" && pre-commit install --install-hooks)
}

parse_profile() {
  case "$1" in
    development|jetson)
      PROFILE="$1"
      ;;
    *)
      echo "Unknown profile: $1 (expected development or jetson)" >&2
      return 1
      ;;
  esac
}

select_action() {
  local requested_action="$1"
  if [[ "${ACTION_EXPLICIT}" == "true" && "${ACTION}" != "${requested_action}" ]]; then
    printf 'conflicting actions: %s and %s\n' "${ACTION}" "${requested_action}" >&2
    return 1
  fi
  ACTION="${requested_action}"
  ACTION_EXPLICIT="true"
}

validate_requested_ros_distro() {
  if [[ ! "${REQUESTED_ROS_DISTRO}" =~ ^[A-Za-z0-9_-]+$ ]]; then
    printf 'invalid ROS distribution name: %s\n' "${REQUESTED_ROS_DISTRO}" >&2
    return 1
  fi
}

main() {
  require_not_root

  while [[ "$#" -gt 0 ]]; do
    case "$1" in
      --check)
        select_action "check"
        ;;
      --install)
        select_action "install"
        ;;
      --install-hooks)
        select_action "install-hooks"
        ;;
      --install-host-tools)
        select_action "install-host-tools"
        ;;
      --profile)
        if [[ "$#" -lt 2 ]]; then
          echo '--profile requires development or jetson.' >&2
          return 2
        fi
        parse_profile "$2"
        shift
        ;;
      --workspace)
        if [[ "$#" -lt 2 ]]; then
          echo '--workspace requires a path.' >&2
          return 2
        fi
        normalize_workspace_root "$2"
        shift
        ;;
      --strict-target)
        STRICT_TARGET="true"
        ;;
      --yes)
        ASSUME_YES="true"
        ;;
      --dry-run)
        DRY_RUN="true"
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

  if [[ "${PROFILE}" == "jetson" ]]; then
    STRICT_TARGET="true"
  fi
  validate_requested_ros_distro

  case "${ACTION}" in
    check)
      run_check
      ;;
    install)
      install_repository_environment
      ;;
    install-hooks)
      install_hooks
      ;;
    install-host-tools)
      install_host_tools_only
      ;;
  esac
}

main "$@"
