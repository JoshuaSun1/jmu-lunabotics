#!/usr/bin/env bash
# Source this file in each terminal after bootstrap has completed.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo 'This helper must be sourced: source scripts/activate.sh' >&2
  exit 1
fi

LB_ACTIVATE_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LB_ACTIVATE_REPOSITORY_ROOT="$(cd "${LB_ACTIVATE_SCRIPT_DIR}/.." && pwd)"
LB_ACTIVATE_CANONICAL_WORKSPACE="$(cd "${LB_ACTIVATE_REPOSITORY_ROOT}/../.." && pwd)"

if [[ -d "${LB_ACTIVATE_CANONICAL_WORKSPACE}/src/jmu-lunabotics" ]] \
  && [[ "$(cd "${LB_ACTIVATE_CANONICAL_WORKSPACE}/src/jmu-lunabotics" && pwd)" == "${LB_ACTIVATE_REPOSITORY_ROOT}" ]]; then
  LB_ACTIVATE_DEFAULT_WORKSPACE="${LB_ACTIVATE_CANONICAL_WORKSPACE}"
else
  LB_ACTIVATE_DEFAULT_WORKSPACE="${LB_ACTIVATE_REPOSITORY_ROOT}"
fi

LB_ACTIVATE_WORKSPACE_ROOT="${LUNABOT_WORKSPACE_ROOT:-${LB_ACTIVATE_DEFAULT_WORKSPACE}}"
LB_ACTIVATE_ROS_DISTRO="${LUNABOT_ROS_DISTRO:-humble}"
if [[ ! "${LB_ACTIVATE_ROS_DISTRO}" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "Invalid ROS distribution name: ${LB_ACTIVATE_ROS_DISTRO}" >&2
  return 1
fi
if [[ "${LB_ACTIVATE_ROS_DISTRO}" != "humble" ]]; then
  if [[ "${LUNABOT_ALLOW_NON_TARGET_ACTIVATION:-}" != "1" ]]; then
    cat >&2 <<EOF
Refusing to activate non-target ROS 2 ${LB_ACTIVATE_ROS_DISTRO}.
For an explicitly labelled structural-only check, set
LUNABOT_ALLOW_NON_TARGET_ACTIVATION=1 as well as LUNABOT_ROS_DISTRO.
EOF
    return 1
  fi
  echo "warning: activating non-target ROS 2 ${LB_ACTIVATE_ROS_DISTRO}; this is structural-only" >&2
fi
if [[ ! -d "${LB_ACTIVATE_WORKSPACE_ROOT}" ]]; then
  echo "Workspace root does not exist: ${LB_ACTIVATE_WORKSPACE_ROOT}" >&2
  return 1
fi
LB_ACTIVATE_WORKSPACE_ROOT="$(cd "${LB_ACTIVATE_WORKSPACE_ROOT}" && pwd)"
if [[ -d "${LB_ACTIVATE_WORKSPACE_ROOT}/src/jmu-lunabotics" ]] \
  && [[ "$(cd "${LB_ACTIVATE_WORKSPACE_ROOT}/src/jmu-lunabotics" && pwd)" == "${LB_ACTIVATE_REPOSITORY_ROOT}" ]]; then
  : # canonical <workspace>/src/jmu-lunabotics layout
elif [[ "${LB_ACTIVATE_WORKSPACE_ROOT}" == "${LB_ACTIVATE_REPOSITORY_ROOT}" ]]; then
  : # standalone/CI layout
else
  echo "Workspace does not contain this repository: ${LB_ACTIVATE_WORKSPACE_ROOT}" >&2
  return 1
fi
LB_ACTIVATE_UNDERLAY="/opt/ros/${LB_ACTIVATE_ROS_DISTRO}/setup.bash"
LB_ACTIVATE_OVERLAY="${LB_ACTIVATE_WORKSPACE_ROOT}/install/setup.bash"

if [[ ! -f "${LB_ACTIVATE_UNDERLAY}" ]]; then
  echo "ROS 2 ${LB_ACTIVATE_ROS_DISTRO} is not installed at ${LB_ACTIVATE_UNDERLAY}." >&2
  echo 'Run the one-time setup from a supported host: ./scripts/bootstrap_dev.sh --install --profile development' >&2
  return 1
fi
if [[ ! -f "${LB_ACTIVATE_OVERLAY}" ]]; then
  echo "Workspace overlay is not built at ${LB_ACTIVATE_OVERLAY}." >&2
  echo 'Run: ./scripts/build.sh' >&2
  return 1
fi

# ROS setup scripts read optional variables that may be unset in an interactive
# shell. Restore the caller's nounset setting once the underlay and overlay load.
LB_ACTIVATE_RESTORE_NOUNSET="false"
LB_ACTIVATE_SOURCE_STATUS=0
if [[ "$-" == *u* ]]; then
  LB_ACTIVATE_RESTORE_NOUNSET="true"
  set +u
fi
# shellcheck disable=SC1090
if source "${LB_ACTIVATE_UNDERLAY}" \
  && source "${LB_ACTIVATE_OVERLAY}"; then
  LB_ACTIVATE_SOURCE_STATUS=0
else
  LB_ACTIVATE_SOURCE_STATUS=$?
fi
if [[ "${LB_ACTIVATE_RESTORE_NOUNSET}" == "true" ]]; then
  set -u
fi
if [[ "${LB_ACTIVATE_SOURCE_STATUS}" -ne 0 ]]; then
  echo 'Unable to source the ROS underlay or Lunabotics workspace overlay.' >&2
  return "${LB_ACTIVATE_SOURCE_STATUS}"
fi

export LUNABOT_WORKSPACE_ROOT="${LB_ACTIVATE_WORKSPACE_ROOT}"
export LUNABOT_ROS_DISTRO="${LB_ACTIVATE_ROS_DISTRO}"
printf 'Activated Lunabotics workspace: %s (ROS 2 %s)\n' \
  "${LUNABOT_WORKSPACE_ROOT}" "${LUNABOT_ROS_DISTRO}"

unset LB_ACTIVATE_SCRIPT_DIR
unset LB_ACTIVATE_REPOSITORY_ROOT
unset LB_ACTIVATE_CANONICAL_WORKSPACE
unset LB_ACTIVATE_DEFAULT_WORKSPACE
unset LB_ACTIVATE_WORKSPACE_ROOT
unset LB_ACTIVATE_ROS_DISTRO
unset LB_ACTIVATE_UNDERLAY
unset LB_ACTIVATE_OVERLAY
unset LB_ACTIVATE_RESTORE_NOUNSET
unset LB_ACTIVATE_SOURCE_STATUS
