#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_CONFIG_FILE="${ROBOT_CONFIG_FILE:-${SCRIPT_DIR}/launch/robot_config.sh}"
SENSORS_LAUNCH="${SCRIPT_DIR}/launch/launch_sensors.sh"

if [[ ! -f "${ROBOT_CONFIG_FILE}" ]]; then
  echo "Robot config file not found: ${ROBOT_CONFIG_FILE}" >&2
  exit 1
fi

if [[ ! -x "${SENSORS_LAUNCH}" ]]; then
  echo "Sensor launcher not found or not executable: ${SENSORS_LAUNCH}" >&2
  exit 1
fi

SENSORS_PID=""

cleanup() {
  if [[ -n "${SENSORS_PID}" ]] && kill -0 "${SENSORS_PID}" >/dev/null 2>&1; then
    kill "${SENSORS_PID}" >/dev/null 2>&1 || true
    wait "${SENSORS_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

echo "Starting bot systems"
echo "  sensors: ${SENSORS_LAUNCH}"

"${SENSORS_LAUNCH}" &
SENSORS_PID="$!"

echo "Master launcher is running. Press Ctrl-C to stop all systems."
wait "${SENSORS_PID}"
