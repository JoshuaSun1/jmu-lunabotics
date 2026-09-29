#!/usr/bin/env bash
# Capture baseline Jetson metrics while a separately launched test runs.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT_DIRECTORY="${REPOSITORY_ROOT}/performance-logs/$(date -u +%Y%m%dT%H%M%SZ)"
CHECK_ONLY="false"

usage() {
  cat <<'EOF'
Usage: scripts/performance_test.sh [--check] [--output DIRECTORY]

Checks for tegrastats, or records it until interrupted. Launch the workload in
another terminal. This Phase 0 utility does not claim a performance result.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --check)
      CHECK_ONLY="true"
      ;;
    --output)
      OUTPUT_DIRECTORY="${2:-}"
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
  shift
done

if ! command -v tegrastats >/dev/null 2>&1; then
  echo 'tegrastats is unavailable; run this only on the Jetson target.' >&2
  exit 1
fi

if [[ "${CHECK_ONLY}" == "true" ]]; then
  echo "ok: tegrastats available at $(command -v tegrastats)"
  exit 0
fi

mkdir -p "${OUTPUT_DIRECTORY}"
echo "Recording tegrastats to ${OUTPUT_DIRECTORY}/tegrastats.log; press Ctrl-C to stop."
exec tegrastats --interval 1000 --logfile "${OUTPUT_DIRECTORY}/tegrastats.log"
