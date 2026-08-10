#!/usr/bin/env bash
# Run repository-owned syntax and formatting checks.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if ! command -v pre-commit >/dev/null 2>&1; then
  echo 'pre-commit is required; run scripts/bootstrap_dev.sh --install-host-tools.' >&2
  exit 1
fi

cleanup_python_bytecode() {
  find "${REPOSITORY_ROOT}" -type d -name __pycache__ -prune -exec rm -rf {} +
}

trap cleanup_python_bytecode EXIT

for script in "${SCRIPT_DIR}"/*.sh; do
  bash -n "${script}"
done

mapfile -t PYTHON_FILES < <(
  find "${REPOSITORY_ROOT}" \
    -path "${REPOSITORY_ROOT}/.git" -prune -o \
    -path "${REPOSITORY_ROOT}/build" -prune -o \
    -path "${REPOSITORY_ROOT}/install" -prune -o \
    -path "${REPOSITORY_ROOT}/log" -prune -o \
    -path '*/__pycache__' -prune -o \
    -name '*.py' -type f -print | sort
)

python3 -m py_compile "${SCRIPT_DIR}/check_tf_authority.py" "${PYTHON_FILES[@]}"

cd "${REPOSITORY_ROOT}"
pre-commit run --all-files
