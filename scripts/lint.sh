#!/usr/bin/env bash
# Run repository-owned syntax and formatting checks.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if ! command -v pre-commit >/dev/null 2>&1; then
  echo 'pre-commit is required; run scripts/bootstrap_dev.sh --install-host-tools.' >&2
  exit 1
fi

for script in "${SCRIPT_DIR}"/*.sh; do
  bash -n "${script}"
done

python3 -m py_compile \
  "${SCRIPT_DIR}/check_tf_authority.py" \
  "${REPOSITORY_ROOT}/src/lb_launch/test/test_scaffold.py"

cd "${REPOSITORY_ROOT}"
exec pre-commit run --all-files
