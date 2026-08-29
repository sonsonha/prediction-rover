#!/usr/bin/env bash
# Run pure-Python Prediction tests under Humble Python 3.10.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=humble_env.sh
source "${SCRIPT_DIR}/humble_env.sh"

cd "${PROJECT_ROOT}"
if [[ ! -f .venv-humble/bin/activate ]]; then
  echo "error: .venv-humble missing — run scripts/humble_install_python.sh first" >&2
  exit 1
fi
# shellcheck disable=SC1091
source .venv-humble/bin/activate

echo "→ pytest (Python $(python --version))"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
