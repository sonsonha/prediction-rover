#!/usr/bin/env bash
# Create / refresh the Humble Python venv (system-site-packages for rclpy).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=humble_env.sh
source "${SCRIPT_DIR}/humble_env.sh"

cd "${PROJECT_ROOT}"

# ROS launch spawns nodes with system Python; install prediction_core where rclpy can import it.
echo "→ pip3 install -e '.[test]' (container system site-packages)"
pip3 install -e '.[test]'

if [[ ! -d .venv-humble ]]; then
  echo "→ creating .venv-humble (system-site-packages)"
  python3 -m venv --system-site-packages .venv-humble
fi
# shellcheck disable=SC1091
source .venv-humble/bin/activate

echo "→ pip install -e '.[test]' (venv)"
python -m pip install --upgrade pip
python -m pip install -e '.[test]'

echo "→ python: $(command -v python) ($(python --version))"
python -c "import prediction_core; print('prediction_core:', prediction_core.__file__)"
