#!/usr/bin/env bash
set -eo pipefail

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

if [[ -f /workspace/prediction-src/ros2/install_humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /workspace/prediction-src/ros2/install_humble/setup.bash
fi

# Optional: activate .venv-humble only when explicitly requested.
# Default OFF — activating the venv breaks colcon/ament (CMake invokes
# /usr/bin/python3 which then misses ament_package on PYTHONPATH).
if [[ "${PREDICTION_USE_VENV:-0}" == "1" ]] \
  && [[ -f /workspace/prediction-src/.venv-humble/bin/activate ]]; then
  # shellcheck disable=SC1091
  source /workspace/prediction-src/.venv-humble/bin/activate
fi

exec "$@"
