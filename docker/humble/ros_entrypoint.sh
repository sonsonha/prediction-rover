#!/usr/bin/env bash
set -eo pipefail

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

if [[ -f /workspace/prediction-src/ros2/install_humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /workspace/prediction-src/ros2/install_humble/setup.bash
fi

if [[ -f /workspace/prediction-src/.venv-humble/bin/activate ]]; then
  # shellcheck disable=SC1091
  source /workspace/prediction-src/.venv-humble/bin/activate
fi

exec "$@"
