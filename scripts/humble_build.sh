#!/usr/bin/env bash
# Build safety_perception_msgs + prediction_ros + prediction_visualization for Humble.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=humble_env.sh
source "${SCRIPT_DIR}/humble_env.sh"

cd "${PROJECT_ROOT}/ros2"
echo "→ colcon build (Humble) into install_humble/"
colcon build \
  --build-base build_humble \
  --install-base install_humble \
  --packages-select safety_perception_msgs prediction_ros prediction_visualization

_restore_nounset=0
case "$-" in
  *u*) _restore_nounset=1; set +u ;;
esac
# shellcheck disable=SC1091
source install_humble/setup.bash
if [[ "${_restore_nounset}" -eq 1 ]]; then set -u; fi
unset _restore_nounset
echo "→ ros2 pkg prefix prediction_ros: $(ros2 pkg prefix prediction_ros)"
echo "→ ros2 pkg prefix prediction_visualization: $(ros2 pkg prefix prediction_visualization)"
