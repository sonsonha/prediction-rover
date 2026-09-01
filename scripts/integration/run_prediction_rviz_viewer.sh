#!/usr/bin/env bash
# Launch RViz with Prediction dynamic visualization preset (inside Humble image).
set -eo pipefail
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_integration_env.sh"
set +u
source /opt/ros/humble/setup.bash
source "${INTEGRATION_WS}/install/setup.bash" 2>/dev/null || true
source "${PREDICTION_SRC_ROOT}/ros2/install_humble/setup.bash" 2>/dev/null || true
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export DISPLAY="${DISPLAY:-:10}"

RVIZ_CONFIG="${PREDICTION_SRC_ROOT}/config/rviz/prediction_dynamic_debug.rviz"
exec rviz2 -d "${RVIZ_CONFIG}" --ros-args -p use_sim_time:=true
