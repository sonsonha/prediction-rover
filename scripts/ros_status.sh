#!/usr/bin/env bash
# Print a short ROS / Python / package status snapshot.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=ros_env.sh
source "${SCRIPT_DIR}/ros_env.sh"

echo "PROJECT_ROOT=${PROJECT_ROOT}"
echo "ROS_DISTRO=${ROS_DISTRO:-<unset>}"
echo -n "which python: "; command -v python
python --version
echo -n "prediction_ros prefix: "
ros2 pkg prefix prediction_ros
echo -n "safety_perception_msgs prefix: "
ros2 pkg prefix safety_perception_msgs
