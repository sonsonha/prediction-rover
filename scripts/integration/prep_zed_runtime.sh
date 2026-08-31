#!/usr/bin/env bash
# Cached ZED runtime deps for integration gates (persisted on mounted volume).
set -eo pipefail
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_integration_env.sh"
set +u
source /opt/ros/humble/setup.bash
set -u

ZED_DEPS="${ZED_DEPS_DIR:-${PREDICTION_CACHE}/zed_deps}"
ZED_DESC="${ZED_DESC_DIR:-${PREDICTION_CACHE}/zed_desc}"

apt-get update -qq
apt-get install -y -qq \
  nlohmann-json3-dev libboost-dev \
  ros-humble-robot-localization ros-humble-diagnostic-updater \
  ros-humble-cv-bridge ros-humble-image-transport ros-humble-xacro \
  ros-humble-grid-map-msgs ros-humble-vision-msgs \
  ros-humble-zed-description \
  >/dev/null

if [ ! -f "${ZED_DEPS}/install/setup.bash" ]; then
  mkdir -p "${ZED_DEPS}/src"
  cd "${ZED_DEPS}/src"
  [ -d zed-ros2-interfaces ] || git clone --depth 1 https://github.com/stereolabs/zed-ros2-interfaces.git
  [ -d nmea_msgs ] || git clone --depth 1 --branch ros2 https://github.com/ros-drivers/nmea_msgs.git
  [ -d geographic_info ] || git clone --depth 1 --branch ros2 https://github.com/ros-geographic-info/geographic_info.git
  [ -d backward_ros ] || git clone --depth 1 --branch 1.0.8 https://github.com/pal-robotics/backward_ros.git
  [ -d angles ] || git clone --depth 1 --branch ros2 https://github.com/ros/angles.git
  [ -d diagnostics ] || git clone --depth 1 --branch ros2 https://github.com/ros/diagnostics.git
  cd "${ZED_DEPS}"
  colcon build --symlink-install
fi

if [ ! -d "${ZED_DESC}/install/share/zed_description" ]; then
  bash "${INTEGRATION_DIR}/ensure_zed_description.sh"
fi

echo "ZED runtime ready: ${ZED_DEPS} ${ZED_DESC}"
