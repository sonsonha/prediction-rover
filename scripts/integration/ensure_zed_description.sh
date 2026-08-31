#!/usr/bin/env bash
# One-time fix for zed_description (colcon only, no apt-get). Safe to call outside gate.
set -eo pipefail
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_integration_env.sh"
set +u
source /opt/ros/humble/setup.bash
set -u

ZED_DESC="${ZED_DESC_DIR:-${PREDICTION_CACHE}/zed_desc}"

have_zed_desc() {
  ros2 pkg prefix zed_description >/dev/null 2>&1
}

have_xacro() {
  command -v xacro >/dev/null 2>&1
}

if have_zed_desc && have_xacro; then
  echo "zed_description OK: $(ros2 pkg prefix zed_description)"
  echo "xacro OK: $(command -v xacro)"
  exit 0
fi

if [[ -f "${ZED_DESC}/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${ZED_DESC}/install/setup.bash"
  set -u
  if have_zed_desc && have_xacro; then
    echo "zed_description OK from cache: $(ros2 pkg prefix zed_description)"
    echo "xacro OK: $(command -v xacro)"
    exit 0
  fi
fi

echo "Building zed_description into ${ZED_DESC} ..."
mkdir -p "${ZED_DESC}/src"
cd "${ZED_DESC}/src"

if [[ ! -d zed-ros2-description ]]; then
  if [[ -d zed-ros2-examples ]]; then
    echo "Removing wrong zed-ros2-examples clone (no zed_description package)"
    rm -rf zed-ros2-examples
  fi
  git clone --depth 1 https://github.com/stereolabs/zed-ros2-description.git
fi

cd "${ZED_DESC}"
colcon build --packages-select zed_description --symlink-install

set +u
# shellcheck disable=SC1091
source "${ZED_DESC}/install/setup.bash"
set -u

if ! have_zed_desc; then
  if [[ "${ENSURE_ALLOW_APT:-0}" == "1" ]]; then
    echo "colcon failed; one-time apt install (ENSURE_ALLOW_APT=1) ..."
    apt-get update -qq
    apt-get install -y -qq ros-humble-zed-description ros-humble-xacro >/dev/null
  else
    echo "FAIL: zed_description still missing after colcon build" >&2
    exit 1
  fi
fi

if ! have_xacro; then
  if [[ "${ENSURE_ALLOW_APT:-0}" == "1" ]]; then
    echo "xacro missing; one-time apt install ros-humble-xacro ..."
    apt-get update -qq
    apt-get install -y -qq ros-humble-xacro >/dev/null
  else
    echo "FAIL: xacro not found (set ENSURE_ALLOW_APT=1 for one-time apt outside gate)" >&2
    exit 1
  fi
fi

if ! have_zed_desc || ! have_xacro; then
  echo "FAIL: zed_description=$(have_zed_desc && echo OK || echo MISSING) xacro=$(have_xacro && echo OK || echo MISSING)" >&2
  exit 1
fi

echo "zed_description ready: $(ros2 pkg prefix zed_description)"
