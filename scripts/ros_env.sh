# Shared ROS developer environment for prediction-src.
# Source from thin run scripts (or manually):  source scripts/ros_env.sh
#
# Sets PROJECT_ROOT and sources:
#   /opt/ros/jazzy/setup.bash
#   .venv/bin/activate
#   ros2/install/setup.bash
#
# Note: ROS/ament setup scripts reference optional unbound variables.
# If the caller has `set -u`, we briefly disable nounset while sourcing.

# Resolve repo root from this file's location (not cwd).
_ROS_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${_ROS_ENV_DIR}/.." && pwd)"
unset _ROS_ENV_DIR

_ros_env_restore_nounset=0
case "$-" in
  *u*) _ros_env_restore_nounset=1; set +u ;;
esac

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "error: ROS setup not found: ${ROS_SETUP}" >&2
  [[ "${_ros_env_restore_nounset}" -eq 1 ]] && set -u
  unset _ros_env_restore_nounset
  return 1 2>/dev/null || exit 1
fi
# shellcheck disable=SC1090
source "${ROS_SETUP}"

VENV_ACTIVATE="${PROJECT_ROOT}/.venv/bin/activate"
if [[ ! -f "${VENV_ACTIVATE}" ]]; then
  echo "error: Python venv not found: ${VENV_ACTIVATE}" >&2
  echo "  hint: make install" >&2
  [[ "${_ros_env_restore_nounset}" -eq 1 ]] && set -u
  unset _ros_env_restore_nounset
  return 1 2>/dev/null || exit 1
fi
# shellcheck disable=SC1090
source "${VENV_ACTIVATE}"

WS_SETUP="${PROJECT_ROOT}/ros2/install/setup.bash"
if [[ ! -f "${WS_SETUP}" ]]; then
  echo "error: built workspace setup not found: ${WS_SETUP}" >&2
  echo "  hint: make ros-build  (after sourcing /opt/ros/jazzy/setup.bash)" >&2
  [[ "${_ros_env_restore_nounset}" -eq 1 ]] && set -u
  unset _ros_env_restore_nounset
  return 1 2>/dev/null || exit 1
fi
# shellcheck disable=SC1090
source "${WS_SETUP}"

if [[ "${_ros_env_restore_nounset}" -eq 1 ]]; then
  set -u
fi
unset _ros_env_restore_nounset
