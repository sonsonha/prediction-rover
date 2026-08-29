# Humble developer environment for prediction-src (Ubuntu 22.04 / Python 3.10).
# Use inside the upstream-style Docker container or any Humble host.
#
# Sets PROJECT_ROOT and sources:
#   /opt/ros/humble/setup.bash
#   .venv-humble/bin/activate (optional but recommended)
#   ros2/install_humble/setup.bash

_ROS_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${_ROS_ENV_DIR}/.." && pwd)"
unset _ROS_ENV_DIR

# ROS node subprocesses use system Python; keep mounted source importable.
export PYTHONPATH="${PROJECT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

_ros_env_restore_nounset=0
case "$-" in
  *u*) _ros_env_restore_nounset=1; set +u ;;
esac

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "error: ROS Humble setup not found: ${ROS_SETUP}" >&2
  echo "  hint: run inside docker/humble container (make humble-shell)" >&2
  [[ "${_ros_env_restore_nounset}" -eq 1 ]] && set -u
  unset _ros_env_restore_nounset
  return 1 2>/dev/null || exit 1
fi
# shellcheck disable=SC1090
source "${ROS_SETUP}"

VENV_ACTIVATE="${PROJECT_ROOT}/.venv-humble/bin/activate"
if [[ -f "${VENV_ACTIVATE}" ]]; then
  # shellcheck disable=SC1090
  source "${VENV_ACTIVATE}"
fi

WS_SETUP="${PROJECT_ROOT}/ros2/install_humble/setup.bash"
if [[ -f "${WS_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${WS_SETUP}"
fi

if [[ "${_ros_env_restore_nounset}" -eq 1 ]]; then
  set -u
fi
unset _ros_env_restore_nounset
