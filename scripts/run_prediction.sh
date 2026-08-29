#!/usr/bin/env bash
# Launch prediction_node (static | dynamic). Works from any cwd.
set -euo pipefail

PROFILE="${1:-}"
case "${PROFILE}" in
  static|dynamic) ;;
  *)
    echo "usage: $0 static|dynamic" >&2
    exit 1
    ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=ros_env.sh
source "${SCRIPT_DIR}/ros_env.sh"

CONFIG_PATH="${PROJECT_ROOT}/config/rover.mock.yaml"
if [[ ! -f "${CONFIG_PATH}" ]]; then
  echo "error: rover config not found: ${CONFIG_PATH}" >&2
  exit 1
fi

echo "→ prediction_node  profile=${PROFILE}  config=${CONFIG_PATH}"
exec ros2 launch prediction_ros prediction.launch.py \
  "config_path:=${CONFIG_PATH}" \
  "prediction_profile:=${PROFILE}"
