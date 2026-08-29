#!/usr/bin/env bash
# Echo /predict_output. Works from any cwd.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=ros_env.sh
source "${SCRIPT_DIR}/ros_env.sh"

TOPIC="${1:-/predict_output}"
echo "→ ros2 topic echo ${TOPIC}"
exec ros2 topic echo "${TOPIC}"
