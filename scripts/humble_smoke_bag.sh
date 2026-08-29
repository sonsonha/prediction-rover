#!/usr/bin/env bash
# Smoke-test recorded rosbag against prediction_node (Humble), short playback.
set -euo pipefail

BAG_PATH="${1:-/workspace/bags/manual_rollover_20260818_080815}"
DURATION="${2:-8}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=humble_env.sh
source "${SCRIPT_DIR}/humble_env.sh"

if [[ ! -d "${BAG_PATH}" ]]; then
  echo "error: bag not found: ${BAG_PATH}" >&2
  exit 1
fi
if [[ ! -f "${PROJECT_ROOT}/ros2/install_humble/setup.bash" ]]; then
  echo "error: Humble ROS workspace not built — run scripts/humble_build.sh" >&2
  exit 1
fi

CONFIG_PATH="${PROJECT_ROOT}/config/rover.mock.yaml"
LOG_DIR="${TMPDIR:-/tmp}/prediction_humble_bag"
mkdir -p "${LOG_DIR}"
PRED_LOG="${LOG_DIR}/prediction_bag.log"
ECHO_LOG="${LOG_DIR}/echo_bag.log"
PLAY_LOG="${LOG_DIR}/play_bag.log"

cleanup() {
  pkill -INT -f 'prediction.launch.py' 2>/dev/null || true
  pkill -INT -f 'prediction_ros/prediction_node' 2>/dev/null || true
  pkill -INT -f 'ros2 bag play' 2>/dev/null || true
  pkill -INT -f 'ros2 topic echo /predict_output' 2>/dev/null || true
}
trap cleanup EXIT

cleanup
sleep 1

echo "→ bag info"
ros2 bag info "${BAG_PATH}" | head -20

echo "→ launching prediction_node profile=dynamic"
ros2 launch prediction_ros prediction.launch.py \
  "config_path:=${CONFIG_PATH}" \
  "prediction_profile:=dynamic" >"${PRED_LOG}" 2>&1 &
PRED_PID=$!

for _ in $(seq 1 30); do
  if ros2 node list 2>/dev/null | grep -q '/prediction_node'; then
    break
  fi
  sleep 0.5
done

echo "→ waiting for first /predict_output (timeout 45s)"
timeout 45 ros2 topic echo /predict_output --once >"${ECHO_LOG}" 2>&1 &
ECHO_PID=$!
sleep 1

echo "→ ros2 bag play (timeout ${DURATION}s)"
timeout "${DURATION}" ros2 bag play "${BAG_PATH}" --disable-keyboard-controls >"${PLAY_LOG}" 2>&1 &
PLAY_PID=$!

if ! wait "${ECHO_PID}"; then
  echo "FAIL: no /predict_output during bag playback" >&2
  tail -40 "${PRED_LOG}" >&2 || true
  tail -20 "${PLAY_LOG}" >&2 || true
  exit 1
fi
wait "${PLAY_PID}" || true

SRC_ID="$(grep '^source_trajectory_id:' "${ECHO_LOG}" | head -1 | awk '{print $2}')"
PUBLISHED="$(grep -c 'Prediction published cycle' "${PRED_LOG}" || true)"
echo "PASS bag: first source_trajectory_id=${SRC_ID}, published_cycles=${PUBLISHED}"
if ! grep -q 'valid: true' "${ECHO_LOG}"; then
  echo "FAIL: stability/zmp not valid in first output" >&2
  exit 1
fi
echo "  logs: ${LOG_DIR}"

kill "${PRED_PID}" 2>/dev/null || true
sleep 1
