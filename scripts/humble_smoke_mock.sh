#!/usr/bin/env bash
# Smoke-test static|dynamic mock upstream against prediction_node (Humble).
set -euo pipefail

MODE="${1:-}"
case "${MODE}" in
  static|dynamic|dynamic_zero|dynamic_wrench) ;;
  *)
    echo "usage: $0 static|dynamic|dynamic_zero|dynamic_wrench" >&2
    exit 1
    ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=humble_env.sh
source "${SCRIPT_DIR}/humble_env.sh"

if [[ ! -f "${PROJECT_ROOT}/ros2/install_humble/setup.bash" ]]; then
  echo "error: Humble ROS workspace not built — run scripts/humble_build.sh" >&2
  exit 1
fi

PROFILE="${MODE}"
if [[ "${MODE}" == dynamic_zero || "${MODE}" == dynamic_wrench ]]; then
  PROFILE="dynamic"
fi

CONFIG_PATH="${PROJECT_ROOT}/config/rover.mock.yaml"
LOG_DIR="${TMPDIR:-/tmp}/prediction_humble_smoke"
mkdir -p "${LOG_DIR}"
PRED_LOG="${LOG_DIR}/prediction_${MODE}.log"
ECHO_LOG="${LOG_DIR}/echo_${MODE}.log"
MOCK_LOG="${LOG_DIR}/mock_${MODE}.log"

cleanup() {
  pkill -INT -f 'prediction.launch.py' 2>/dev/null || true
  pkill -INT -f 'prediction_ros/prediction_node' 2>/dev/null || true
  pkill -INT -f 'mock_upstream_node' 2>/dev/null || true
  pkill -INT -f 'ros2 topic echo /predict_output' 2>/dev/null || true
}
trap cleanup EXIT

cleanup
sleep 1

echo "→ launching prediction_node profile=${PROFILE}"
ros2 launch prediction_ros prediction.launch.py \
  "config_path:=${CONFIG_PATH}" \
  "prediction_profile:=${PROFILE}" >"${PRED_LOG}" 2>&1 &
PRED_PID=$!

for _ in $(seq 1 30); do
  if ros2 node list 2>/dev/null | grep -q '/prediction_node'; then
    break
  fi
  sleep 0.5
done

echo "→ waiting for /predict_output (timeout 30s)"
timeout 30 ros2 topic echo /predict_output --once >"${ECHO_LOG}" 2>&1 &
ECHO_PID=$!
sleep 1

echo "→ launching mock_upstream_node demo_mode=${MODE}"
ros2 run prediction_ros mock_upstream_node --ros-args -p "demo_mode:=${MODE}" >"${MOCK_LOG}" 2>&1 &
MOCK_PID=$!

if ! wait "${ECHO_PID}"; then
  echo "FAIL: no /predict_output within timeout" >&2
  echo "--- prediction log tail ---" >&2
  tail -40 "${PRED_LOG}" >&2 || true
  echo "--- mock log tail ---" >&2
  tail -20 "${MOCK_LOG}" >&2 || true
  exit 1
fi

if ! grep -q '^source_trajectory_id:' "${ECHO_LOG}"; then
  echo "FAIL: echo output missing source_trajectory_id" >&2
  exit 1
fi

if [[ "${PROFILE}" == dynamic ]]; then
  if ! grep -q 'stability_moment:' "${ECHO_LOG}" || ! grep -q 'valid: true' "${ECHO_LOG}"; then
    echo "FAIL: dynamic output missing valid stability_moment" >&2
    exit 1
  fi
  if ! grep -q 'zmp:' "${ECHO_LOG}"; then
    echo "FAIL: dynamic output missing zmp evidence" >&2
    exit 1
  fi
fi

SRC_ID="$(grep '^source_trajectory_id:' "${ECHO_LOG}" | head -1 | awk '{print $2}')"
echo "PASS ${MODE}: /predict_output source_trajectory_id=${SRC_ID}"
echo "  logs: ${LOG_DIR}"

kill "${MOCK_PID}" 2>/dev/null || true
kill "${PRED_PID}" 2>/dev/null || true
sleep 1
