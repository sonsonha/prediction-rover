#!/usr/bin/env bash
# Replay canonical Prediction INPUT bag against static then dynamic profiles.
set -eo pipefail
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_integration_env.sh"
set +u
source /opt/ros/humble/setup.bash
source "${PREDICTION_CACHE}/zed_deps/install/setup.bash" 2>/dev/null || true
source "${PREDICTION_CACHE}/zed_desc/install/setup.bash" 2>/dev/null || true
source "${INTEGRATION_WS}/install/setup.bash"
source "${PREDICTION_SRC_ROOT}/ros2/install_humble/setup.bash"
set -u

export PYTHONPATH="${PREDICTION_SRC_ROOT}":${PYTHONPATH:-}
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

BAG="${1:-${PREDICTION_BAGS}/session_0924_pipe_prediction_inputs}"
CONFIG="${PREDICTION_SRC_ROOT}/config/rover.mock.yaml"
LOG_DIR="${PREDICTION_LOGS}/bag_replay_$(date +%Y%m%d_%H%M%S)"
MONITOR="${INTEGRATION_DIR}/condition_gate_monitor.py"
REPLAY_DURATION="${REPLAY_DURATION:-50}"
TARGET_PREDICT="${TARGET_PREDICT:-5}"

mkdir -p "${LOG_DIR}"

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG_DIR}/run.log"; }

if [[ ! -f "${BAG}/metadata.yaml" ]]; then
  log "FAIL: bag not found: ${BAG}"
  exit 1
fi

log "LOG_DIR=${LOG_DIR} BAG=${BAG}"
log "bag topics:"
ros2 bag info "${BAG}" | tee "${LOG_DIR}/bag_info.txt"

for forbidden in /predict_output /clock; do
  if grep -q "Topic: ${forbidden}" "${LOG_DIR}/bag_info.txt" 2>/dev/null; then
    log "FAIL: bag must not contain ${forbidden}; run remux_canonical_bag.sh first"
    exit 2
  fi
done

STATIC_EXIT=0
DYNAMIC_EXIT=0

log "=== static replay ==="
ros2 run prediction_ros prediction_node --ros-args \
  -p use_sim_time:=true -p config_path:="${CONFIG}" -p prediction_profile:=static \
  >"${LOG_DIR}/replay_static_prediction.log" 2>&1 &
PRED_PID=$!
sleep 2
python3 "${MONITOR}" --log-dir "${LOG_DIR}" --bag-dir "${BAG}" --phase replay-static \
  --replay-duration-s "${REPLAY_DURATION}" --target-predict "${TARGET_PREDICT}" \
  | tee "${LOG_DIR}/report_replay_static.json" || STATIC_EXIT=$?
kill -INT "${PRED_PID}" 2>/dev/null || true
wait "${PRED_PID}" 2>/dev/null || true

log "=== dynamic replay ==="
ros2 run prediction_ros prediction_node --ros-args \
  -p use_sim_time:=true -p config_path:="${CONFIG}" -p prediction_profile:=dynamic \
  >"${LOG_DIR}/replay_dynamic_prediction.log" 2>&1 &
PRED_PID=$!
sleep 2
python3 "${MONITOR}" --log-dir "${LOG_DIR}" --bag-dir "${BAG}" --phase replay-dynamic \
  --replay-duration-s "${REPLAY_DURATION}" --target-predict "${TARGET_PREDICT}" \
  | tee "${LOG_DIR}/report_replay_dynamic.json" || DYNAMIC_EXIT=$?
kill -INT "${PRED_PID}" 2>/dev/null || true
wait "${PRED_PID}" 2>/dev/null || true

python3 - <<PY
import json
from pathlib import Path
log = Path("${LOG_DIR}")
static = json.loads((log / "report_replay-static.json").read_text()) if (log / "report_replay-static.json").exists() else {}
dynamic = json.loads((log / "report_replay-dynamic.json").read_text()) if (log / "report_replay-dynamic.json").exists() else {}
summary = {
    "bag": "${BAG}",
    "static": {
        "success": static.get("success"),
        "predict_output_messages": static.get("predict_output_messages"),
        "duplicate_source_trajectory_ids": static.get("once_per_trajectory", {}).get("duplicate_prediction_source_ids", []),
    },
    "dynamic": {
        "success": dynamic.get("success"),
        "predict_output_messages": dynamic.get("predict_output_messages"),
        "duplicate_source_trajectory_ids": dynamic.get("once_per_trajectory", {}).get("duplicate_prediction_source_ids", []),
    },
    "log_dir": str(log),
}
(log / "summary.json").write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
PY

log "DONE ${LOG_DIR}"
exit $(( STATIC_EXIT != 0 || DYNAMIC_EXIT != 0 ))
