#!/usr/bin/env bash
# ZED runtime preflight for fresh prediction-humble-dev container.
# No apt-get. Does NOT run full integration gate.
set -eo pipefail
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_integration_env.sh"

LOG_DIR="${1:-${PREDICTION_LOGS}/zed_preflight_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${LOG_DIR}"
REPORT="${LOG_DIR}/zed_preflight_report.json"

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG_DIR}/run.log"; }

BLOCKER=""
REBUILD_PASS="UNKNOWN"
ROS_RESULTS=()
LDD_UNRESOLVED=()
ZED_LIB=""
ZED_ONLY="SKIP"
GATE_RAN="NO"

write_report() {
  python3 - <<PY
import json
from pathlib import Path
unresolved = []
p = Path("${LOG_DIR}/ldd_unresolved.txt")
if p.exists():
    unresolved = [ln.strip() for ln in p.read_text().splitlines() if ln.strip()]
report = {
    "docker_rebuild": "${REBUILD_PASS}",
    "ros_packages": {
        "xacro": "${ROS_XACRO:-UNKNOWN}",
        "zed_description": "${ROS_ZED_DESC:-UNKNOWN}",
        "robot_localization": "${ROS_ROBOT_LOC:-UNKNOWN}",
        "diagnostic_updater": "${ROS_DIAG_UPD:-UNKNOWN}",
        "vision_msgs": "${ROS_VISION:-UNKNOWN}",
    },
    "python": {
        "torch": "${PY_TORCH:-}",
        "cuda": "${PY_CUDA:-}",
        "ultralytics": "${PY_ULTRA:-}",
        "cv2": "${PY_CV2:-}",
    },
    "zed_component_library": "${ZED_LIB}",
    "ldd_unresolved_count": len(unresolved),
    "ldd_unresolved": unresolved,
    "zed_only_load": "${ZED_ONLY}",
    "blocker": """${BLOCKER}""".strip(),
    "log_dir": "${LOG_DIR}",
    "gate_ran": "${GATE_RAN}",
}
Path("${REPORT}").write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
PY
}

check_ros_pkg() {
  local pkg="$1"
  local var="$2"
  if out=$(ros2 pkg prefix "${pkg}" 2>&1); then
    log "OK ros2 pkg ${pkg} => ${out}"
    eval "${var}=PASS"
    return 0
  fi
  log "FAIL ros2 pkg ${pkg}: ${out}"
  eval "${var}=FAIL"
  BLOCKER="ros2 pkg ${pkg} missing: ${out}"
  return 1
}

log "LOG_DIR=${LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${PREDICTION_CACHE}/zed_desc/install/setup.bash" 2>/dev/null || true
source "${INTEGRATION_WS}/install/setup.bash" 2>/dev/null || true
set -u

# --- 1. ROS package preflight ---
FAIL=0
check_ros_pkg xacro ROS_XACRO || FAIL=1
check_ros_pkg zed_description ROS_ZED_DESC || FAIL=1
check_ros_pkg robot_localization ROS_ROBOT_LOC || FAIL=1
check_ros_pkg diagnostic_updater ROS_DIAG_UPD || FAIL=1
check_ros_pkg vision_msgs ROS_VISION || FAIL=1

if ! python3 - <<'PY' >"${LOG_DIR}/python_preflight.txt" 2>&1; then
import torch
import ultralytics
import cv2
print("torch", torch.__version__)
print("cuda", torch.cuda.is_available())
print("ultralytics", ultralytics.__version__)
print("cv2", cv2.__version__)
PY
  log "FAIL Python preflight"
  cat "${LOG_DIR}/python_preflight.txt" | tee -a "${LOG_DIR}/run.log"
  BLOCKER="${BLOCKER:-Python/CUDA preflight failed}"
  FAIL=1
else
  cat "${LOG_DIR}/python_preflight.txt" | tee -a "${LOG_DIR}/run.log"
  PY_TORCH=$(grep '^torch ' "${LOG_DIR}/python_preflight.txt" | awk '{print $2}')
  PY_CUDA=$(grep '^cuda ' "${LOG_DIR}/python_preflight.txt" | awk '{print $2}')
  PY_ULTRA=$(grep '^ultralytics ' "${LOG_DIR}/python_preflight.txt" | awk '{print $2}')
  PY_CV2=$(grep '^cv2 ' "${LOG_DIR}/python_preflight.txt" | awk '{print $2}')
fi

if [[ "${FAIL}" -ne 0 ]]; then
  write_report
  exit 2
fi

# --- 2. ZED component shared-library preflight ---
ZED_LIB=$(find /opt/ros/humble ${INTEGRATION_WS}/install -name 'libzed_camera_component.so' 2>/dev/null | head -1 || true)
log "ZED_LIB=${ZED_LIB:-NOT_FOUND}"

if [[ -z "${ZED_LIB}" ]]; then
  BLOCKER="libzed_camera_component.so not found in fresh container"
  write_report
  exit 2
fi

ldd "${ZED_LIB}" | tee "${LOG_DIR}/ldd_zed_camera_component.txt"
grep -i 'not found' "${LOG_DIR}/ldd_zed_camera_component.txt" | tee "${LOG_DIR}/ldd_unresolved.txt" || true
UNRESOLVED_COUNT=$(grep -ci 'not found' "${LOG_DIR}/ldd_unresolved.txt" 2>/dev/null || echo 0)
ldconfig -p 2>/dev/null | grep -E 'robot_localization|zed' | tee "${LOG_DIR}/ldconfig_grep.txt" || true

LDD_UNRESOLVED_JSON=$(python3 - <<PY
import json
from pathlib import Path
lines = Path("${LOG_DIR}/ldd_unresolved.txt").read_text().strip().splitlines() if Path("${LOG_DIR}/ldd_unresolved.txt").exists() else []
print(json.dumps(lines))
PY
)

if [[ "${UNRESOLVED_COUNT}" -gt 0 ]]; then
  BLOCKER="ldd unresolved dependencies (${UNRESOLVED_COUNT}): $(tr '\n' '; ' < "${LOG_DIR}/ldd_unresolved.txt")"
  write_report
  exit 2
fi
log "ldd unresolved count: 0"

# --- 3. ZED-only fail-fast test (30s) ---
SVO="${RAW_DATA}/session_20260710_0924/zed/zed_20260710_092420_0001.svo2"
GPS="${RAW_DATA}/session_20260710_0924/mavlink/extracted/session_20260710_0924_mavlink/gps.csv"
ATT="${RAW_DATA}/session_20260710_0924/mavlink/extracted/session_20260710_0924_mavlink/attitude.csv"

log "ZED-only startup test (30s max) ..."
timeout 30s ros2 launch lr_bringup rover.launch.py \
  camera_model:=zed2i svo_path:="${SVO}" gps_path:="${GPS}" attitude_path:="${ATT}" \
  publish_svo_clock:=true svo_fps:=15.0 use_rviz:=false accumulate_cloud:=false \
  param_overrides:='depth.depth_mode:=PERFORMANCE;depth.point_cloud_freq:=5.0' \
  >"${LOG_DIR}/zed_only_launch.log" 2>&1 &
ZED_PID=$!

ZED_ONLY_OK=false
for _ in $(seq 1 25); do
  if grep -q "=== zed started ===" "${LOG_DIR}/zed_only_launch.log" 2>/dev/null; then
    ZED_ONLY_OK=true
    break
  fi
  if grep -qiE 'Failed to load library|dlopen error|not found.*\.so' "${LOG_DIR}/zed_only_launch.log" 2>/dev/null; then
    break
  fi
  sleep 1
done

kill -INT "${ZED_PID}" 2>/dev/null || true
wait "${ZED_PID}" 2>/dev/null || true

if [[ "${ZED_ONLY_OK}" == "true" ]]; then
  ZED_ONLY="PASS"
  log "ZED-only load PASS"
else
  ZED_ONLY="FAIL"
  BLOCKER="ZED-only startup failed; see ${LOG_DIR}/zed_only_launch.log"
  tail -30 "${LOG_DIR}/zed_only_launch.log" | tee -a "${LOG_DIR}/run.log"
  write_report
  exit 2
fi

write_report
log "ZED preflight complete — ready for full gate"
exit 0
