#!/usr/bin/env bash
# Short condition-driven gate (max 180s upstream). No apt-get. One upstream attempt.
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
export LR_SEGMENTATION_MODEL_PATH="${LR_SEGMENTATION_MODEL_PATH:-${LR_ROS2}/models/best.pt}"

SVO="${RAW_DATA}/session_20260710_0924/zed/zed_20260710_092420_0001.svo2"
GPS="${RAW_DATA}/session_20260710_0924/mavlink/extracted/session_20260710_0924_mavlink/gps.csv"
ATT="${RAW_DATA}/session_20260710_0924/mavlink/extracted/session_20260710_0924_mavlink/attitude.csv"
MODEL="${LR_SEGMENTATION_MODEL_PATH}"
CONFIG="${PREDICTION_SRC_ROOT}/config/rover.mock.yaml"
BAG="${PREDICTION_BAGS}/session_0924_pipe_prediction_inputs"
LOG_DIR="${PREDICTION_LOGS}/condition_gate_$(date +%Y%m%d_%H%M%S)"
MONITOR="${INTEGRATION_DIR}/condition_gate_monitor.py"
GATE_LIMIT=180

mkdir -p "${LOG_DIR}" "${PREDICTION_BAGS}"
PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]}"; do kill -KILL "${pid}" 2>/dev/null || true; done
}
trap cleanup EXIT

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG_DIR}/run.log"; }

write_early_summary() {
  local blocker="$1"
  python3 - <<PY
import json
from pathlib import Path
blocker = """${blocker}"""
summary = {
    "1_build_test": "SKIP",
    "2_tracked_objects_runtime": "FAIL",
    "3_first_nonempty_tracked_sim_t": None,
    "4_geometry_at_first_tracked": False,
    "5_rover_state_at_first_tracked": False,
    "6_canonical_bag_created": False,
    "7_static_prediction": "SKIP",
    "8_dynamic_prediction": "SKIP",
    "9_collision_steps": None,
    "10_rollover": None,
    "11_once_per_trajectory": None,
    "12_blocker": blocker,
    "log_dir": "${LOG_DIR}",
    "bag_path": "${BAG}",
}
Path("${LOG_DIR}/summary.json").write_text(json.dumps(summary, indent=2))
Path("${LOG_DIR}/report_upstream.json").write_text(json.dumps({
    "phase": "upstream", "success": False, "blocker": blocker,
    "readiness": {"trajectory": False, "geometry": False, "rover_state": False, "nonempty_tracked": False},
    "bag_created": False,
}, indent=2))
PY
}

log "LOG_DIR=${LOG_DIR} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} GATE_LIMIT=${GATE_LIMIT}s"

# --- Ensure zed_description + xacro (colcon cache fix; apt only via ensure script) ---
if ! ros2 pkg prefix zed_description >/dev/null 2>&1 || ! command -v xacro >/dev/null 2>&1; then
  log "zed_description/xacro missing; running ensure_zed_description.sh"
  if ! bash "${INTEGRATION_DIR}/ensure_zed_description.sh" >>"${LOG_DIR}/ensure_zed_desc.log" 2>&1; then
    log "FAIL: ensure_zed_description.sh (rebuild prediction-humble-dev image with ros-humble-xacro)"
    write_early_summary "ensure_zed_description failed: xacro or zed_description missing in container image"
    exit 2
  fi
  set +u
  # shellcheck disable=SC1091
  source "${PREDICTION_CACHE}/zed_desc/install/setup.bash"
  set -u
fi

# --- Preflight (no apt-get): required packages must already exist ---
if ! ros2 pkg prefix zed_description >/dev/null 2>&1; then
  log "FAIL preflight: zed_description not in ROS environment after ensure step"
  write_early_summary "zed_description not in ROS environment after ensure step"
  exit 2
fi
if ! command -v xacro >/dev/null 2>&1; then
  log "FAIL preflight: xacro not found (rebuild prediction-humble-dev image with ros-humble-xacro)"
  write_early_summary "xacro not found in container PATH (rebuild image with ros-humble-xacro)"
  exit 2
fi
if ! python3 -c "import torch, ultralytics" >/dev/null 2>&1; then
  log "FAIL preflight: torch/ultralytics missing in runtime environment"
  exit 2
fi
if ! find /opt/ros/humble ${PREDICTION_CACHE}/zed_deps/install -name 'librobot_localization__rosidl_typesupport_cpp.so' 2>/dev/null | grep -q .; then
  log "FAIL preflight: librobot_localization__rosidl_typesupport_cpp.so not found"
  write_early_summary "librobot_localization__rosidl_typesupport_cpp.so missing (rebuild image with ros-humble-robot-localization)"
  exit 2
fi
log "preflight PASS (zed_description, xacro, robot_localization, torch, ultralytics)"

# --- Build/test (quick, no rebuild) ---
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
if ! python3 -m pytest "${PREDICTION_SRC_ROOT}/ros2/lr_prediction_bridge/test/test_detection3d_conversion.py" -q \
  >"${LOG_DIR}/pytest.log" 2>&1; then
  log "FAIL unit tests"; cat "${LOG_DIR}/pytest.log"; exit 1
fi
log "unit tests PASS"

# --- Start ALL upstream consumers together (no empty_objects bridge) ---
ros2 launch lr_bringup rover.launch.py \
  camera_model:=zed2i svo_path:="${SVO}" gps_path:="${GPS}" attitude_path:="${ATT}" \
  publish_svo_clock:=true svo_fps:=15.0 future_path_frames:=200 use_rviz:=false \
  accumulate_cloud:=false map_frame_step:=5 \
  param_overrides:='depth.depth_mode:=PERFORMANCE;depth.point_cloud_freq:=5.0' \
  >"${LOG_DIR}/rover.log" 2>&1 &
PIDS+=($!)

ros2 launch lr_segmentation segmentation.launch.py \
  use_sim_time:=true model_path:="${MODEL}" model_device:=0 \
  >"${LOG_DIR}/segmentation.log" 2>&1 &
PIDS+=($!)

ros2 launch lr_terrain_geometry terrain_geometry.launch.py \
  use_sim_time:=true point_cloud_topic:=/lr/point_cloud/cloud_in_map \
  sensor_frame:=zed_camera_link object_filter_enabled:=true \
  >"${LOG_DIR}/terrain.log" 2>&1 &
PIDS+=($!)

ros2 run lr_prediction_bridge trajectory_adapter_node --ros-args -p use_sim_time:=true \
  >"${LOG_DIR}/bridge_traj.log" 2>&1 &
PIDS+=($!)

ros2 run lr_prediction_bridge geometry_adapter_node --ros-args \
  -p use_sim_time:=true -p allow_flat_fallback:=false \
  >"${LOG_DIR}/bridge_geom.log" 2>&1 &
PIDS+=($!)

ros2 run lr_prediction_bridge rover_state_adapter_node --ros-args -p use_sim_time:=true \
  >"${LOG_DIR}/bridge_rover.log" 2>&1 &
PIDS+=($!)

ros2 run lr_prediction_bridge tracked_objects_adapter_node --ros-args -p use_sim_time:=true \
  >"${LOG_DIR}/bridge_tracked.log" 2>&1 &
PIDS+=($!)

ros2 run prediction_ros prediction_node --ros-args \
  -p use_sim_time:=true -p config_path:="${CONFIG}" -p prediction_profile:=static \
  >"${LOG_DIR}/prediction_static.log" 2>&1 &
PIDS+=($!)

log "all nodes started concurrently"

# Early ZED failure detection (no long warmup — fail fast within 20s)
ZED_OK=false
for _ in $(seq 1 20); do
  if grep -q "=== zed started ===" "${LOG_DIR}/rover.log" 2>/dev/null; then
    ZED_OK=true
    break
  fi
  if grep -q "zed_description' not found\|Caught exception in launch\|No such file or directory: 'xacro'" "${LOG_DIR}/rover.log" 2>/dev/null; then
    break
  fi
  sleep 1
done
if [[ "${ZED_OK}" != "true" ]]; then
  log "FAIL: ZED did not start (see rover.log)"
  tail -20 "${LOG_DIR}/rover.log" | tee -a "${LOG_DIR}/run.log"
  blocker="ZED launch failed before replay (check zed_description / xacro / robot_localization deps)"
  python3 - <<PY
import json
from pathlib import Path
blocker = "${blocker}"
report = {
    "phase": "upstream",
    "success": False,
    "blocker": blocker,
    "readiness": {"trajectory": False, "geometry": False, "rover_state": False, "nonempty_tracked": False},
    "bag_created": False,
}
summary = {
    "1_build_test": "PASS",
    "2_tracked_objects_runtime": "FAIL",
    "3_first_nonempty_tracked_sim_t": None,
    "4_geometry_at_first_tracked": False,
    "5_rover_state_at_first_tracked": False,
    "6_canonical_bag_created": False,
    "7_static_prediction": "SKIP",
    "8_dynamic_prediction": "SKIP",
    "9_collision_steps": None,
    "10_rollover": None,
    "11_once_per_trajectory": None,
    "12_blocker": blocker,
    "log_dir": "${LOG_DIR}",
}
Path("${LOG_DIR}/report_upstream.json").write_text(json.dumps(report, indent=2))
Path("${LOG_DIR}/summary.json").write_text(json.dumps(summary, indent=2))
PY
  exit 2
fi
log "ZED started"

# --- Condition-driven monitor (hard 180s upstream) ---
UPSTREAM_EXIT=0
python3 "${MONITOR}" --log-dir "${LOG_DIR}" --bag-dir "${BAG}" --phase upstream \
  --gate-limit-s "${GATE_LIMIT}" \
  | tee "${LOG_DIR}/report_upstream_stdout.json" || UPSTREAM_EXIT=$?

# Stop upstream stack before bag replays
cleanup
PIDS=()
trap - EXIT

log "upstream phase exit=${UPSTREAM_EXIT}"

# --- Bag replay: static then dynamic (fast, no ZED/YOLO) ---
REPLAY_STATIC_EXIT=0
REPLAY_DYNAMIC_EXIT=0

if [[ -d "${BAG}" ]] && [[ -f "${BAG}/metadata.yaml" ]]; then
  log "canonical bag found: ${BAG}"

  ros2 run prediction_ros prediction_node --ros-args \
    -p use_sim_time:=true -p config_path:="${CONFIG}" -p prediction_profile:=static \
    >"${LOG_DIR}/replay_static_prediction.log" 2>&1 &
  PIDS+=($!)
  sleep 2
  python3 "${MONITOR}" --log-dir "${LOG_DIR}" --bag-dir "${BAG}" --phase replay-static \
    --replay-duration-s 30 --target-predict 10 \
    | tee "${LOG_DIR}/report_replay_static_stdout.json" || REPLAY_STATIC_EXIT=$?
  kill -INT "${PIDS[-1]}" 2>/dev/null || true
  wait "${PIDS[-1]}" 2>/dev/null || true
  PIDS=("${PIDS[@]:0:$((${#PIDS[@]}-1))}")

  ros2 run prediction_ros prediction_node --ros-args \
    -p use_sim_time:=true -p config_path:="${CONFIG}" -p prediction_profile:=dynamic \
    >"${LOG_DIR}/replay_dynamic_prediction.log" 2>&1 &
  PIDS+=($!)
  sleep 2
  python3 "${MONITOR}" --log-dir "${LOG_DIR}" --bag-dir "${BAG}" --phase replay-dynamic \
    --replay-duration-s 30 --target-predict 10 \
    | tee "${LOG_DIR}/report_replay_dynamic_stdout.json" || REPLAY_DYNAMIC_EXIT=$?
  kill -INT "${PIDS[-1]}" 2>/dev/null || true
  wait "${PIDS[-1]}" 2>/dev/null || true
  PIDS=("${PIDS[@]:0:$((${#PIDS[@]}-1))}")
else
  log "SKIP bag replay: canonical bag not created"
fi

cleanup 2>/dev/null || true

LOG_DIR="${LOG_DIR}" BAG="${BAG}" python3 - <<'PY' | tee "${LOG_DIR}/summary_stdout.json"
import json
import os
from pathlib import Path

log = Path(os.environ["LOG_DIR"])
bag = os.environ["BAG"]
def load(name):
    p = log / name
    return json.loads(p.read_text()) if p.exists() else {}

upstream = load("report_upstream.json")
static = load("report_replay-static.json")
dynamic = load("report_replay-dynamic.json")

def pass_fail(report, key="success"):
    if not report:
        return "SKIP"
    return "PASS" if report.get(key) else "FAIL"

readiness = upstream.get("readiness", {})
collection = upstream.get("collection", {})
once = upstream.get("once_per_trajectory", {})
static_once = static.get("once_per_trajectory", {})
dynamic_once = dynamic.get("once_per_trajectory", {})

summary = {
    "1_build_test": "PASS",
    "2_tracked_objects_runtime": (
        "PASS" if readiness.get("nonempty_tracked") and collection.get("nonempty_tracked_messages", 0) > 0
        else "FAIL"
    ),
    "3_first_nonempty_tracked_sim_t": upstream.get("readiness_sim_t", {}).get("nonempty_tracked"),
    "4_geometry_at_first_tracked": upstream.get("geometry_at_first_tracked"),
    "5_rover_state_at_first_tracked": upstream.get("rover_state_at_first_tracked"),
    "6_canonical_bag_created": upstream.get("bag_created", False),
    "7_static_prediction": pass_fail(static),
    "8_dynamic_prediction": pass_fail(dynamic),
    "9_collision_steps": {
        "upstream": upstream.get("prediction", {}).get("collision_steps_total"),
        "static_replay": static.get("prediction", {}).get("collision_steps_total"),
        "dynamic_replay": dynamic.get("prediction", {}).get("collision_steps_total"),
    },
    "10_rollover": {
        "upstream_stability_valid": upstream.get("stability_valid_seen"),
        "upstream_zmp_valid": upstream.get("zmp_valid_seen"),
        "static_stability_valid": static.get("stability_valid_seen"),
        "static_zmp_valid": static.get("zmp_valid_seen"),
        "dynamic_stability_valid": dynamic.get("stability_valid_seen"),
        "dynamic_zmp_valid": dynamic.get("zmp_valid_seen"),
        "upstream_rollover_steps_total": upstream.get("prediction", {}).get("rollover_steps_total"),
        "static_rollover_steps_total": static.get("prediction", {}).get("rollover_steps_total"),
        "dynamic_rollover_steps_total": dynamic.get("prediction", {}).get("rollover_steps_total"),
    },
    "11_once_per_trajectory": {
        "upstream_duplicate_prediction_source_ids": once.get("duplicate_prediction_source_ids", []),
        "static_duplicate_prediction_source_ids": static_once.get("duplicate_prediction_source_ids", []),
        "dynamic_duplicate_prediction_source_ids": dynamic_once.get("duplicate_prediction_source_ids", []),
    },
    "12_blocker": upstream.get("blocker") or static.get("blocker") or dynamic.get("blocker") or (
        "canonical bag not created" if not upstream.get("bag_created", False) else None
    ),
    "bag_path": bag,
    "log_dir": str(log),
    "upstream_elapsed_wall_s": upstream.get("elapsed_wall_s"),
    "upstream_collection": collection,
}
Path(log / "summary.json").write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
PY

log "DONE ${LOG_DIR}"
exit "${UPSTREAM_EXIT}"
