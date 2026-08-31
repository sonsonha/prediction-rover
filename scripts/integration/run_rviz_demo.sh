#!/usr/bin/env bash
# Persistent live demo for RViz (session_0924). Runs laps until Ctrl+C.
#
# Each lap starts the full processing stack. On SVO EOF the stack is torn down
# and restarted so sim-time /clock reset does not leave stale downstream state.
#
# Optional test (near-EOF, ~45 s to EOF on lap 1 only):
#   RVIZ_DEMO_TEST_NEAR_EOF=1 bash run_rviz_demo.sh
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
LOG_DIR="${PREDICTION_LOGS}/rviz_demo_$(date +%Y%m%d_%H%M%S)"

# session_0924 has 28154 frames @ 15 fps; 27479 ≈ 45 s before EOF (test only)
RVIZ_DEMO_NEAR_EOF_FRAME="${RVIZ_DEMO_NEAR_EOF_FRAME:-27479}"

LAP_NUM=0
LAP_PIDS=()
EOF_MONITOR_PID=""
READINESS_PID=""
USER_STOP=0
LAP_ACTIVE=0

mkdir -p "${LOG_DIR}"

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "${LOG_DIR}/run.log"; }

kill_pid_tree() {
  local pid="$1"
  local sig="$2"
  [[ -z "${pid}" ]] && return 0
  if ! kill -0 "${pid}" 2>/dev/null; then
    return 0
  fi
  local child
  for child in $(pgrep -P "${pid}" 2>/dev/null || true); do
    kill_pid_tree "${child}" "${sig}"
  done
  kill "-${sig}" "${pid}" 2>/dev/null || true
}

wait_pid_tree() {
  local pid="$1"
  local timeout_s="$2"
  local elapsed=0
  while kill -0 "${pid}" 2>/dev/null; do
    if (( elapsed >= timeout_s )); then
      return 1
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 0
}

stop_background_job() {
  local pid="$1"
  [[ -z "${pid}" ]] && return 0
  kill_pid_tree "${pid}" INT
  sleep 1
  kill_pid_tree "${pid}" KILL
  wait "${pid}" 2>/dev/null || true
}

lap_svo_start_frame() {
  local lap="$1"
  if [[ "${RVIZ_DEMO_TEST_NEAR_EOF:-0}" == "1" && "${lap}" -eq 1 ]]; then
    echo "${RVIZ_DEMO_NEAR_EOF_FRAME}"
  else
    echo "${RVIZ_DEMO_SVO_START_FRAME:-0}"
  fi
}

rover_param_overrides() {
  local start_frame="$1"
  local overrides='depth.depth_mode:=PERFORMANCE;depth.point_cloud_freq:=5.0'
  if [[ "${start_frame}" != "0" ]]; then
    overrides+=";svo.play_from_frame:=${start_frame}"
  fi
  echo "${overrides}"
}

start_eof_monitor() {
  local rover_log="$1"
  local eof_flag="$2"
  rm -f "${eof_flag}"
  (
    # Wait until ZED is running before watching for EOF (avoid false positives).
    for _ in $(seq 1 180); do
      if grep -q "=== zed started ===" "${rover_log}" 2>/dev/null; then
        break
      fi
      sleep 1
    done
    tail -n 0 -F "${rover_log}" 2>/dev/null | while IFS= read -r line; do
      if [[ "${line}" == *"END OF SVO FILE REACHED"* ]] \
        || [[ "${line}" == *"SVO reached the end."* ]]; then
        : >"${eof_flag}"
        break
      fi
    done
  ) &
  EOF_MONITOR_PID=$!
}

start_readiness_monitor() {
  local lap="$1"
  local rover_log="$2"
  (
    for _ in $(seq 1 180); do
      if grep -q "=== zed started ===" "${rover_log}" 2>/dev/null; then
        echo "[$(date +%H:%M:%S)] LAP ${lap} ZED started" >>"${LOG_DIR}/run.log"
        break
      fi
      sleep 1
    done

    if out=$(timeout 120 ros2 topic echo /clock rosgraph_msgs/msg/Clock --once 2>/dev/null); then
      sec=$(echo "${out}" | awk '/sec:/{print $2; exit}')
      nanosec=$(echo "${out}" | awk '/nanosec:/{print $2; exit}')
      echo "[$(date +%H:%M:%S)] LAP ${lap} first /clock sec=${sec} nanosec=${nanosec}" >>"${LOG_DIR}/run.log"
    else
      echo "[$(date +%H:%M:%S)] LAP ${lap} first /clock: not observed within 120s" >>"${LOG_DIR}/run.log"
    fi

    for spec in \
      "/trajectory:safety_perception_msgs/msg/Trajectory" \
      "/geometry:safety_perception_msgs/msg/GeometryArray" \
      "/rover/state:safety_perception_msgs/msg/RoverState"; do
      topic="${spec%%:*}"
      msg="${spec#*:}"
      if out=$(timeout 120 ros2 topic echo "${topic}" "${msg}" --once 2>/dev/null); then
        sec=$(echo "${out}" | awk '/sec:/{print $2; exit}')
        echo "[$(date +%H:%M:%S)] LAP ${lap} first ${topic} stamp.sec=${sec}" >>"${LOG_DIR}/run.log"
      else
        echo "[$(date +%H:%M:%S)] LAP ${lap} first ${topic}: not observed within 120s" >>"${LOG_DIR}/run.log"
      fi
    done

    if out=$(timeout 120 ros2 topic echo /tracked_objects safety_perception_msgs/msg/TrackedObjectArray --once 2>/dev/null); then
      count=$(echo "${out}" | awk '/^objects:/{c=0} /^- /{c++} END{print c+0}')
      if [[ "${count}" -gt 0 ]]; then
        echo "[$(date +%H:%M:%S)] LAP ${lap} first non-empty /tracked_objects (${count} objects)" >>"${LOG_DIR}/run.log"
      else
        echo "[$(date +%H:%M:%S)] LAP ${lap} first /tracked_objects empty (no objects in window)" >>"${LOG_DIR}/run.log"
      fi
    else
      echo "[$(date +%H:%M:%S)] LAP ${lap} first /tracked_objects: not observed within 120s" >>"${LOG_DIR}/run.log"
    fi

    if out=$(timeout 180 ros2 topic echo /predict_output safety_perception_msgs/msg/PredictionOutput --once 2>/dev/null); then
      sec=$(echo "${out}" | awk '/sec:/{print $2; exit}')
      echo "[$(date +%H:%M:%S)] LAP ${lap} first /predict_output stamp.sec=${sec}" >>"${LOG_DIR}/run.log"
    else
      echo "[$(date +%H:%M:%S)] LAP ${lap} first /predict_output: not observed within 180s" >>"${LOG_DIR}/run.log"
    fi
  ) &
  READINESS_PID=$!
}

start_lap() {
  LAP_NUM=$((LAP_NUM + 1))
  LAP_PIDS=()
  LAP_ACTIVE=1

  local lap_dir="${LOG_DIR}/lap_${LAP_NUM}"
  mkdir -p "${lap_dir}"
  local start_frame
  start_frame=$(lap_svo_start_frame "${LAP_NUM}")
  local overrides
  overrides=$(rover_param_overrides "${start_frame}")
  local eof_flag="${lap_dir}/.eof"

  log "LAP ${LAP_NUM} START (svo.play_from_frame=${start_frame})"

  ros2 launch lr_bringup rover.launch.py \
    camera_model:=zed2i svo_path:="${SVO}" gps_path:="${GPS}" attitude_path:="${ATT}" \
    publish_svo_clock:=true svo_fps:=15.0 future_path_frames:=200 use_rviz:=false \
    accumulate_cloud:=false map_frame_step:=5 \
    param_overrides:="${overrides}" \
    >"${lap_dir}/rover.log" 2>&1 &
  LAP_PIDS+=($!)

  ros2 launch lr_segmentation segmentation.launch.py \
    use_sim_time:=true model_path:="${MODEL}" model_device:=0 \
    >"${lap_dir}/segmentation.log" 2>&1 &
  LAP_PIDS+=($!)

  ros2 launch lr_terrain_geometry terrain_geometry.launch.py \
    use_sim_time:=true point_cloud_topic:=/lr/point_cloud/cloud_in_map \
    sensor_frame:=zed_camera_link object_filter_enabled:=true \
    >"${lap_dir}/terrain.log" 2>&1 &
  LAP_PIDS+=($!)

  ros2 run lr_prediction_bridge trajectory_adapter_node --ros-args -p use_sim_time:=true \
    >"${lap_dir}/bridge_traj.log" 2>&1 &
  LAP_PIDS+=($!)

  ros2 run lr_prediction_bridge geometry_adapter_node --ros-args \
    -p use_sim_time:=true -p allow_flat_fallback:=false \
    >"${lap_dir}/bridge_geom.log" 2>&1 &
  LAP_PIDS+=($!)

  ros2 run lr_prediction_bridge rover_state_adapter_node --ros-args -p use_sim_time:=true \
    >"${lap_dir}/bridge_rover.log" 2>&1 &
  LAP_PIDS+=($!)

  ros2 run lr_prediction_bridge tracked_objects_adapter_node --ros-args -p use_sim_time:=true \
    >"${lap_dir}/bridge_tracked.log" 2>&1 &
  LAP_PIDS+=($!)

  ros2 run prediction_ros prediction_node --ros-args \
    -p use_sim_time:=true -p config_path:="${CONFIG}" -p prediction_profile:=dynamic \
    >"${lap_dir}/prediction_dynamic.log" 2>&1 &
  LAP_PIDS+=($!)

  start_eof_monitor "${lap_dir}/rover.log" "${eof_flag}"
  start_readiness_monitor "${LAP_NUM}" "${lap_dir}/rover.log"
}

stop_lap() {
  [[ "${LAP_ACTIVE}" -eq 0 ]] && return 0

  log "=== SVO EOF: restarting complete demo stack ==="
  log "LAP ${LAP_NUM} EOF"

  stop_background_job "${EOF_MONITOR_PID}"
  EOF_MONITOR_PID=""
  stop_background_job "${READINESS_PID}"
  READINESS_PID=""

  local pid
  for pid in "${LAP_PIDS[@]}"; do
    kill_pid_tree "${pid}" INT
  done

  sleep 2

  for pid in "${LAP_PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      log "lap child ${pid} still alive after SIGINT; sending SIGTERM"
      kill_pid_tree "${pid}" TERM
    fi
  done

  sleep 2

  for pid in "${LAP_PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      log "lap child ${pid} still alive after SIGTERM; sending SIGKILL"
      kill_pid_tree "${pid}" KILL
    fi
  done

  for pid in "${LAP_PIDS[@]}"; do
    wait_pid_tree "${pid}" 5 || log "WARN: lap child ${pid} did not exit cleanly"
  done

  local stale
  stale=$(ros2 node list 2>/dev/null | grep -E \
    'mavlink_csv_pose|pointcloud_transform|trajectory_adapter|geometry_adapter|rover_state_adapter|tracked_objects_adapter|prediction_node|segmentation|terrain_geometry|zed\.zed_node' \
    || true)
  if [[ -n "${stale}" ]]; then
    log "WARN: stale ROS nodes after lap cleanup:${stale}"
  fi

  LAP_PIDS=()
  LAP_ACTIVE=0
  log "LAP ${LAP_NUM} CLEANUP COMPLETE"
}

wait_for_eof_or_user_stop() {
  local lap_dir="${LOG_DIR}/lap_${LAP_NUM}"
  local eof_flag="${lap_dir}/.eof"
  while [[ "${USER_STOP}" -eq 0 ]]; do
    if [[ -f "${eof_flag}" ]]; then
      return 0
    fi
    sleep 5
  done
  return 0
}

on_signal() {
  USER_STOP=1
  log "user stop requested (Ctrl+C)"
}

cleanup() {
  USER_STOP=1
  if [[ "${LAP_ACTIVE}" -eq 1 ]]; then
    log "shutting down active lap..."
    stop_lap
  fi
  stop_background_job "${EOF_MONITOR_PID}"
  stop_background_job "${READINESS_PID}"
}
trap on_signal INT TERM
trap cleanup EXIT

log "LOG_DIR=${LOG_DIR} ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
if [[ "${RVIZ_DEMO_TEST_NEAR_EOF:-0}" == "1" ]]; then
  log "TEST MODE: lap 1 uses svo.play_from_frame=${RVIZ_DEMO_NEAR_EOF_FRAME} (~45s to EOF)"
else
  log "NORMAL MODE: svo.play_from_frame=0 for every lap"
fi
log "Starting persistent demo laps (Ctrl+C to stop)..."
log "RViz topics (Fixed Frame: map): /lr/point_cloud/cloud_in_map /lr/mavlink/trajectory_future"
log "  /terrain_geometry/markers /terrain_geometry/object_box_markers /segmentation/overlay TF"

while [[ "${USER_STOP}" -eq 0 ]]; do
  start_lap
  wait_for_eof_or_user_stop
  if [[ "${USER_STOP}" -eq 1 ]]; then
    stop_lap
    break
  fi
  stop_lap
  sleep 2
done

log "demo exited"
