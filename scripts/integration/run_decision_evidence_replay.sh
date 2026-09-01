#!/usr/bin/env bash
# Headless Prediction + Decision evidence against the dynamic replay fixture.
# Default ROS_DOMAIN_ID=49. Starts Prediction + decision_ros BEFORE bag play.
set -eo pipefail
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_integration_env.sh"
set +u
source /opt/ros/humble/setup.bash
source "${PREDICTION_SRC_ROOT}/ros2/install_humble/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-49}"
export PYTHONPATH="${PREDICTION_SRC_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
BAG="${1:-${PREDICTION_BAGS}/session_0924_dynamic_prediction_inputs}"
CONFIG="${PREDICTION_SRC_ROOT}/config/rover.mock.yaml"
LOG_DIR="${LOG_DIR:-${PREDICTION_LOGS}/decision_evidence_replay_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${LOG_DIR}"

PRED_PID=""
DEC_PID=""
BAG_PID=""
MON_PID=""

cleanup() {
  for pid in "${BAG_PID}" "${MON_PID}" "${DEC_PID}" "${PRED_PID}"; do
    [[ -z "${pid}" ]] && continue
    kill -INT "${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${BAG_PID}" "${MON_PID}" "${DEC_PID}" "${PRED_PID}"; do
    [[ -z "${pid}" ]] && continue
    kill -TERM "${pid}" 2>/dev/null || true
    kill -KILL "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} BAG=${BAG} LOG_DIR=${LOG_DIR}" | tee "${LOG_DIR}/run.log"

{
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  ros2 node list 2>&1 || true
  ros2 topic list 2>&1 || true
} | tee "${LOG_DIR}/domain_sanity.txt"

if ros2 node list 2>/dev/null | grep -qE 'prediction_node|decision_evidence'; then
  echo "FAIL: nodes already present on domain ${ROS_DOMAIN_ID}" >&2
  exit 2
fi

ros2 run prediction_ros prediction_node --ros-args \
  -p use_sim_time:=true -p config_path:="${CONFIG}" -p prediction_profile:=dynamic \
  >"${LOG_DIR}/prediction.log" 2>&1 &
PRED_PID=$!
sleep 2

ros2 launch decision_ros decision_evidence.launch.py use_sim_time:=true \
  >"${LOG_DIR}/decision.log" 2>&1 &
DEC_PID=$!
sleep 2

ros2 node list | tee "${LOG_DIR}/nodes_pre_play.txt"
ros2 topic list | tee "${LOG_DIR}/topics_pre_play.txt"

python3 - <<PY >"${LOG_DIR}/monitor_stdout.txt" 2>&1 &
import json, time
from pathlib import Path as FsPath
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from safety_perception_msgs.msg import DecisionEvidence, PredictionOutput, Trajectory

LOG = FsPath("${LOG_DIR}")

class Mon(Node):
    def __init__(self):
        super().__init__("decision_replay_mon")
        self.predict_ids = []
        self.evidence = []
        self.stale_windows = []
        self._active_traj = None
        self._had_current = False
        self.create_subscription(Trajectory, "/trajectory", self._t, 10)
        self.create_subscription(PredictionOutput, "/predict_output", self._p, 10)
        self.create_subscription(DecisionEvidence, "/decision/evidence", self._e, 10)

    def _t(self, msg):
        new_id = int(msg.trajectory_id)
        if self._active_traj is not None and new_id != self._active_traj and self._had_current:
            self.stale_windows.append(
                {"active_traj": new_id, "note": "trajectory_advanced"}
            )
        self._active_traj = new_id
        self._had_current = False

    def _p(self, msg):
        self.predict_ids.append(int(msg.source_trajectory_id))

    def _e(self, msg):
        row = {
            "source_trajectory_id": msg.source_trajectory_id,
            "evidence_state": int(msg.evidence_state),
            "collision_candidates_present": bool(msg.collision_candidates_present),
            "rollover_baseline_present": bool(msg.rollover_baseline_present),
            "dynamic_stability_moment_valid": bool(msg.dynamic_stability_moment_valid),
            "zmp_valid": bool(msg.zmp_valid),
        }
        self.evidence.append(row)
        if int(msg.evidence_state) == 2:
            self._had_current = True
        if int(msg.evidence_state) == 1 and self.stale_windows:
            self.stale_windows[-1]["stale_evidence"] = row

rclpy.init()
node = Mon()
node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
start = time.time()
while time.time() - start < 25.0 and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.05)

by_id = {}
for row in node.evidence:
    tid = row["source_trajectory_id"]
    if int(row["evidence_state"]) == 2:
        by_id[tid] = row

report = {
    "predict_ids": node.predict_ids,
    "current_evidence_by_trajectory_id": by_id,
    "stale_windows": node.stale_windows,
    "evidence_sample_tail": node.evidence[-20:],
}
(LOG / "decision_replay_report.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
node.destroy_node()
rclpy.shutdown()
PY
MON_PID=$!
sleep 1

ros2 bag play "${BAG}" --clock >"${LOG_DIR}/bag_play.log" 2>&1 &
BAG_PID=$!
wait "${BAG_PID}" 2>/dev/null || true
BAG_PID=""
wait "${MON_PID}" 2>/dev/null || true
MON_PID=""

echo "=== report ===" | tee -a "${LOG_DIR}/run.log"
cat "${LOG_DIR}/decision_replay_report.json" | tee -a "${LOG_DIR}/run.log"
