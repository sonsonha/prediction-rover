#!/usr/bin/env bash
# Headless Prediction + Decision V0 evidence + V1 STOP/GO policy + viz replay.
# Default ROS_DOMAIN_ID=50. Start all nodes BEFORE bag play.
set -eo pipefail
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_integration_env.sh"
set +u
source /opt/ros/humble/setup.bash
source "${PREDICTION_SRC_ROOT}/ros2/install_humble/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-50}"
export PYTHONPATH="${PREDICTION_SRC_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
BAG="${1:-${PREDICTION_BAGS}/session_0924_dynamic_prediction_inputs}"
CONFIG="${PREDICTION_SRC_ROOT}/config/rover.mock.yaml"
LOG_DIR="${LOG_DIR:-${PREDICTION_LOGS}/decision_stop_go_replay_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${LOG_DIR}"

PRED_PID=""
EVID_PID=""
POL_PID=""
VIZ_PID=""
BAG_PID=""
MON_PID=""

cleanup() {
  for pid in "${BAG_PID}" "${MON_PID}" "${VIZ_PID}" "${POL_PID}" "${EVID_PID}" "${PRED_PID}"; do
    [[ -z "${pid}" ]] && continue
    kill -INT "${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${BAG_PID}" "${MON_PID}" "${VIZ_PID}" "${POL_PID}" "${EVID_PID}" "${PRED_PID}"; do
    [[ -z "${pid}" ]] && continue
    kill -TERM "${pid}" 2>/dev/null || true
    kill -KILL "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} BAG=${BAG} LOG_DIR=${LOG_DIR}" | tee "${LOG_DIR}/run.log"

if ros2 node list 2>/dev/null | grep -qE 'prediction_node|decision_evidence|decision_policy|prediction_visualization'; then
  echo "FAIL: nodes already present on domain ${ROS_DOMAIN_ID}" >&2
  exit 2
fi

ros2 run prediction_ros prediction_node --ros-args \
  -p use_sim_time:=true -p config_path:="${CONFIG}" -p prediction_profile:=dynamic \
  >"${LOG_DIR}/prediction.log" 2>&1 &
PRED_PID=$!
sleep 2

ros2 launch decision_ros decision_evidence.launch.py use_sim_time:=true \
  >"${LOG_DIR}/decision_evidence.log" 2>&1 &
EVID_PID=$!
sleep 1

ros2 launch decision_ros decision_policy.launch.py use_sim_time:=true \
  >"${LOG_DIR}/decision_policy.log" 2>&1 &
POL_PID=$!
sleep 1

ros2 launch prediction_visualization prediction_visualization.launch.py \
  use_sim_time:=true \
  >"${LOG_DIR}/viz.log" 2>&1 &
VIZ_PID=$!
sleep 2

python3 - <<PY >"${LOG_DIR}/monitor_stdout.txt" 2>&1 &
import json, time
from pathlib import Path as FsPath
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from safety_perception_msgs.msg import DecisionOutput, Trajectory

LOG = FsPath("${LOG_DIR}")
REASON = {
    0: "CURRENT_CLEAR",
    1: "NO_CURRENT_PREDICTION",
    2: "PREDICTION_STALE",
    3: "COLLISION_CANDIDATE",
    4: "ROLLOVER_EVIDENCE_INVALID",
    5: "ROLLOVER_POLICY_TRIGGERED",
}

class Mon(Node):
    def __init__(self):
        super().__init__("decision_stop_go_mon")
        self.transitions = []
        self.by_traj = {}
        self._last_key = None
        self.create_subscription(Trajectory, "/trajectory", self._t, 10)
        self.create_subscription(DecisionOutput, "/decision", self._d, 10)

    def _t(self, msg):
        self._active = int(msg.trajectory_id)

    def _d(self, msg):
        tid = str(msg.source_trajectory_id)
        decision = "GO" if int(msg.decision) == 0 else "STOP"
        reason = REASON.get(int(msg.reason), str(int(msg.reason)))
        key = (tid, decision, reason)
        if key == self._last_key:
            return
        self._last_key = key
        row = {
            "source_trajectory_id": tid,
            "decision": decision,
            "reason": reason,
            "prototype_policy": bool(msg.prototype_policy),
        }
        self.transitions.append(row)
        self.by_traj.setdefault(tid, []).append(row)

rclpy.init()
node = Mon()
node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
start = time.time()
while time.time() - start < 25.0 and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.05)

go_ids = sorted({r["source_trajectory_id"] for r in node.transitions if r["decision"] == "GO"})
stop_by_id = {}
for tid, rows in node.by_traj.items():
    stops = [r for r in rows if r["decision"] == "STOP"]
    if stops:
        stop_by_id[tid] = stops[-1]["reason"]

report = {
    "transitions": node.transitions,
    "go_trajectory_ids": go_ids,
    "stop_reason_by_trajectory_id": stop_by_id,
}
(LOG / "decision_stop_go_report.json").write_text(json.dumps(report, indent=2))
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
cat "${LOG_DIR}/decision_stop_go_report.json" | tee -a "${LOG_DIR}/run.log"
