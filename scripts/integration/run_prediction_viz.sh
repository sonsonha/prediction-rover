#!/usr/bin/env bash
# Headless Prediction + visualization against the dynamic replay fixture.
# Default ROS_DOMAIN_ID=48. Starts Prediction + viz BEFORE bag play.
set -eo pipefail
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_integration_env.sh"
set +u
source /opt/ros/humble/setup.bash
source "${PREDICTION_SRC_ROOT}/ros2/install_humble/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-48}"
export PYTHONPATH="${PREDICTION_SRC_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
BAG="${1:-${PREDICTION_BAGS}/session_0924_dynamic_prediction_inputs}"
CONFIG="${PREDICTION_SRC_ROOT}/config/rover.mock.yaml"
LOG_DIR="${LOG_DIR:-${PREDICTION_LOGS}/prediction_viz_replay_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${LOG_DIR}"

PRED_PID=""
VIZ_PID=""
BAG_PID=""
MON_PID=""

cleanup() {
  for pid in "${BAG_PID}" "${MON_PID}" "${VIZ_PID}" "${PRED_PID}"; do
    [[ -z "${pid}" ]] && continue
    kill -INT "${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${BAG_PID}" "${MON_PID}" "${VIZ_PID}" "${PRED_PID}"; do
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

if ros2 node list 2>/dev/null | grep -qE 'prediction_node|prediction_visualization'; then
  echo "FAIL: nodes already present on domain ${ROS_DOMAIN_ID}" >&2
  exit 2
fi

ros2 run prediction_ros prediction_node --ros-args \
  -p use_sim_time:=true -p config_path:="${CONFIG}" -p prediction_profile:=dynamic \
  >"${LOG_DIR}/prediction.log" 2>&1 &
PRED_PID=$!
sleep 2

ros2 launch prediction_visualization prediction_visualization.launch.py \
  use_sim_time:=true \
  >"${LOG_DIR}/viz.log" 2>&1 &
VIZ_PID=$!
sleep 2

ros2 node list | tee "${LOG_DIR}/nodes_pre_play.txt"
ros2 topic list | tee "${LOG_DIR}/topics_pre_play.txt"

python3 - <<PY >"${LOG_DIR}/monitor_stdout.txt" 2>&1 &
import json, time
from pathlib import Path as FsPath
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from safety_perception_msgs.msg import PredictionOutput
from nav_msgs.msg import Path as NavPath
from visualization_msgs.msg import MarkerArray

LOG = FsPath("${LOG_DIR}")

class Mon(Node):
    def __init__(self):
        super().__init__("viz_replay_mon")
        self.predict_ids = []
        self.counts = {t: 0 for t in [
            "/prediction_viz/trajectory",
            "/prediction_viz/objects",
            "/prediction_viz/terrain",
            "/prediction_viz/rover",
            "/prediction_viz/collision",
            "/prediction_viz/rollover",
            "/prediction_viz/zmp",
            "/prediction_viz/status",
        ]}
        self.create_subscription(PredictionOutput, "/predict_output", self._p, 10)
        self.create_subscription(
            NavPath, "/prediction_viz/trajectory",
            lambda m: self._c("/prediction_viz/trajectory"), 10,
        )
        for t in list(self.counts)[1:]:
            self.create_subscription(
                MarkerArray, t, lambda m, tt=t: self._c(tt), 10,
            )
    def _p(self, msg):
        self.predict_ids.append(int(msg.source_trajectory_id))
    def _c(self, t):
        self.counts[t] += 1

rclpy.init()
node = Mon()
node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
start = time.time()
while time.time() - start < 25.0 and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.05)
report = {"predict_ids": node.predict_ids, "viz_counts": node.counts}
(LOG / "viz_replay_report.json").write_text(json.dumps(report, indent=2))
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
cat "${LOG_DIR}/viz_replay_report.json" | tee -a "${LOG_DIR}/run.log"
