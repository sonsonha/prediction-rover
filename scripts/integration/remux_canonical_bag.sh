#!/usr/bin/env bash
# Remux canonical Prediction INPUT bag using rosbag2_py (preserves timestamps).
set -eo pipefail
# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_integration_env.sh"
set +u
source /opt/ros/humble/setup.bash
set -u

SRC="${1:-${PREDICTION_BAGS}/session_0924_pipe_prediction_inputs.with_predict_output.bak}"
DST="${2:-${PREDICTION_BAGS}/session_0924_pipe_prediction_inputs}"

if [[ ! -f "${SRC}/metadata.yaml" ]]; then
  echo "FAIL: source bag missing: ${SRC}/metadata.yaml" >&2
  exit 1
fi

python3 "${INTEGRATION_DIR}/remux_canonical_bag.py" --src "${SRC}" --dst "${DST}.tmp"
rm -rf "${DST}"
mv "${DST}.tmp" "${DST}"

echo "Canonical bag topics:"
ros2 bag info "${DST}"
