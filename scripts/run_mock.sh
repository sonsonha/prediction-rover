#!/usr/bin/env bash
# Launch mock_upstream_node. Works from any cwd.
# Modes: static | dynamic | dynamic_zero | dynamic_wrench
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
# shellcheck source=ros_env.sh
source "${SCRIPT_DIR}/ros_env.sh"

echo "→ mock_upstream_node  demo_mode=${MODE}"
exec ros2 run prediction_ros mock_upstream_node \
  --ros-args -p "demo_mode:=${MODE}"
