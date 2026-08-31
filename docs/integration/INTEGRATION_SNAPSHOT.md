# Integration Snapshot

Validated session and repository SHAs for reproducing the real pipeline integration.

## Validated session

| Field | Value |
|-------|-------|
| Session ID | `session_20260710_0924` |
| SVO | `raw/session_20260710_0924/zed/zed_20260710_092420_0001.svo2` |
| MAVLink CSV | `raw/session_20260710_0924/mavlink/extracted/session_20260710_0924_mavlink/` |
| SVO frames | 28154 @ 15 fps |
| Pipe-rich sim window | ~`1783700698` – `1783700725` |

## Upstream repositories

| Repo | Branch | SHA |
|------|--------|-----|
| `lr-ros2` | `integration/real-terrain-segmentation` | `cb580c0` |
| `ROS2_rover_trajectory` | `main` | `e345768` |
| `prediction-rover` (this repo) | `integration/humble-real-pipeline` | `1682efd` |

## Runtime

| Setting | Value |
|---------|-------|
| `ROS_DOMAIN_ID` | `42` |
| Docker image | `prediction-humble-dev:latest` |
| Canonical bag | `prediction/bags/session_0924_pipe_prediction_inputs` |

## Gate reference

Primary upstream PASS: `prediction/logs/condition_gate_20260831_035052/report_upstream.json`

## Local-only (not in this repo)

- `/data/rover_workspace/integration_ws/` colcon workspace
- `/data/rover_workspace/prediction/logs/` generated run output
- `/data/rover_workspace/raw/` session recordings
- `/data/rover_workspace/prediction/bags/` rosbag artifacts
- `/data/rover_workspace/prediction/.cache/` ZED build caches
