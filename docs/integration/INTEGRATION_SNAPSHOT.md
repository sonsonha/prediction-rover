# Integration Snapshot

Reproducible source pins for the Landfill Rover → Prediction real pipeline.

## Snapshot tag

```text
integration-20260831
```

Create after checking out the documentation commit on
`integration/humble-real-pipeline`:

```bash
git tag -a integration-20260831 -m "Landfill Rover real pipeline integration snapshot (2026-08-31)"
git push origin integration-20260831
```

## Repository pins

The `sonsonha` repositories below are **integration forks/snapshots**. They are
**not necessarily the canonical team upstreams** — use them to reproduce this
integration exactly.

| Role | Integration fork | Branch | SHA |
|------|------------------|--------|-----|
| Prediction + bridge + tooling | [sonsonha/prediction-rover](https://github.com/sonsonha/prediction-rover) | `integration/humble-real-pipeline` | checkout branch; use tag `integration-20260831` when published |
| Segmentation + terrain | [sonsonha/lr-ros2](https://github.com/sonsonha/lr-ros2) | `integration/real-terrain-segmentation` | `cb580c0` |
| ZED / MAVLink / pointcloud | [sonsonha/ROS2_rover_trajectory](https://github.com/sonsonha/ROS2_rover_trajectory) | `main` | `e345768` |

### Original upstream references (not used for this snapshot)

| Upstream | URL |
|----------|-----|
| lr-ros2 (original) | https://github.com/hachitrung024/lr-ros2 |
| ROS2_rover_trajectory (original) | https://github.com/haitrung3101nht/ROS2_rover_trajectory |

## Validated session

| Field | Value |
|-------|-------|
| Session ID | `session_20260710_0924` |
| SVO | `raw/session_20260710_0924/zed/zed_20260710_092420_0001.svo2` |
| MAVLink CSV | `raw/session_20260710_0924/mavlink/extracted/session_20260710_0924_mavlink/` |
| SVO frames | 28154 @ 15 fps |
| Pipe-rich sim window | ~`1783700698` – `1783700725` |

## Runtime

| Setting | Value |
|---------|-------|
| `ROS_DOMAIN_ID` | `42` |
| Docker image | `prediction-humble-dev:latest` (built from this repo) |
| Canonical frame | `map` |
| Sim time source | ZED `/clock` (`publish_svo_clock:=true`) |
| Canonical bag | `prediction/bags/session_0924_pipe_prediction_inputs` |

## Gate reference

Primary upstream PASS:
`prediction/logs/condition_gate_20260831_035052/report_upstream.json`

## Local-only (not in Git)

- `/data/rover_workspace/integration_ws/` — colcon overlay (`build/`, `install/`, `log/`)
- `/data/rover_workspace/prediction/logs/` — generated gate/demo output
- `/data/rover_workspace/raw/` — session recordings (SVO2, MAVLink)
- `/data/rover_workspace/prediction/bags/` — rosbag artifacts
- `/data/rover_workspace/prediction/.cache/` — ZED build caches
