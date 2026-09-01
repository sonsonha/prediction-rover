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
| `ROS_DOMAIN_ID` | `42` (demo/gate); isolated validation may use other domains |
| Docker image | `prediction-humble-dev:latest` (built from this repo) |
| Canonical frame | `map` |
| Sim time source | ZED `/clock` (`publish_svo_clock:=true`) |
| Dynamic replay fixture | `prediction/bags/session_0924_dynamic_prediction_inputs` |
| Older input bag (not dynamic fixture) | `prediction/bags/session_0924_pipe_prediction_inputs` |

## Validation references

| Item | Result | Log / bag |
|------|--------|-----------|
| Upstream condition gate | PASS | `prediction/logs/condition_gate_20260831_035052/report_upstream.json` |
| Dynamic Prediction live E2E | PASS | `prediction/logs/dynamic_e2e_concurrent_20260831_225025/` |
| Dynamic fixture capture + replay | PASS | `prediction/logs/dynamic_fixture_20260831_230721/` |
| Dynamic replay fixture bag | PASS | `prediction/bags/session_0924_dynamic_prediction_inputs` (traj 16 / objects 4 / geom 13 / state 27; output IDs 1, 11) |
| Decision V0 evidence (fixture replay) | PASS | `prediction/logs/decision_evidence_replay_20260901_013026/` (domain 49) |
| Decision Prototype V1 STOP/GO (fixture replay) | PASS | `prediction/logs/decision_stop_go_replay_20260901_013843/` (domain 50) |
| Decision Prototype V1 live E2E | PASS | `prediction/logs/decision_live_e2e_20260901_084141/` (domain 51; predict IDs 7, 12; 121 transitions; **PROTOTYPE ONLY**) |
| Older bag as dynamic replay | FAIL_INPUT_ALIGNMENT | `prediction/logs/dynamic_bag_replay_20260831_225636/` + `session_0924_pipe_prediction_inputs` |
| Physical terrain/rollover correctness | PENDING | — |
| Decision production policy / vehicle control | NOT IMPLEMENTED | Prototype STOP/GO only; **NOT APPROVED FOR VEHICLE CONTROL** |

## Local-only (not in Git)

- `/data/rover_workspace/integration_ws/` — colcon overlay (`build/`, `install/`, `log/`)
- `/data/rover_workspace/prediction/logs/` — generated gate/demo/validation output
- `/data/rover_workspace/raw/` — session recordings (SVO2, MAVLink)
- `/data/rover_workspace/prediction/bags/` — rosbag artifacts
- `/data/rover_workspace/prediction/.cache/` — ZED build caches
