# Integration Runbook

## Prerequisites

- Linux host with Docker and NVIDIA GPU support
- Workspace mounted at `/data/rover_workspace`
- Session `0924` raw data under `/data/rover_workspace/raw/session_20260710_0924/`
- Integration forks cloned and pinned per `INTEGRATION_SNAPSHOT.md`
- `integration_ws` colcon overlay built against those pins
- Segmentation model weights (e.g. `lr-ros2/models/best.pt`)

## Clone reproducible forks

```bash
# Prediction (this repo)
git clone git@github.com:sonsonha/prediction-rover.git \
  /data/rover_workspace/prediction/prediction-src
cd /data/rover_workspace/prediction/prediction-src
git checkout integration/humble-real-pipeline

# Perception / segmentation / terrain
git clone git@github.com:sonsonha/lr-ros2.git /data/rover_workspace/lr-ros2
cd /data/rover_workspace/lr-ros2
git checkout integration/real-terrain-segmentation

# Trajectory / MAVLink / pointcloud
git clone git@github.com:sonsonha/ROS2_rover_trajectory.git \
  /data/rover_workspace/ROS2_rover_trajectory
cd /data/rover_workspace/ROS2_rover_trajectory
git checkout main   # SHA e345768
```

These `sonsonha` repositories are **integration snapshots**, not guaranteed to track
the canonical team upstreams.

## Docker build

Build the Humble integration image from this repository:

```bash
cd /data/rover_workspace/prediction/prediction-src
docker build -f docker/humble/Dockerfile -t prediction-humble-dev:latest .
```

All integration commands below run **inside** `prediction-humble-dev:latest`.
Do not run ROS nodes or RViz directly on an Ubuntu 24.04 host without the container.

## Required runtime flags

Always use for integration/demo:

```text
ROS_DOMAIN_ID=42
--network host
--ipc=host
```

GPU demo/gate additionally requires `--gpus all`.

Use an isolated clean `ROS_DOMAIN_ID` for formal validation (e.g. 44 live dynamic
E2E; 46/47 dynamic fixture capture/replay). Verify no application nodes before
start.

## Persistent demo (processing stack)

```bash
docker run --rm --gpus all --network host --ipc=host \
  -e ROS_DOMAIN_ID=42 \
  -v /data/rover_workspace:/data/rover_workspace \
  prediction-humble-dev:latest \
  bash /data/rover_workspace/prediction/prediction-src/scripts/integration/run_rviz_demo.sh
```

Runs laps until Ctrl+C. Logs go to `/data/rover_workspace/prediction/logs/rviz_demo_*`
(outside Git).

## RViz viewer (separate container)

RViz runs **inside the Humble Docker image**, not on the host OS.

```bash
docker run --rm --network host --ipc=host \
  -e ROS_DOMAIN_ID=42 \
  -e DISPLAY=:10 \
  -v /data/rover_workspace:/data/rover_workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  prediction-humble-dev:latest \
  bash /data/rover_workspace/prediction/prediction-src/scripts/integration/run_rviz_viewer.sh
```

The viewer loads `config/rviz/real_pipeline_debug.rviz` from this repository.

### RViz display settings

| Setting | Value |
|---------|-------|
| Fixed Frame | `map` |
| Image topic | `/segmentation/overlay` |
| Image Reliability Policy | **Best Effort** |
| Image Durability Policy | **Volatile** |

`config/rviz/real_pipeline_debug.rviz` is a **debug layout only** — not a final
Prediction demo config (collision/rollover visualization not implemented).

## Mac VNC tunnel (view RViz remotely)

On Mac (replace `HOST` with Linux machine IP):

```bash
ssh -L 5910:localhost:5910 user@HOST
```

On the Linux host, ensure a VNC/display server is available at `:10` and that
`DISPLAY=:10` is passed to the viewer container.

## Upstream condition gate (~180 s)

```bash
docker run --rm --gpus all --network host --ipc=host \
  -e ROS_DOMAIN_ID=42 \
  -v /data/rover_workspace:/data/rover_workspace \
  prediction-humble-dev:latest \
  bash /data/rover_workspace/prediction/prediction-src/scripts/integration/run_condition_gate.sh
```

## Canonical bag replay

Default script target is the older bag
`session_0924_pipe_prediction_inputs` (static / historical). For **Dynamic**
Prediction replay validation, use the dynamic fixture instead:

```text
/data/rover_workspace/prediction/bags/session_0924_dynamic_prediction_inputs
```

Topics only: `/trajectory`, `/tracked_objects`, `/geometry`, `/rover/state`
(no `/predict_output`, no `/clock`). Play with `ros2 bag play <bag> --clock`.

Start `prediction_node` with `prediction_profile:=dynamic` and `use_sim_time:=true`
**before** playback. Validated PASS evidence:
`prediction/logs/dynamic_fixture_20260831_230721/`.

Do **not** treat `session_0924_pipe_prediction_inputs` as a dynamic replay fixture
(see `KNOWN_LIMITATIONS.md` / `VALIDATION_STATUS.md`).

```bash
docker run --rm --network host --ipc=host \
  -e ROS_DOMAIN_ID=42 \
  -v /data/rover_workspace:/data/rover_workspace \
  prediction-humble-dev:latest \
  bash /data/rover_workspace/prediction/prediction-src/scripts/integration/run_replay_canonical_bag.sh
```

## Dynamic Prediction validation evidence (local logs)

| Run | Domain | Result | Path |
|-----|--------|--------|------|
| Live concurrent Dynamic E2E | 44 | PASS | `prediction/logs/dynamic_e2e_concurrent_20260831_225025/` |
| Dynamic fixture capture + replay | 46 / 47 | PASS | `prediction/logs/dynamic_fixture_20260831_230721/` |
| Older bag dynamic replay | 45 | FAIL_INPUT_ALIGNMENT | `prediction/logs/dynamic_bag_replay_20260831_225636/` |

Physical terrain/rollover correctness: **PENDING**.
