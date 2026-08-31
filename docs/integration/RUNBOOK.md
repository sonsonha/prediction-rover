# Integration Runbook

## Prerequisites

- Docker with NVIDIA GPU support
- Host workspace mounted at `/data/rover_workspace`
- Session `0924` raw data under `/data/rover_workspace/raw/session_20260710_0924/`
- `integration_ws` built (see `integration_ws/README.md` in workspace)
- Segmentation model: `${LR_ROS2}/models/best.pt`

## Docker build

```bash
cd /data/rover_workspace/prediction/prediction-src
docker build -f docker/humble/Dockerfile -t prediction-humble-dev:latest .
```

## Environment

Always use for integration/demo:

```text
ROS_DOMAIN_ID=42
--network host
--ipc=host
```

## Persistent demo (processing stack)

```bash
docker run --rm --gpus all --network host --ipc=host \
  -e ROS_DOMAIN_ID=42 \
  -v /data/rover_workspace:/data/rover_workspace \
  prediction-humble-dev:latest \
  bash /data/rover_workspace/prediction/prediction-src/scripts/integration/run_rviz_demo.sh
```

Runs laps until Ctrl+C. Logs go to `/data/rover_workspace/prediction/logs/rviz_demo_*`.

## RViz viewer (separate container)

```bash
docker run --rm --network host --ipc=host \
  -e ROS_DOMAIN_ID=42 \
  -e DISPLAY=:10 \
  -v /data/rover_workspace:/data/rover_workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  prediction-humble-dev:latest \
  bash /data/rover_workspace/prediction/prediction-src/scripts/integration/run_rviz_viewer.sh
```

### RViz settings

- **Fixed Frame:** `map`
- **Image topic:** `/segmentation/overlay`
- **Image reliability:** Best Effort (configured in `config/rviz/real_pipeline_debug.rviz`)

## Mac VNC tunnel (view RViz remotely)

On Mac (replace `HOST` with Linux machine IP):

```bash
ssh -L 5910:localhost:5910 user@HOST
```

On Linux host, ensure VNC/display `:10` is available and `DISPLAY=:10` is set in the
viewer container.

## Upstream condition gate (~180 s)

```bash
docker run --rm --gpus all --network host --ipc=host \
  -e ROS_DOMAIN_ID=42 \
  -v /data/rover_workspace:/data/rover_workspace \
  prediction-humble-dev:latest \
  bash /data/rover_workspace/prediction/prediction-src/scripts/integration/run_condition_gate.sh
```

## Canonical bag replay

```bash
docker run --rm --network host --ipc=host \
  -e ROS_DOMAIN_ID=42 \
  -v /data/rover_workspace:/data/rover_workspace \
  prediction-humble-dev:latest \
  bash /data/rover_workspace/prediction/prediction-src/scripts/integration/run_replay_canonical_bag.sh
```
