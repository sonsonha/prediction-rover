# Prediction — ROS 2 Humble dev (Ubuntu 22.04)

Layered on the upstream integration base image family:

`stereolabs/zed:5.4-devel-cuda12.8-ubuntu22.04`

This workflow is **separate** from the known-good Jazzy host setup:

- Jazzy build: `ros2/install/`
- Humble build: `ros2/install_humble/`
- Jazzy venv: `.venv/`
- Humble venv: `.venv-humble/`

## Quick start (RTX host)

```bash
cd /data/rover_workspace/prediction/prediction-src

make humble-image      # once: build Docker image
make humble-install    # Python 3.10 + prediction_core
make humble-build      # colcon -> ros2/install_humble
make humble-test       # 148 pytest
make humble-static     # mock static smoke
make humble-dynamic    # mock dynamic smoke
make humble-bag        # short rosbag regression
make humble-shell      # interactive shell
```

## Live upstream (inside container)

Terminal 1 — Prediction (waits for upstream topics):

```bash
source scripts/humble_env.sh
ros2 launch prediction_ros prediction.launch.py \
  config_path:=/workspace/prediction/prediction-src/config/rover.mock.yaml \
  prediction_profile:=dynamic
```

Terminal 2 — monitor:

```bash
source scripts/humble_env.sh
ros2 topic echo /predict_output
```

No mock dependency in production launch. Upstream must publish:

- `/trajectory`
- `/tracked_objects`
- `/geometry`
- `/rover/state` (dynamic profile)

## Note on upstream lr-ros2

`lr-ros2` was **not found** on this RTX machine at inspection time.
When that repo is available, prefer reusing its Dockerfile/entrypoint and
only add Prediction-specific layers (Python deps + mount paths).
