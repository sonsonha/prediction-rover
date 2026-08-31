# Real Pipeline Architecture

This document describes the validated Landfill Rover → Prediction integration path
using session `0924` SVO replay with MAVLink pose, real segmentation, and terrain
geometry.

## Repository roles

The integration uses **reproducible forks/snapshots** under the `sonsonha` GitHub
account. These are **not necessarily the canonical team upstreams** — they pin a
known-good combination for Prediction integration work.

| Role | Integration fork | Original upstream (reference) |
|------|------------------|-------------------------------|
| Prediction + bridge + tooling | [sonsonha/prediction-rover](https://github.com/sonsonha/prediction-rover) | — |
| Segmentation + terrain | [sonsonha/lr-ros2](https://github.com/sonsonha/lr-ros2) | [hachitrung024/lr-ros2](https://github.com/hachitrung024/lr-ros2) |
| ZED / MAVLink / pointcloud | [sonsonha/ROS2_rover_trajectory](https://github.com/sonsonha/ROS2_rover_trajectory) | [haitrung3101nht/ROS2_rover_trajectory](https://github.com/haitrung3101nht/ROS2_rover_trajectory) |

See `INTEGRATION_SNAPSHOT.md` for branch names, SHAs, and the snapshot tag.

## Integrated pipeline

```
SVO2 + MAVLink
      |
      v
ROS2_rover_trajectory
  - ZED playback
  - MAVLink replay
  - pointcloud transform -> map
      |
      v
lr-ros2
  - segmentation
  - terrain geometry
  - object 3D boxes
      |
      v
lr_prediction_bridge
  - /trajectory
  - /tracked_objects
  - /geometry
  - /rover/state
      |
      v
prediction_ros
      |
      v
/predict_output
```

## Canonical frame

All Prediction inputs use **`map`** as the world frame.

## Simulation time

- ZED publishes SVO timestamps on **`/clock`** when `publish_svo_clock:=true`.
- Downstream nodes use **`use_sim_time:=true`**.

## ROS domain

Integration and demo runs use **`ROS_DOMAIN_ID=42`**.

## Packages in this repository

| Package | Path |
|---------|------|
| `prediction_ros` | `ros2/prediction_ros` |
| `safety_perception_msgs` | `ros2/safety_perception_msgs` |
| `lr_prediction_bridge` | `ros2/lr_prediction_bridge` |

Adapter topic wiring: `ros2/lr_prediction_bridge/config/bridge.yaml`.

## Colcon overlay

Upstream packages from `lr-ros2` and `ROS2_rover_trajectory` are typically symlinked
into a local `integration_ws` colcon workspace alongside packages from this repo.
The workspace itself is **not version controlled** — only the pinned repository
branches/SHAs are.

## Session data (local, not in Git)

| Path | Content |
|------|---------|
| `/data/rover_workspace/raw/session_20260710_0924/` | SVO2 + MAVLink CSV |
| `/data/rover_workspace/prediction/bags/` | Canonical input rosbags |
| `/data/rover_workspace/prediction/logs/` | Generated gate/demo logs |

## Demo lap behavior

`scripts/integration/run_rviz_demo.sh` restarts the **entire** processing stack when
the SVO reaches EOF. Native ZED `svo_loop` is incompatible with the current SVO
timestamp / `/clock` configuration. Sim time jumps backward at each lap boundary.
