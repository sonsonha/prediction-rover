# Real Pipeline Architecture

This document describes the validated Landfill Rover → Prediction integration path
using session `0924` SVO replay with MAVLink pose and real terrain/segmentation.

## Data flow

```
session_0924 SVO + MAVLink CSV
        ↓
ZED wrapper + mavlink_csv_pose + pointcloud_transform
        ↓
lr_segmentation + lr_terrain_geometry (object filter)
        ↓
lr_prediction_bridge
        ↓
/trajectory
/tracked_objects
/geometry
/rover/state
        ↓
prediction_ros (Prediction)
        ↓
/predict_output
```

## Canonical frame

All Prediction inputs use **`map`** as the world frame.

## Simulation time

- ZED publishes SVO timestamps on **`/clock`** when `publish_svo_clock:=true`.
- Downstream nodes use **`use_sim_time:=true`**.
- `ROS_DOMAIN_ID=42` is used for integration/demo runs.

## External dependencies (not in this repo)

| Component | Location |
|-----------|----------|
| `lr-ros2` (segmentation, terrain) | `/data/rover_workspace/lr-ros2` |
| `ROS2_rover_trajectory` (bringup, MAVLink, pointcloud) | `/data/rover_workspace/ROS2_rover_trajectory` |
| `integration_ws` colcon overlay | `/data/rover_workspace/integration_ws` |
| Session raw data | `/data/rover_workspace/raw/session_20260710_0924/` |

## Adapter package

`ros2/lr_prediction_bridge` converts upstream topics to `safety_perception_msgs`
types expected by Prediction. See `ros2/lr_prediction_bridge/config/bridge.yaml`.

## Demo lap behavior

`scripts/integration/run_rviz_demo.sh` restarts the **entire** processing stack when
the SVO reaches EOF (native ZED loop is incompatible with SVO timestamps). Sim time
jumps backward at each lap boundary.
