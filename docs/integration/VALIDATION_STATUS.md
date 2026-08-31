# Validation Status

Conservative status based on recorded integration runs (session `0924`).

## PASS

| Item | Evidence |
|------|----------|
| Real SVO playback | `condition_gate_20260831_035052` |
| MAVLink CSV pose replay | Same gate; `/lr/mavlink/*` topics |
| Point cloud transformed to `map` | `/lr/point_cloud/cloud_in_map` in gate |
| Segmentation | Gate concurrent startup |
| Terrain geometry | Gate terrain logs |
| Real 3D object boxes | `/terrain_geometry/object_boxes_3d` |
| Trajectory adapter | Gate readiness `trajectory` |
| Geometry adapter | Gate readiness `geometry` |
| Rover state adapter | Gate readiness `rover_state` |
| Tracked objects adapter | Non-empty `pipe` class in gate report |
| All four canonical inputs together | `report_upstream.json` `success: true` |
| Static Prediction with live upstream | 3 `/predict_output` msgs in gate |
| Canonical 4-topic input bag | `bags/session_0924_pipe_prediction_inputs` |
| Persistent SVO EOF whole-stack restart | `rviz_demo_20260831_075329` lap 1→2 |
| `lr_prediction_bridge` unit tests | 10 passed in gate (`pytest.log`) |

## PARTIAL

| Item | Notes |
|------|-------|
| Static canonical bag replay | `bag_replay_20260831_042521`: output produced, zero duplicate IDs; formal `success: false` |
| RViz formal validation across clock reset | Viewer script exists; interactive lap-boundary behavior not formally signed off |

## UNVERIFIED

| Item | Notes |
|------|-------|
| Dynamic Prediction E2E on live SVO | Gate used static profile for upstream validation |
| Dynamic canonical bag replay | Replay script static phase only logged in last run |

Do **not** claim full dynamic Prediction validation until dedicated E2E evidence exists.
