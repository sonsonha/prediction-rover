# Validation Status

Conservative status based on recorded integration runs (session `0924`).
See `INTEGRATION_SNAPSHOT.md` for the pinned repository snapshot.

Do **not** claim full end-to-end dynamic Prediction validation.

## PASS

| Item | Evidence |
|------|----------|
| Real SVO playback | `condition_gate_20260831_035052` |
| MAVLink replay | Same gate; `/lr/mavlink/*` topics |
| Point cloud transformed into `map` | `/lr/point_cloud/cloud_in_map` in gate |
| Segmentation | Gate concurrent startup |
| Terrain geometry | Gate terrain logs |
| Real object 3D boxes | `/terrain_geometry/object_boxes_3d` |
| Trajectory adapter | Gate readiness `trajectory` |
| Geometry adapter | Gate readiness `geometry` |
| Rover state adapter | Gate readiness `rover_state` |
| Tracked objects adapter | Non-empty `pipe` class in gate report |
| All four canonical Prediction inputs together | `report_upstream.json` `success: true` |
| Static Prediction using live real upstream | 3 `/predict_output` msgs in gate |
| Canonical 4-topic input bag creation | `bags/session_0924_pipe_prediction_inputs` |
| Persistent whole-stack restart at SVO EOF | `rviz_demo_20260831_075329` lap 1→2 |
| Existing bridge helper tests | `lr_prediction_bridge` helper unit tests pass |

## PARTIAL

| Item | Notes |
|------|-------|
| Static canonical bag replay | `bag_replay_20260831_042521`: output produced, zero duplicate IDs; formal `success: false` |
| Formal RViz behavior across SVO clock reset | Viewer runs in Docker; lap-boundary flicker/recovery not formally signed off |

## UNVERIFIED

| Item | Notes |
|------|-------|
| Dynamic Prediction E2E | No dedicated live-SVO dynamic validation run signed off |
| Dynamic canonical bag replay | Replay script static phase only logged in last run |
