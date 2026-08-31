# Validation Status

Conservative status based on recorded integration runs (session `0924`).
See `INTEGRATION_SNAPSHOT.md` for the pinned repository snapshot.

**PHYSICAL TERRAIN/ROLLOVER CORRECTNESS REMAINS PENDING.** Runtime dynamic
diagnostics (acceleration, Stability Moment, ZMP validity flags) are validated;
physical model correctness is not.

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
| Dynamic Prediction live E2E | `dynamic_e2e_concurrent_20260831_225025` — 2 `/predict_output`; `acceleration_available=true`; SM/ZMP valid; no duplicates; no NaN/Inf |
| Real acceleration consumed (dynamic) | Same live E2E + fixture replay |
| Stability Moment diagnostics valid (dynamic) | Same; `stability_moment.valid=true` |
| ZMP diagnostics valid (dynamic) | Same; `zmp.valid=true` |
| Dynamic replay fixture | `session_0924_dynamic_prediction_inputs` — replay PASS in `dynamic_fixture_20260831_230721` (outputs for trajectory IDs 1, 11) |
| Canonical 4-topic input bag creation | `bags/session_0924_pipe_prediction_inputs` (static/upstream input bag; see note below) |
| Persistent whole-stack restart at SVO EOF | `rviz_demo_20260831_075329` lap 1→2 |
| Existing bridge helper tests | `lr_prediction_bridge` helper unit tests pass |

### Dynamic replay fixture (PASS applies only to)

```text
/data/rover_workspace/prediction/bags/session_0924_dynamic_prediction_inputs
```

| Topic | Count |
|-------|------:|
| `/trajectory` | 16 |
| `/tracked_objects` | 4 |
| `/geometry` | 13 |
| `/rover/state` | 27 |

Derived from a successful live concurrent Dynamic Prediction run
(`dynamic_live_evidence_20260831`). Successful live/replay source trajectory IDs:
**1, 11**. Replay produced **2** `/predict_output` messages with matching dynamic
diagnostics.

### Older bag — not a valid dynamic replay fixture

```text
/data/rover_workspace/prediction/bags/session_0924_pipe_prediction_inputs
```

Kept as an **input-alignment failure case** for Dynamic Prediction replay.
Message ordering repeatedly places `tracked_objects` / `rover/state` **before** a
new `trajectory_id`. PredictionRuntime correctly clears the previous cycle cache on
new trajectory → inputs never co-present for the active cycle → no `/predict_output`.

This is **not** a PredictionRuntime bug. Do not use this bag as the dynamic replay
fixture. See `dynamic_bag_replay_20260831_225636` (`FAIL_INPUT_ALIGNMENT`).

## PARTIAL / PENDING

| Item | Notes |
|------|-------|
| Physical terrain normal correctness | Runtime path validated; physical normal/terrain correctness not signed off |
| Physical rollover model validation | SM/ZMP diagnostic flags valid on dynamic path; physical rollover correctness PENDING |
| Static canonical bag replay | `bag_replay_20260831_042521`: output produced, zero duplicate IDs; formal `success: false` |
| Final customer RViz visualization | Debug layout only; collision/rollover overlays not customer-ready |
| Decision node | Not implemented |
| Formal RViz behavior across SVO clock reset | Viewer runs in Docker; lap-boundary flicker/recovery not formally signed off |

## Status summary

**PASS:** static Prediction live · dynamic Prediction live E2E · dynamic replay
fixture (`session_0924_dynamic_prediction_inputs`) · real acceleration consumed ·
Stability Moment diagnostics valid · ZMP diagnostics valid

**PARTIAL / PENDING:** physical terrain normal correctness · physical rollover
model validation · final customer RViz visualization · Decision node
