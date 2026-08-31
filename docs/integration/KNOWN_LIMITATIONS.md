# Known Limitations

## Tracked objects

- Tracked IDs are currently **non-persistent** across frames (frame-local hashing).
- Current tracked-object **velocity is unavailable**; `velocity_valid=false` for real
  terrain-box objects.

## Collision (Prediction V1)

- Collision V1 is **discrete** geometry sampling — no swept interpolation.
- Collision V1 **currently ignores object velocity**.

## Rollover (Prediction V1)

- Rollover model does **not** model suspension, soil interaction, or rotational inertia.
- On validated Dynamic Prediction runs (live E2E + dynamic replay fixture),
  **diagnostic** fields are populated with `acceleration_available=true`,
  `stability_moment.valid=true`, and `zmp.valid=true`.
- **Physical terrain normal correctness** and **physical rollover model validation**
  remain **PENDING** — do not treat diagnostic validity as physical correctness.

## Dynamic bag fixtures / cycle ordering

PredictionRuntime clears cycle-bound objects/geometry/state when a **new**
`trajectory_id` arrives. That is intended cycle semantics, not a bug.

| Bag | Dynamic replay |
|-----|----------------|
| `session_0924_dynamic_prediction_inputs` | **PASS** — ordering preserves successful live cycles (e.g. traj → geometry → objects → state → predict for IDs 1, 11) |
| `session_0924_pipe_prediction_inputs` | **Not a valid dynamic fixture** — objects/state often arrive **before** the new trajectory; cache clear leaves the active cycle without objects → no `/predict_output` (`FAIL_INPUT_ALIGNMENT`). Kept as a reference / static input bag |

## Trajectory source

- Trajectory replay is a **recorded/reference future trajectory**, not an online
  production planner.
- Trajectory adapter applies ~**1 m near-origin skip** — an integration workaround for
  terrain forward coverage.

## SVO / demo

- Native ZED SVO looping is **incompatible** with the current SVO timestamp / `/clock`
  configuration.
- Demo performs **whole-stack restart** on SVO EOF.
- Sim clock **jumps backward** between laps; RViz may flicker.

## Scope gaps

- **Decision node** not implemented.
- **Prediction-specific RViz visualization** (collision/rollover overlays) not
  implemented yet — final customer RViz visualization **PENDING**.
- `config/rviz/real_pipeline_debug.rviz` is a debug layout only.

## Repository notes

- `sonsonha` forks are **reproducible integration snapshots**, not necessarily the
  canonical team upstreams.
- `integration_ws` colcon overlay must be built against compatible pinned branches
  (see `INTEGRATION_SNAPSHOT.md`).
