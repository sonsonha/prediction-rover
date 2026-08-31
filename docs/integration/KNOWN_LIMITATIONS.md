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
- Stability/ZMP evidence often invalid on real upstream data.

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
  implemented yet.
- `config/rviz/real_pipeline_debug.rviz` is a debug layout only.

## Repository notes

- `sonsonha` forks are **reproducible integration snapshots**, not necessarily the
  canonical team upstreams.
- `integration_ws` colcon overlay must be built against compatible pinned branches
  (see `INTEGRATION_SNAPSHOT.md`).
