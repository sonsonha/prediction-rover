# Known Limitations

## Tracked objects

- Track IDs are **non-persistent** across frames (frame-local hashing).
- `velocity_valid=false` for current real objects from terrain boxes.
- Object velocity is **not used** by Collision V1.

## Collision / rollover (Prediction V1)

- Collision V1 uses **discrete geometry** sampling.
- No swept collision interpolation.
- Rollover model has known limitations; stability/ZMP evidence often invalid on real data.

## Trajectory source

- `/lr/mavlink/trajectory_future` is a **recorded/reference future path**, not a production online planner.
- Trajectory adapter applies ~**1 m near-origin skip** (integration workaround for terrain forward coverage).

## SVO / demo

- Native ZED `svo_loop` is **incompatible** with `publish_svo_clock` / SVO timestamps.
- Demo (`run_rviz_demo.sh`) restarts the **complete stack** at SVO EOF.
- Sim clock **jumps backward** each lap; RViz may flicker.

## Scope gaps

- **Decision node** not implemented.
- **Prediction-specific RViz visualization** (collision/rollover overlays) not implemented.
- `config/rviz/real_pipeline_debug.rviz` is a debug layout only.

## Upstream repos

- `lr-ros2` and `ROS2_rover_trajectory` are separate repositories with their own release process.
- `integration_ws` colcon overlay must be built against compatible branches (see `INTEGRATION_SNAPSHOT.md`).
