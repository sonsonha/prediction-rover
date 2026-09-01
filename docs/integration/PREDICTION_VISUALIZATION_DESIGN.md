# Prediction Visualization Design

Visualization-only design for RViz. **No** PredictionCore / Runtime / physics /
adapter / Decision changes. **No** invented risk thresholds.

Validated dynamic fixture (IDs 1, 11):
`prediction/bags/session_0924_dynamic_prediction_inputs`

Physical terrain/rollover correctness: **PENDING**. Decision: **NOT IMPLEMENTED**.

## Exact message contracts (from `.msg` + adapters)

### Inputs (map frame)

| Topic | Type | Key fields |
|-------|------|------------|
| `/trajectory` | `Trajectory` | `header`, `trajectory_id`, `steps[]` with `step_id`, `x`, `y`, `yaw` (map) |
| `/tracked_objects` | `TrackedObjectArray` | `header`, `objects[]` with `track_id`, `class_name`, `footprint_polygon_xy[]` (`Point2D` x,y in **map**), velocity optional |
| `/geometry` | `GeometryArray` | `header`, `source_trajectory_id`, `steps[]` with `step_id`, `plane_id`, `normal` (`Vector3`), `confidence`/`confidence_valid` |
| `/rover/state` | `RoverState` | `header`, `pose`/`pose_valid`, `twist`/`twist_valid`, `acceleration`/`acceleration_valid` (pose in **map**) |

Empty `TrackedObjectArray.objects=[]` is valid input.

### Output

`PredictionOutput`:

| Field | Meaning |
|-------|---------|
| `header` | Stamp / frame of the prediction publish |
| `source_trajectory_id` | Must match `Trajectory.trajectory_id` of the cycle that produced this output |
| `collision_steps[]` | Only steps with **collision candidates** (`collision_objects` non-empty); keyed by `step_id` |
| `rollover_steps[]` | Per-step rollover evidence; keyed by `step_id` |

#### `CollisionStep`

- `step_id` — join to `TrajectoryStep.step_id` (**not** array index)
- `distance_along_route_m`
- `collision_objects[]` — each: `track_id`, `object_class`, `min_distance_m`, `confidence`/`confidence_valid`

Presence of a `CollisionStep` means at least one object was within
`collision_margin_m` of the body footprint at that step. There is **no** separate
boolean or severity enum in the message.

#### `RolloverStep`

- `step_id` — join to `TrajectoryStep.step_id`
- `predicted_roll_deg`, `predicted_pitch_deg`
- `static_stability_margin_m`, `normalized_static_stability_margin`
- `stability_moment` (`StabilityMomentEvidence`)
- `zmp` (`ZmpEvidence`)
- `terrain_id`, `confidence`, `confidence_valid`

No Decision / safe-warning-danger field exists. Visualization shows reported
values only.

#### `StabilityMomentEvidence`

- `valid`, `validity_reason`
- Edge moments / normalized moments (front/rear/left/right)
- `minimum_stability_moment_nm`, `normalized_minimum_stability_moment`
- `minimum_normalized_moment_edge`
- `acceleration_available`, `external_wrench_available`, `external_wrench_included`

Invalid → display **N/A**, never fabricate `0`.

#### `ZmpEvidence`

- `valid`, `x`, `y`, `margin_m`, `normalized_margin`, `nearest_edge`

### ZMP coordinate semantics (critical)

From `prediction_core/rollover.py` and offline `visualization/rollover_plot.py`:

- Point-mass ZMP is computed in the **rover support-plane / body-local XY**:
  - **+X** = rover forward
  - **+Y** = rover left
  - Origin = support rectangle center (same convention as support polygon plot)
- `ZmpEvidence.x` / `.y` are **not** map coordinates.

To place ZMP in **map** for RViz, use the matching trajectory step pose:

```text
map_x = step.x + cos(yaw) * zmp.x - sin(yaw) * zmp.y
map_y = step.y + sin(yaw) * zmp.x + cos(yaw) * zmp.y
```

This matches `geometry_utils.rover_rectangle` local→map corner transform.
Invalid ZMP (`valid=false`) → **no** ZMP marker.

### Join / cycle rules

1. Collision and rollover positions: look up `Trajectory.steps` by `step_id`.
2. Attach `PredictionOutput` **only** when `source_trajectory_id == trajectory_id`.
3. On new trajectory without a matching prediction yet: clear stale
   collision / rollover / ZMP / prediction-status markers.
4. Geometry normals: display provided vectors faithfully; do not flip/correct.
   Skip non-finite / zero-length normals (no fabrication).

## Visualization topics

| Topic | Type |
|-------|------|
| `/prediction_viz/trajectory` | `nav_msgs/Path` (+ optional step MarkerArray on same node publish path) |
| `/prediction_viz/objects` | `visualization_msgs/MarkerArray` |
| `/prediction_viz/terrain` | `visualization_msgs/MarkerArray` |
| `/prediction_viz/rover` | `visualization_msgs/MarkerArray` |
| `/prediction_viz/collision` | `visualization_msgs/MarkerArray` |
| `/prediction_viz/rollover` | `visualization_msgs/MarkerArray` |
| `/prediction_viz/zmp` | `visualization_msgs/MarkerArray` |
| `/prediction_viz/status` | `visualization_msgs/MarkerArray` |

`fixed_frame` default: `map`.

## Package

`ros2/prediction_visualization` — pure `marker_builders` + ROS `node`; unit tests
without RViz.

## Validation

- Unit tests: `ros2/prediction_visualization/test` (13 passed).
- Fixture replay topic check (domain 48): predict source IDs **1, 11**; all
  `/prediction_viz/*` topics published (`prediction_viz_replay_20260831_domain48`).

Physical terrain/rollover correctness: **PENDING**. Decision: **NOT IMPLEMENTED**.

## Out of scope

- Decision node / invented severity
- Physics / terrain-normal “fixes”
- Changing `config/rviz/real_pipeline_debug.rviz` (upstream debug preset stays)
