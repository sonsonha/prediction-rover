# Landfill Rover Prediction Core V1

This package produces **safety evidence** for a downstream Decision Node. V1
implements exact 2D footprint collision candidates, terrain-normal-derived roll
and pitch (R0), and quasi-static Static Stability Margin (R1). It deliberately
does not assign severity, overall risk, nearest hazards, thresholds, or Stop/Go.
It has no ROS dependency and needs no upstream service to run.

> `config/rover.mock.yaml` contains TEST / MOCK VALUES ONLY. Its dimensions and
> CoM values are plausible software fixtures, not measured rover parameters. Populate
> `config/rover.example.yaml` with verified dimensions before field use.

## Package layout

```text
prediction_core/   validated contracts, algorithms, and combined API
mock/              deterministic JSON inputs and loader
visualization/     development plots (never decision logic)
tests/             synthetic convention and edge-case tests
config/            placeholder and explicitly mock rover parameters
ros2/              future adapter design only
```

The integration entry point is:

```python
PredictionCore.predict(
    trajectory: Trajectory,
    tracked_objects: list[TrackedObject],
    geometry: list[GeometryStep],
    state: RoverState | None = None,
) -> PredictionOutput
```

`RoverState` is accepted for future compatibility but is not used by V1.

## Common frame and sign conventions

All input is metric and already transformed into one common ENU/map frame:

- world +X East, +Y North, +Z Up;
- rover +X forward, +Y left, +Z up;
- yaw is radians, positive counter-clockwise about world +Z (+X toward +Y);
- pitch is positive nose-up: terrain height rises along rover +X;
- roll follows the right-hand rule around rover +X: positive roll means terrain
  rises toward rover +Y (the left side is higher).

Normals are normalized internally and reversed when necessary to point upward.
Zero normals are invalid. Normals with normalized `abs(nz) <= 1e-6` represent a
vertical/nearly vertical surface and are rejected as V1 ground planes. Tests use
constructed 15-degree planes to lock yaw, pitch, roll, and opposite-slope signs.

## Exact upstream input contract

Upstream modules will eventually convert their outputs to these fields (or ROS
messages with identical meaning):

```text
Trajectory
  timestamp: float
  frame_id: str
  steps: list[TrajectoryStep]
TrajectoryStep
  step_id: int
  x, y: float metres in map
  yaw: float radians

TrackedObject
  timestamp: float
  track_id: int | str               (unique per prediction call)
  class_name: str
  footprint_polygon_xy: list[(x,y)] (valid metric map polygon)
  height_m: float | None
  velocity_xy: (vx,vy) | None
  confidence: float | None

GeometryStep
  timestamp: float
  step_id: int                      (joins directly to TrajectoryStep)
  plane_id: int | str
  normal_xyz: (nx,ny,nz)
  centroid_xyz: (x,y,z) | None
  confidence: float | None

RoverState (optional and unused by V1)
  timestamp: float
  x, y, yaw, roll, pitch: float | None
  velocity_xy, acceleration_xy: tuple[float,float] | None
  angular_velocity_xyz: tuple[float,float,float] | None
```

Prediction never consumes raw image masks. Objects are 2.5D by contract: a 2D
footprint plus optional height; there is intentionally no `position_3d`,
`size_3d`, separate heading array, or geometry `region`.

## Collision V1

For each trajectory step, the predictor rotates a rectangle of configured rover
length and width about `(x, y)`. For every valid object polygon it computes the
exact Shapely boundary/area distance. A candidate is emitted iff:

```text
minimum(rover physical footprint, object footprint) <= collision_margin_m
```

Overlap therefore has distance zero. The development plot also draws a
rectangular safety footprint expanded by the margin on every side. That visual
rectangle is not substituted for the exact distance rule. One step can emit
multiple objects, and the same object can occur at several steps. Malformed,
self-intersecting, non-finite, zero-area polygons and duplicate IDs raise clear
errors rather than being repaired silently.

## Rollover R0: normal to attitude

For yaw `psi`, horizontal forward and left directions are:

```text
f = [cos(psi), sin(psi), 0]
l = [-sin(psi), cos(psi), 0]
```

For upward unit normal `n`, plane rise along either horizontal direction `d` is
`-(n dot d) / nz`. Signed pitch and roll are the corresponding `atan2` slopes.
This predicts the terrain-following quasi-static attitude, not body dynamics.

## Rollover R1: Static Stability Margin

The implementation constructs orthonormal forward, left, and terrain-up axes.
It transforms world gravity into that rover frame, traces a line from configured
CoM along gravity, and intersects it with support plane `z=0`. SSM is the
minimum signed directional margin to the four edges of the rectangular support
polygon:

- positive: projection is inside;
- zero: projection is on a tipping boundary;
- negative: projection is outside.

`static_stability_margin_m` is the physical signed distance in metres to the
nearest support edge. `normalized_static_stability_margin` is a separate,
dimensionless edge-wise metric:

```text
min(current_margin_to_edge / reference_margin_to_same_edge)
```

The reference margins come from the configured flat-terrain CoM position, so
longitudinal and lateral edges retain their own reference distances. `1.0`
means the reference position, `0.0` a tipping edge, and negative values are
beyond the support polygon; the result is never clamped. This is an edge-wise
normalized geometric SSM, **not** Normalized Energy Stability Margin (NESM).

No safety class or threshold is inferred from this value. The quasi-static model
assumes rigid terrain contact, a rectangular support polygon, constant configured
CoM, no suspension articulation, and no inertial or load-transfer effects.

### Rollover V1 assumptions

- Rover is static/quasi-static: no velocity, acceleration, angular velocity, or
  inertial force is used.
- Collision uses the configured **body footprint**; rollover uses the separate
  track-contact **support polygon**.
- CoM and CoG are equivalent under uniform gravity. V1 supports configured
  offsets, while the mock baseline centers CoM in XY: `(0, 0, 0.33 m)`.
- The mock support polygon is `0.75 × 0.88 m`; all mock dimensions are temporary
  simulation estimates and must be replaced by measured/CAD rover parameters.
- Dynamic FASM, ZMP, LTR, and vehicle dynamics are explicitly out of scope.

## Missing geometry and timestamp validity

Geometry joins to trajectory strictly by `step_id`. With no match, collision
still runs, no `RolloverStep` is fabricated, the missing ID is logged, and it is
available as `core.rollover_predictor.last_missing_step_ids`.

V1 does not assume a production freshness timeout. Core output retains
`source_trajectory_stamp`; the typed ROS output uses `source_trajectory_id`.
See [`documents/ROS_UPSTREAM_INTERFACE_CONTRACT.md`](documents/ROS_UPSTREAM_INTERFACE_CONTRACT.md)
and [`ros2/README.md`](ros2/README.md) for the canonical ROS interface.
`PredictionCore.stale_input_warnings(...)` supports optional object and geometry
age limits. Passing no limits disables stale classification. A ROS adapter or
deployment config must later supply validated policies.

## Output contract

`PredictionOutput` has an output `timestamp`, `source_trajectory_stamp`, only
non-empty `collision_steps`, and available `rollover_steps`. Collision entries
contain step ID, cumulative distance from trajectory step 0, and every candidate's
ID, class, exact minimum distance, and optional confidence. Rollover entries
contain step ID, signed degrees, SSM metres, terrain ID, and optional confidence.

The output intentionally has no predicted boolean, severity, first/nearest
hazard, critical distance, overall risk, or motion command. Those are policy and
remain the Decision Node's responsibility.

## Install and test

Use Python 3.10 or newer (the verified local run used Python 3.11):

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[test]'
.venv/bin/python -m pytest -q
```

## Run deterministic mocks

```bash
.venv/bin/python -m prediction_core.cli run-scenario \
  mock/scenarios/pipe_collision.json \
  --config config/rover.mock.yaml \
  --output outputs/pipe_collision

.venv/bin/python -m prediction_core.cli run-all-mocks \
  --config config/rover.mock.yaml \
  --output outputs/mock_validation
```

Each scenario writes `prediction_output.json`, `collision_topdown.png`,
`rollover_profile.png`, and `run_summary.txt`. The six mocks cover flat/empty,
pipe overlap, 0.15 m near-margin, 0.25 m outside-margin, 15-degree uphill, and
the same world slope observed as side slope after a 90-degree yaw change.

## Known limitations and roadmap

- Parameters are uncalibrated mocks; body footprint, support polygon, and loaded CoM require
  measurement and field validation.
- Objects are treated as stationary V1 polygons despite optional velocity.
- Terrain is one plane per sampled trajectory step; uncertainty is only passed
  through as confidence and is not propagated.
- R1 is quasi-static and the support polygon is a simple rectangle.
- Synchronization, ROS messages, runtime QoS, and field telemetry are pending.

R2 will add dynamic Force-Angle Stability Measure (FASM). Later work may add ZMP,
LTR, a validated dynamic vehicle model, a thin ROS 2 wrapper, and field parameter
calibration. None belongs in V1.
