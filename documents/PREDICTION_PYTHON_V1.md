# Prediction Python V1

Canonical technical document for the ROS-independent Prediction package.

Validation evidence: [`ROLLOVER_METHOD_COMPARISON.md`](../ROLLOVER_METHOD_COMPARISON.md),
[`DYNAMIC_ROLLOVER_METRICS.md`](DYNAMIC_ROLLOVER_METRICS.md),
`outputs/rollover_method_comparison.json`.

## 1. Purpose

Prediction produces **safety evidence** for a downstream Decision Node:

- collision candidates (footprint vs tracked objects)
- terrain-following roll / pitch
- static and dynamic rollover margins / moments

It does **not** assign severity, safe/unsafe labels, Stop/Go, or thresholds.

Version marker: **Prediction Python V1** (`prediction_core.version`).

## 2. Python architecture

```text
                        Prediction V1

Trajectory ────────────────┐
Tracked Objects ───────────┤
Geometry ──────────────────┤
Rover State ───────────────┤
External Wrench ───────────┘
                            │
                            ▼
                    PredictionRuntime
                      cache / cycle
                      readiness
                      validation
                            │
                            ▼
                    PredictionCore
                    /              \
                   /                \
                  ▼                  ▼
        CollisionPredictor    RolloverPredictor
                                   │
                     ┌─────────────┼─────────────┐
                     ▼             ▼             ▼
                roll/pitch     Static SSM    Dynamic Evidence
                                  │               │
                                  ▼               ▼
                           Normalized SSM   Stability Moment
                                                  │
                                         optional ZMP/debug
                            │
                            ▼
                     PredictionOutput
```

`PredictionCore` is independent of ROS, filesystem, CLI, and transport.

## 3. Runtime readiness profiles

Prediction executes **at most once per trajectory cycle**, so readiness policy
determines which inputs must exist before the cycle is consumed.

```python
from prediction_core import PredictionProfile, PredictionRuntime

PredictionRuntime(config)                         # default: static
PredictionRuntime(config, profile="static")
PredictionRuntime(config, profile=PredictionProfile.DYNAMIC)
```

### STATIC PROFILE (default)

Required before prediction:

- active trajectory
- tracked objects batch (`[]` is valid)
- matching geometry
- frame / cycle validation

`RoverState` and external wrenches are **optional**.

Behavior: `trajectory → objects → geometry → predict immediately`.

### DYNAMIC PROFILE

Required before prediction:

- everything static requires, **plus**
- `RoverState` with valid linear `acceleration_xyz`

Do **not** treat `acceleration_xyz=None` as zero. Do **not** require linear /
angular velocity or angular acceleration (current algorithms do not use them).

External wrench remains optional evidence:

| Value | Meaning |
|---|---|
| `external_wrenches=None` | Wrench information unavailable |
| `external_wrenches=[]` | Explicitly available and empty |
| `external_wrenches=[...]` | One or more wrenches available |

Dynamic may predict with valid acceleration when wrenches are `None`; assumptions
must still record that wrench information was unavailable. There is **no**
wrench wait / timeout in V1 (a future `dynamic_wrench_required` policy is out of
scope).

### Cycle reset (Python V1)

A new trajectory ID clears cycle-bound **objects**, **geometry**, **state**, and
**external wrenches**. Prior-cycle state cannot accidentally satisfy dynamic
readiness for a new cycle.

## 4. Required / optional inputs

| Input | Role |
|---|---|
| Trajectory | Discrete route poses `(x, y, yaw)` in `map` |
| Tracked objects | Metric polygons (`[]` = known empty; `None` = unavailable in runtime) |
| Geometry | Terrain plane normal per `step_id` |
| Rover YAML | Mass, body footprint, support polygon, CoM |
| `RoverState.acceleration_xyz` | Required for **dynamic** readiness; optional for static |
| `external_wrenches` | Optional additional dynamic evidence |

Availability:

| Value | Meaning |
|---|---|
| `acceleration_xyz=None` | Unavailable (never treated as zero) |
| `acceleration_xyz=(0,0,0)` | Valid zero kinematic acceleration |
| `external_wrenches=None` | Wrench information unavailable |
| `external_wrenches=[]` | Explicitly empty |

`angular_velocity_xyz` may be present but is unused (no inertia tensor).

`PredictionCore.predict(..., state=None, external_wrenches=None)` remains valid
for static evidence; readiness profiles belong to the **runtime** layer only.

## 5. Collision V1

Rover **body** rectangle at each discrete trajectory pose vs tracked object polygon:

- Shapely minimum distance
- candidate iff distance ≤ `collision_margin_m`

Limitations (intentional V1):

- discrete poses only — **no swept-path interpolation**
- object velocity is **not** used for motion prediction

## 6. Rollover baseline (PRIMARY)

When trajectory + geometry + config are valid:

| Field | Meaning |
|---|---|
| `predicted_roll_deg` / `predicted_pitch_deg` | Terrain-normal → attitude |
| `static_stability_margin_m` | Gravity projection → min signed support margin |
| `normalized_static_stability_margin` | Edge-wise vs flat reference margins |
| `nearest_static_edge` | Edge with min raw static margin |

## 7. Dynamic rollover

Point-mass / translational model only:

- gravity
- translational acceleration (`F = −m a`)
- external force with known application point
- external free torque

**Not** included: `−Iα`, `−ω×Iω`, suspension, track-load LTR, soil mechanics.

## 8. Metric hierarchy

```text
PRIMARY BASELINE
  Terrain normal → roll/pitch
  Static SSM
  Normalized Static SSM

PRIMARY DYNAMIC EXTENSION
  Stability Moment / Moment Balance

OPTIONAL DIAGNOSTIC
  Point-mass / translational ZMP

SECONDARY DIAGNOSTICS
  Critical geometric tip angle   (config / chassis property)
  Effective-gravity SSM          (acceleration-only resultant)
```

**Equivalence:** under gravity + translational acceleration with empty wrenches,
Effective SSM ≈ Point-mass ZMP margin (same resultant line of action). They are
**not** independent primary algorithms. External force height / free torque make
ZMP and Stability Moment diverge from Effective SSM.

**Edge naming:**

| Concept | Field |
|---|---|
| Nearest support edge (raw margin) | `nearest_static_edge`, `nearest_effective_edge`, `nearest_zmp_edge` |
| Most depleted normalized moment | `minimum_normalized_moment_edge` (alias of legacy `critical_edge`) |
| Smallest tip angle | `minimum_tip_angle_edge` (alias of tip `critical_edge`) |

Nearest geometric edge ≠ most depleted normalized moment edge in general.

## 9. Coordinate / frame convention

- Common ENU / `map`: +X East, +Y North, +Z Up
- Rover: +X forward, +Y left, +Z up
- Gravity: `g_world = (0, 0, −9.80665)`
- Pitch positive nose-up; roll right-hand about rover +X

## 10. Availability semantics

See §4. Dynamic evidence is marked `valid=False` when acceleration is missing.
Unavailable quantities serialize as JSON `null`, never silent zeros.

## 11. How to run

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[test]'
.venv/bin/python -m pytest -q

# Easy Makefile demos (PNG + JSON under outputs/runtime_demos/)
make demo-viz
make demo-flat
make demo-dynamic
make runtime SCENARIO=mock/runtime_scenarios/demo_side_slope_15deg.json PROFILE=static

# Canonical V1 CLI (default profile: static)
.venv/bin/python -m prediction_core.replay \
  --profile static \
  --config config/rover.mock.yaml \
  --scenario mock/runtime_scenarios/demo_flat_static.json \
  --viz-dir outputs/runtime_demos/demo_flat_static

.venv/bin/python -m prediction_core.replay \
  --profile dynamic \
  --config config/rover.mock.yaml \
  --scenario mock/runtime_scenarios/demo_dynamic_waits_for_state.json

# Algorithm regression mocks (plots)
.venv/bin/python -m prediction_core.cli run-all-mocks \
  --config config/rover.mock.yaml \
  --output outputs/mock_validation
```

Human output includes `Prediction profile: static|dynamic` and exact readiness
reasons when waiting (e.g. `missing rover state`, `rover acceleration unavailable`).

Demo streams: existing `demo_*.json` keep state before geometry (works for both
profiles). `demo_dynamic_waits_for_state` / `demo_dynamic_zero_acceleration`
prove geometry-before-state waiting under `--profile dynamic`.
## 12. Example outputs

Human summary prints baseline + primary Stability Moment + optional ZMP.
Full JSON via `--output` contains nested `critical_tip` and `dynamic_stability`.

## 13. Known limitations

- Mock rover parameters (not measured CAD)
- Collision: discrete steps; no object dynamics
- Dynamic model: no inertia tensor / full rigid-body ZMP / LTR / FASM
- No Decision thresholds
- ROS wrappers exist as design/stubs; not validated in this Python V1 phase

## 14. Future ROS integration

ROS is a **transport/adapter** around frozen `PredictionRuntime` + `PredictionCore`.

- Parameter `prediction_profile`: `static` | `dynamic`
- Adapters map `RoverState.acceleration.linear.{x,y,z}` → `acceleration_xyz`
- External wrench `None` ≠ `[]` preserved across the ROS boundary
- Output: baseline SSM + `StabilityMomentEvidence` + diagnostic `ZmpEvidence`

See [`ROS_UPSTREAM_INTERFACE_CONTRACT.md`](ROS_UPSTREAM_INTERFACE_CONTRACT.md).

Do not expand Decision semantics into ROS messages.
