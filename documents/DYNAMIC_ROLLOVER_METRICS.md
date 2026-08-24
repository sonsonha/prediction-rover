# Dynamic Rollover Metrics (Pure Python V1)

See also the canonical overview: [`PREDICTION_PYTHON_V1.md`](PREDICTION_PYTHON_V1.md)
and validation report [`ROLLOVER_METHOD_COMPARISON.md`](../ROLLOVER_METHOD_COMPARISON.md).

## Metric hierarchy

```text
PRIMARY BASELINE
  Terrain normal → predicted_roll_deg / predicted_pitch_deg
  static_stability_margin_m
  normalized_static_stability_margin
  nearest_static_edge

PRIMARY DYNAMIC
  Stability Moment (edge_stability_moments_nm, normalized_*,
                    minimum_normalized_moment_edge / legacy critical_edge)

OPTIONAL DIAGNOSTIC
  Point-mass ZMP (zmp_xy, zmp_margin_m, nearest_zmp_edge)

SECONDARY DIAGNOSTICS
  Critical tip angle (CriticalTipEvidence) — chassis/config property
  Effective-gravity SSM — acceleration-only resultant diagnostic
```

Under gravity + translational acceleration with `external_wrenches=[]`,
**Effective SSM ≈ Point-mass ZMP margin** (same resultant line of action).
They are not independent primary algorithms. External force application height
and free torque make ZMP / Stability Moment diverge from Effective SSM.

## Acceleration semantics

`RoverState.acceleration_xyz` is kinematic CoM acceleration in `map` (m/s²)
**without gravity**.

| Value | Meaning |
|---|---|
| `(0,0,0)` | Valid stationary / zero kinematic acceleration |
| `None` | Unavailable — never treated as zero |

Gravity is introduced by Prediction as `g_world = (0, 0, -9.80665)`.

Do **not** feed raw accelerometer specific force into `acceleration_xyz`.

`acceleration_xy` remains for compatibility and is not promoted to 3-D.
`angular_velocity_xyz` is accepted but unused (no inertia tensor in config).

## External wrench semantics

| Argument | Meaning |
|---|---|
| `external_wrenches=None` | Unavailable / not supplied |
| `external_wrenches=[]` | Explicitly empty |

`ExternalWrench` vectors are in `map` (N, N·m, m). `torque_xyz` is a free couple.
A force without `application_point_xyz` does **not** invent a CoM lever arm.

## Implemented algorithms

### A. Critical geometric tip angle (secondary diagnostic)
`theta_i = atan(reference_margin_i / com_height)` on the flat configured support.
Outputs per-edge degrees, `minimum_deg`, and `critical_edge` /
`minimum_tip_angle_edge`. Not Decision thresholds.

### B. Static SSM (primary baseline)
World gravity projection onto the terrain support plane.
Raw minimum edge margin + edge-normalized SSM vs flat reference margins.
Signed and unclamped. Publishes `nearest_static_edge`.

### C. Effective-gravity / inertial SSM (secondary diagnostic)
When acceleration is available: `g_eff = g_world - a_world`.
Project CoM along `g_eff`. Same margin machinery as static SSM.
Ignores external wrenches in the projection.

### D. Stability moment balance (primary dynamic)
Edge restoring moments from:
- `m g` at CoM
- `-m a` at CoM (if acceleration available)
- external forces with known application points
- external free torques

Normalized by `M_ref_i = m g * reference_margin_i`.
`critical_edge` / `minimum_normalized_moment_edge` uses the minimum
**normalized** moment (not raw N·m).

Point-mass translational model only. No `-Iα` / `-ω×Iω`.

### E. Point-mass / translational ZMP (optional diagnostic)
Support-plane ZMP from the non-contact wrench (`x=-My/Fz`, `y=Mx/Fz`).
Useful for visualization and cross-checks with Stability Moment.
Invalid when `|Fz|` is too small.

## Edge selection note

| Selector | Criterion |
|---|---|
| `nearest_*_edge` | Minimum **raw** signed support margin |
| `minimum_normalized_moment_edge` | Minimum **normalized** restoring moment |

These are not guaranteed to match when longitudinal vs lateral reference margins differ.

## Not implemented

| Metric | Why |
|---|---|
| Canonical FASM | No project-owned canonical formulation yet |
| Full rigid-body ZMP | No inertia tensor / rotational dynamics |
| LTR | No left/right contact-load model |
| Multibody / suspension / terramechanics | Out of scope |
| Decision severity / Stop-Go | Evidence only |

## ROS publication

Python may emit more fields than `RolloverStep.msg`.

**TODO:** after validation, decide which dynamic evidence fields should be
exposed through ROS. Do not expand ROS messages in this phase.
