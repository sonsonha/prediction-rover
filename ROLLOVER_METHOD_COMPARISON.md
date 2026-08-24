# Rollover Method Comparison (Pure Python)

No new algorithms. No ROS work.

Pytest: **119 passed in 0.12s**
Scenarios: **22**

## 1. Current implemented methods

1. Critical geometric tip angle
2. Static SSM
3. Normalized Static SSM
4. Effective-gravity / inertial SSM
5. Stability Moment / Moment Balance
6. Point-mass / translational ZMP

Not implemented: canonical FASM, full rigid-body ZMP, LTR, multibody, Decision thresholds.

## 2. What each method actually calculates

| Method | Calculates | Answers |
|---|---|---|
| Critical tip angle | atan(reference_margin_i / com_height) on flat configured support | Ideal geometric slope putting CoM ray on an edge for this chassis |
| Static SSM | Gravity projection of CoM onto support; min signed edge margin | Remaining geometric margin for this terrain/heading |
| Normalized Static SSM | Edge margin / flat reference margin; take min | Dimensionless static margin vs flat design margins |
| Effective SSM | Same as static along g_eff = g - a | How translational acceleration moves the support intercept |
| Point-mass ZMP | Support point from non-contact wrench (x=-My/Fz, y=Mx/Fz) | Resultant-force contact location under point-mass assumptions |
| Stability Moment | Restoring moment about each edge from gravity, -ma, wrenches | Moment budget to tip; critical by min normalized moment |

Verified live formulas from `prediction_core/rollover.py`.

## 3. Required inputs

| Method | Terrain+pose | Config | Accel | External wrench |
|---|---|---|---|---|
| Tip angle | no | yes | no | no |
| Static/normalized SSM | yes | yes | no | no |
| Effective SSM | yes | yes | required (None => unavailable) | ignored for projection |
| Point-mass ZMP | yes | yes | required | used when list provided |
| Stability Moment | yes | yes | required for dynamic package | used when list provided |

`acceleration_xyz=None` is not `(0,0,0)`.
`external_wrenches=None` is not `[]`.

## 4. Static cases

Flat tip angles: front/rear ~ **48.65 deg**, left/right ~ **53.13 deg**,
critical tip edge `front`.

Tip angles do **not** change with terrain or acceleration (chassis property).

Flat static SSM = **0.375000 m**, normalized = **1.000000**.
On slopes with a=(0,0,0) and empty wrenches, effective SSM matches static SSM.

Normalized moment on flat static is ~1. On slopes, normalized moment need not equal normalized SSM
because M_ref = m g m_ref uses flat reference margins while restoring moment depends on support-frame forces.

## 5. Acceleration cases

On flat ground, static SSM stays constant while effective SSM and ZMP margin fall with lateral accel
(`accel_sweep_static_constant=True`).

Primary justification (`flat_lateral_accel_4`):
- normalized static SSM = **1.0000** (unchanged vs flat)
- normalized effective SSM = **0.6941**
- static SSM = 0.3750 m, effective SSM = 0.3054 m

Combined slope+accel (`side_slope_plus_lateral_accel`):
- normalized static = **0.8678**
- normalized effective = **0.7042**
- static SSM = 0.3750 m, effective SSM = 0.3098 m

Beyond tip (`dynamic_beyond_tip`): ZMP margin = -0.2330 m,
min moment = -228.51 N*m.

## 6. External force / torque cases

Same force, low vs high application point:
- low: ZMP=0.3750 m, moment=418.99 N*m
- high: ZMP=0.2106 m, moment=206.49 N*m
- Effective SSM unchanged at 0.3750 m for both
  (`force_height_effective_ssm_constant=True`)

Pure torque: static/effective SSM stay flat-static while ZMP/moment move
(`zmp_xy=[0.0, -0.08157729703823426]`).

None vs empty wrench: numeric margins match when unloaded, but
`external_wrench_included` is False for None and True for [].

## 7. Mathematical redundancy / equivalence analysis

### Effective SSM vs point-mass ZMP
Under gravity + translational acceleration with empty wrenches, Effective SSM and point-mass ZMP encode the same resultant line of action (max |dxy|=5.551e-17 m, max |dmargin|=5.551e-17 m; equivalent=True). They are largely redundant in the accel-only regime.

With external wrenches (5 cases), Effective SSM vs ZMP max |margin diff|=0.16443614792003386; diverges=True.

### ZMP vs Stability Moment
Under the point-mass ZMP definition, M_edge ~= (-Fz_support) * signed_ZMP_distance_to_edge (max |residual| wrench-free=5.684e-14 N*m; pure-torque residual=0.000e+00 N*m). Holds by construction once ZMP includes all support-plane moments.

Edge disagreements: **11** scenarios (see JSON).
Moment critical edge uses min **normalized** moment; SSM/ZMP use raw min margin.

## 8. Advantages and limitations

| Method | Advantage | Limitation |
|---|---|---|
| Tip angle | Design intuition | Not a route-step metric |
| Static SSM | Cheap continuous terrain margin | Blind to accel/loads |
| Effective SSM | Accel effect without wrench plumbing | Ignores external wrenches |
| Point-mass ZMP | Accel + wrench-aware contact point | Not rigid-body ZMP |
| Stability Moment | Force height + pure torque; normalized budget | Needs accel; no rotational inertia |

## 9. Classification and recommendation

| Method | Class |
|---|---|
| Critical geometric tip angle | USEFUL DIAGNOSTIC / REDUNDANT for per-step decisions |
| Static SSM | BASELINE |
| Normalized Static SSM | BASELINE |
| Effective-gravity / inertial SSM | KEEP AS DYNAMIC EXTENSION (accel-only) / diagnostic if ZMP kept |
| Point-mass / translational ZMP | KEEP AS DYNAMIC EXTENSION when wrenches matter; redundant with Effective SSM for accel-only |
| Stability Moment / Moment Balance | KEEP AS DYNAMIC EXTENSION (best wrench-aware scalar) |

### Minimal useful stack today

1. Terrain roll/pitch
2. Static SSM + normalized Static SSM
3. Stability Moment as primary wrench-aware dynamic metric
4. Optional ZMP for operator visualization
5. Tip angles as config/diagnostic only
6. Effective SSM optional if wrench topics may be None and you still want accel-only evidence; otherwise redundant with ZMP when wrenches=`[]`

## 10. Missing FASM / full ZMP / LTR

- **FASM**: not implemented; needs agreed canonical formula; overlaps Stability Moment under current point-mass wrench set. Do not label Stability Moment as FASM.
- **Full ZMP**: needs inertia tensor, angular velocity/acceleration, rotational inertial terms.
- **LTR**: needs left/right vertical contact loads or a validated estimator; not available now.

## 11. Landfill Rover recommendation

Keep static baseline. Add one wrench-aware dynamic metric (Stability Moment recommended; ZMP optional visual).
Treat Effective SSM as accel-only diagnostic. Publish tip angles as vehicle properties, not per-step alarms.
Defer FASM/LTR/full ZMP until required inputs and formulas exist.

## Artifacts

- `outputs/rollover_method_comparison.json`
- Plots:
  - `outputs/rollover_method_comparison/plots/plot1_lateral_accel_margins.png`
  - `outputs/rollover_method_comparison/plots/plot2_lateral_accel_norm_moment.png`
  - `outputs/rollover_method_comparison/plots/plot3_force_height_zmp_moment.png`
  - `outputs/rollover_method_comparison/plots/plot4_support_snapshot_combined.png`
