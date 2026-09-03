# Partial Geometry Semantics

**Branch:** `review/partial-geometry-coverage`  
**Scope:** Geometry adapter publish policy only — not terrain physics, Decision, or Prediction math.

---

## Summary

A trajectory may request **N** future steps. The canonical `/geometry`
(`GeometryArray`) may contain only a **subset** of those steps — the ones with
valid terrain normals.

Missing terrain is **UNKNOWN**, not flat ground.

---

## Contracts

| Topic | Role |
|-------|------|
| `/trajectory` | Full planned/future pose sequence (`step_id` 0..N-1 typically) |
| `/geometry` | Terrain normal evidence for **supported** `step_id`s only |
| `/tracked_objects` | Collision inputs (independent of terrain geometry) |
| `/rover/state` | Required for `prediction_profile:=dynamic` |

### Identity rules

1. `GeometryArray.source_trajectory_id` must equal `Trajectory.trajectory_id`.
2. Each `GeometryStep.step_id` is the **original** trajectory `step_id`.
3. Step IDs are **not** renumbered when some steps are omitted.
4. Prediction joins geometry ↔ trajectory by **`step_id`**, not array index.

Example:

```text
trajectory step_ids:  0 1 2 3 4 5
valid geometry:       0 1   3   5
GeometryArray.steps:  0,1,3,5   (not 0,1,2,3)
```

---

## Adapter behavior (`allow_flat_fallback=false`, default)

For each trajectory step:

1. Look up terrain normal in GridMap.
2. If missing → **omit** that step (`continue`); do not abort the cycle.
3. If present → append a `GeometryStep` with that `step_id` and normal.

After all steps:

- If **≥1** valid step → publish `GeometryArray` with only those steps.
- If **0** valid steps → **do not publish** (no empty success message).

When `allow_flat_fallback=true` (smoke only): missing normals become upright
`(0,0,1)` with `plane_id=flat-fallback-{step_id}`. Production integration keeps
fallback **off**.

---

## Prediction behavior

Default config: `require_full_geometry_coverage: false`.

| Channel | Sparse geometry effect |
|---------|------------------------|
| **Rollover** | Evaluates **only** steps with matching geometry `step_id`. Missing IDs are skipped and logged (`last_missing_step_ids`). |
| **Collision** | Runs on **all** trajectory steps from objects; **independent** of terrain geometry coverage. |
| **Readiness** | Needs a non-empty geometry batch whose `step_id`s intersect the trajectory; full coverage is **not** required. |

Partial publish does **not** claim full terrain coverage of the requested horizon.

Continuous useful terrain horizon is the longest prefix of consecutive valid
geometry steps from the rover forward (e.g. steps 0–11 valid → ≈ 2.75 s at
dt = 0.25 s), not the last isolated valid cell.

---

## What this change does / does not do

**Does**

- Preserve early valid terrain evidence when a later step misses a normal.
- Match upstream partial-`GeometryArray` publish policy.

**Does not**

- Change terrain estimator thresholds, plane fitting, or normal math.
- Change GridMap indexing, TF / map-cloud paths, or GridMap QoS.
- Change Prediction physics or Decision policy.
- Assert that terrain normals are physically correct (still pending validation).
- Alter ROS message definitions (no schema change).

---

## Validation notes

Old bags recorded under the aborting adapter cannot reconstruct omitted GridMap
normals. They must not be used to claim improved cell-level terrain coverage.

Use synthetic fixtures (e.g. 20 trajectory steps, geometry on `0..11`) to prove
Prediction accepts sparse `GeometryArray` by `step_id`.

Live SVO re-measure is a separate task after `/clock` replay is restored.

---

## Safety wording

Partial geometry means **incomplete terrain evidence**, not “safe flat ground”
for missing steps. Product stop/go policy is unchanged by this document.
