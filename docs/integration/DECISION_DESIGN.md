# Decision Module Design

Design / requirements audit for a downstream **Decision** layer on the Landfill
Rover Safety Prediction system.

**Status:** **Decision V0 IMPLEMENTED** (evidence-only). **Decision V1 NOT IMPLEMENTED** (no safety policy).  
**Repository HEAD audited:** `integration/humble-real-pipeline` @ `5a06906`  
**Decision V0 branch:** `feature/decision-evidence-v0`  
**Scope:** audit existing Prediction evidence and gaps; **no** physics, runtime,
adapter, message, or threshold changes in this milestone.

Physical terrain/rollover correctness: **PENDING** (diagnostic validity ≠ physical
validation). Decision must not treat current rollover metrics as field-proven.

---

## 1. Purpose

Prediction Python V1 and `prediction_ros` produce **safety evidence** — collision
candidates, terrain attitude, static/dynamic rollover diagnostics — for a
downstream **Decision** module.

Decision’s job (once requirements exist) is to interpret that evidence into
operator/vehicle-appropriate safety outcomes (status, commands, or alerts).

This document records:

- what evidence already exists in `PredictionOutput`
- what Decision **can** consume today
- what product/safety policy is **still undefined**
- a minimal proposed interface and architecture **before coding**

Explicit non-goals for this audit:

- no SAFE / WARNING / DANGER / STOP / SLOW assignment
- no new risk thresholds
- no vehicle controller interface

---

## 2. Current System Boundary

```text
Perception / adapters
  /trajectory
  /tracked_objects
  /geometry
  /rover/state
  /external_wrenches   (optional)
        │
        ▼
PredictionRuntime + PredictionCore
  (at most once per trajectory_id)
        │
        ▼
  /predict_output      ← evidence only
        │
        ▼   (NOT IMPLEMENTED)
    DecisionNode
        │
        ▼
  /decision            ← proposed; undefined today
```

Validated today (integration branch):

| Layer | Status |
|-------|--------|
| Static Prediction live | PASS |
| Dynamic Prediction live E2E | PASS |
| Dynamic replay fixture | PASS |
| Prediction RViz visualization | IMPLEMENTED + replay validated |
| Physical terrain correctness | PENDING |
| Physical rollover correctness | PENDING |
| **Decision** | **V0 IMPLEMENTED** (evidence-only on `/decision/evidence`) · **V1 NOT IMPLEMENTED** (no SAFE/STOP policy) |

Sources: `docs/integration/VALIDATION_STATUS.md`, `README.md`,
`documents/PREDICTION_PYTHON_V1.md`, `documents/ROS_UPSTREAM_INTERFACE_CONTRACT.md`.

---

## 3. Existing Decision Requirements Found

### 3.1 Code search result

Repository search for `decision`, `decision_node`, `severity`, `STOP`, `SAFE`,
`WARNING`, `unsafe`, etc. found:

| Finding | Location |
|---------|----------|
| **No Decision implementation** | No `*decision*` package, node, or `.msg` |
| **No `Decision*.msg`** | `safety_perception_msgs` has 16 messages; none are Decision |
| Explicit “evidence only” | `README.md`, `PREDICTION_PYTHON_V1.md`, `ROS_UPSTREAM_INTERFACE_CONTRACT.md` |
| `decision_status: intentionally not computed` | `prediction_core/cli.py`, `visualization/export_demo.py` |
| Decision node “not implemented” | `docs/integration/KNOWN_LIMITATIONS.md`, `VALIDATION_STATUS.md` |
| Visualization must not invent severity | `docs/integration/PREDICTION_VISUALIZATION_DESIGN.md` |
| `critical_edge` naming | Physics/diagnostic naming — **not** a Decision state |

### 3.2 Explicit architectural intent (existing)

From `documents/PREDICTION_PYTHON_V1.md` §1 and §14:

> Prediction produces **safety evidence** for a downstream Decision Node …  
> It does **not** assign severity, safe/unsafe labels, Stop/Go, or thresholds.

From `documents/ROS_UPSTREAM_INTERFACE_CONTRACT.md`:

> `/predict_output` publishes physical evidence only — **no** severity, safe/unsafe, Stop/Go.

From `documents/DYNAMIC_ROLLOVER_METRICS.md`:

> Decision severity / Stop-Go | Evidence only

### 3.3 Thresholds that exist today (Prediction-internal, not Decision)

| Parameter | Where | Role |
|-----------|-------|------|
| `prediction.collision_margin_m` | `config/rover.mock.yaml` (default **0.20 m**) | **Prediction-internal** proximity gate for collision **candidates** |
| Support rectangle / CoM geometry | `RoverConfig` | Rollover baseline geometry |
| Normalized SSM reference margins | `RoverConfig` | Normalization reference for static SSM |

**None of these are documented as Decision policy thresholds.**  
Decision must **not** silently adopt `collision_margin_m` as a STOP distance without
explicit product/safety agreement.

### 3.4 Conclusion

**No existing Decision contract, message, node, or externally approved safety
states were found.** Only directional documentation: Prediction → evidence;
Decision → future interpretation.

---

## 4. Exact PredictionOutput Evidence

### 4.1 ROS message: `safety_perception_msgs/msg/PredictionOutput`

```text
std_msgs/Header header
uint64 source_trajectory_id
CollisionStep[] collision_steps
RolloverStep[] rollover_steps
```

`source_trajectory_id` is set from the active `Trajectory.trajectory_id` at
publish time (`prediction_node.py` → `RosAdapters.prediction_to_ros`).

### 4.2 `CollisionStep`

```text
uint32 step_id
float64 distance_along_route_m
CollisionObject[] collision_objects
```

### 4.3 `CollisionObject`

```text
uint64 track_id
string object_class
float64 min_distance_m
float32 confidence
bool confidence_valid
```

### 4.4 `RolloverStep`

```text
uint32 step_id
float64 predicted_roll_deg
float64 predicted_pitch_deg
float64 static_stability_margin_m
float64 normalized_static_stability_margin
StabilityMomentEvidence stability_moment
ZmpEvidence zmp
string terrain_id
float32 confidence
bool confidence_valid
```

### 4.5 `StabilityMomentEvidence`

```text
bool valid
string validity_reason
float64 front_moment_nm / rear / left / right
float64 normalized_front_moment / rear / left / right
float64 minimum_stability_moment_nm
float64 normalized_minimum_stability_moment
string minimum_normalized_moment_edge
bool acceleration_available
bool external_wrench_available
bool external_wrench_included
```

### 4.6 `ZmpEvidence`

```text
bool valid
float64 x
float64 y
float64 margin_m
float64 normalized_margin
string nearest_edge
```

`x` / `y` are **rover support-plane / body-local** coordinates (+X forward,
+Y left), **not map**. Map placement requires the matching trajectory step pose
(see `docs/integration/PREDICTION_VISUALIZATION_DESIGN.md`).

### 4.7 Field classification

| Field | Class | Notes |
|-------|-------|-------|
| `header` | D | publish stamp / frame context |
| `source_trajectory_id` | D | cycle binding |
| `collision_steps[]` | A | primary collision evidence (sparse) |
| `CollisionStep.step_id` | D | join key to trajectory step |
| `CollisionStep.distance_along_route_m` | D | arc-length along route |
| `CollisionObject.min_distance_m` | A | footprint-to-footprint distance at step |
| `CollisionObject.track_id`, `object_class` | A / D | object identity |
| `CollisionObject.confidence*` | C | optional detection confidence |
| `rollover_steps[]` | A | per-step rollover evidence (sparse vs trajectory) |
| `predicted_roll_deg`, `predicted_pitch_deg` | A | terrain attitude baseline |
| `static_stability_margin_m` | A | raw static SSM (signed margin) |
| `normalized_static_stability_margin` | A | normalized static SSM |
| `stability_moment.*` | A (dynamic) / C (`valid`, flags, `validity_reason`) | primary dynamic when `valid=true` |
| `zmp.*` | B (diagnostic) / C (`valid`) | optional diagnostic; local XY |
| `terrain_id`, `confidence*` | C / D | terrain step metadata |

**Legend:** A = primary safety evidence · B = diagnostic · C = validity/confidence · D = identifier/spatial metadata

### 4.8 Python-only evidence NOT in ROS `PredictionOutput`

These exist in `prediction_core` but are **not** on `/predict_output` today:

| Evidence | Notes |
|----------|-------|
| `CriticalTipEvidence` (`critical_tip`) | Secondary chassis diagnostic; “not Decision thresholds” (`models.py`) |
| `effective_ssm_m`, `effective_gravity_projection_xy` | Secondary dynamic diagnostic |
| Per-edge `nearest_static_edge` | Present in Python `RolloverStep`; **not** in ROS msg |
| Full `edge_stability_moments_nm` dict | Partially mapped into `StabilityMomentEvidence` |

Decision design should assume **ROS `/predict_output` is the integration
contract**, not full Python JSON dumps.

---

## 5. Collision Evidence Semantics

Source: `prediction_core/collision.py`, `PREDICTION_PYTHON_V1.md` §5.

### 5.1 What `collision_steps[]` means

**Not** “confirmed geometric intersection” and **not** “time-to-collision.”

It means: at discrete trajectory pose `step_id`, the rover **body rectangle**
(minimum Shapely distance to each tracked object polygon) satisfies:

```text
min_distance_m ≤ collision_margin_m   (Prediction config, default 0.20 m)
```

Steps with **no** object within margin are **omitted** from `collision_steps[]`.
An empty `collision_steps[]` means **no candidate steps**, not “proven safe.”

### 5.2 Non-collision steps

**Omitted.** Decision cannot infer “clear” per step from absence alone without
knowing the full trajectory horizon policy.

### 5.3 `distance_along_route_m`

Cumulative arc length along the trajectory polyline from the first step to this
`step_id` (`trajectory.cumulative_distances_m()`). Useful for horizon / ordering;
**not** time.

### 5.4 Object information

Per candidate object: `track_id`, `object_class`, `min_distance_m`,
optional `confidence` / `confidence_valid`.

### 5.5 Not available in PredictionOutput

| Quantity | Status |
|----------|--------|
| Time-to-collision (TTC) | **Not computed** |
| Object velocity / swept path | **Not used** (V1 limitation) |
| Confirmed impact / penetration depth | **Not computed** |
| Separate boolean “collision=true” | **Not present** — presence of `CollisionStep` is the signal |
| Decision severity | **Not present** |

### 5.6 Can current output support STOP vs WARNING vs informational?

**Not without new assumptions.** Existing evidence supports statements like:

- “collision **candidates** exist at step(s) X with object(s) Y at distance Z”
- “no collision candidates were reported for this prediction cycle”

It does **not** define:

- whether any candidate ⇒ STOP
- whether distance bands ⇒ SLOW vs STOP
- which horizon (nearest step only vs any step along route) matters

**Missing requirement/data for Decision:** product rules mapping candidate
proximity / step horizon to discrete actions.

---

## 6. Rollover Evidence Semantics

Sources: `prediction_core/rollover.py`, `documents/DYNAMIC_ROLLOVER_METRICS.md`,
`ROS_UPSTREAM_INTERFACE_CONTRACT.md`.

### 6.1 What `rollover_steps[]` contains

One entry per trajectory step that has **matching geometry** for `step_id`.
Steps **without** geometry are **skipped** (logged warning; not present in output).

Each step includes **baseline** roll/pitch + static SSM always (when geometry exists).

### 6.2 Metric hierarchy (documented)

| Metric | Role in docs |
|--------|----------------|
| `predicted_roll_deg` / `predicted_pitch_deg` | PRIMARY BASELINE |
| `static_stability_margin_m` | PRIMARY BASELINE (signed raw margin) |
| `normalized_static_stability_margin` | PRIMARY BASELINE |
| `stability_moment` (when `valid=true`) | PRIMARY DYNAMIC |
| `zmp` (when `valid=true`) | OPTIONAL DIAGNOSTIC |
| `critical_tip` (Python only) | SECONDARY DIAGNOSTIC |

### 6.3 Explicit unsafe/safe results?

**None.** Numeric margins and moments are continuous diagnostics.

Examples:

- `static_stability_margin_m < 0` ⇒ geometric support margin negative (past edge
  in the quasi-static model) — **not** labeled “unsafe” in messages
- `normalized_static_stability_margin` can be `< 0` (tests preserve negative values)
- `stability_moment.valid=false` ⇒ dynamic evidence **unavailable** (not zero risk)
- `zmp.valid=false` ⇒ ZMP **undefined** (not zero risk)

### 6.4 Threshold dependence

Any discrete rollover Decision (e.g. SLOW/STOP) requires **external thresholds**
on one or more of:

- raw or normalized static SSM
- minimum stability moment (raw or normalized)
- ZMP margin (diagnostic)
- roll/pitch limits

**No such thresholds are defined in repository requirements.**

### 6.5 Physical validation caveat

Integration validation confirms **diagnostic flags populate** on dynamic path
(`acceleration_available`, `stability_moment.valid`, `zmp.valid`).  
**Physical terrain/rollover correctness is PENDING.** Decision design must treat
rollover numbers as **model outputs**, not field-calibrated safety limits.

### 6.6 `critical_tip` / effective SSM

Not published on `/predict_output`. Decision cannot rely on them without ROS
schema extension (out of scope for this audit).

---

## 7. Valid / Invalid / Unknown Semantics

Decision must distinguish **SAFE** (if ever defined) from **UNKNOWN /
INSUFFICIENT EVIDENCE**.

### 7.1 Validity signals in PredictionOutput

| Condition | Interpretation for Decision |
|-----------|----------------------------|
| No `/predict_output` received for active cycle | **INSUFFICIENT EVIDENCE** (Prediction not ready or not run) |
| `source_trajectory_id` ≠ active trajectory | **STALE / MISMATCH** — do not apply to current cycle |
| `collision_steps[]` empty | **No collision candidates reported** — ≠ proven safe |
| `rollover_steps[]` empty | **No rollover evidence** (e.g. missing geometry alignment) — ≠ proven safe |
| `stability_moment.valid=false` | Dynamic stability moment **unavailable** — do not read numeric moments as evidence |
| `zmp.valid=false` | ZMP **unavailable** — do not use `x/y/margin_m` |
| `confidence_valid=false` on rollover step | Terrain confidence unknown — policy TBD |
| Invalid / non-finite numeric fields | Should not occur from validated Prediction; treat as **fault** if seen |

### 7.2 Invalid diagnostics are not zero risk

Repository rule (`ROS_UPSTREAM_INTERFACE_CONTRACT.md`):

> `valid=false` on nested dynamic messages means unavailable — do not treat numeric zeros as evidence.

Decision must **not** map `valid=false` → SAFE.

### 7.3 Proposed evidence sufficiency states (neutral — not product SAFE)

Until product defines SAFE/STOP, Decision should expose **evidence availability**
only:

| State | Meaning |
|-------|---------|
| `NO_PREDICTION` | No output for active trajectory cycle |
| `PREDICTION_STALE` | Latest output references older `source_trajectory_id` |
| `PREDICTION_CURRENT` | Output matches active `source_trajectory_id` |

Within `PREDICTION_CURRENT`, separate **evidence flags** (not actions):

- `collision_candidates_present`
- `rollover_baseline_present`
- `dynamic_stability_moment_valid` (any step)
- `zmp_valid` (any step)

Final SAFE vs UNKNOWN vs CAUTION policy: **OPEN** (§9).

---

## 8. Cycle and Staleness Semantics

Sources: `prediction_core/runtime.py`, `prediction_core/coordinator.py`,
`ROS_UPSTREAM_INTERFACE_CONTRACT.md`.

### 8.1 Prediction cadence

- **At most one** `/predict_output` per `source_trajectory_id` (duplicate cycles
  suppressed; logged as “duplicate cycle”).
- New `trajectory_id` clears cycle-bound objects/geometry/state/wrenches in
  PredictionRuntime.

### 8.2 Decision implications

| Event | Decision behavior (proposed) |
|-------|------------------------------|
| New trajectory, no Prediction yet | Emit `NO_PREDICTION` / hold previous decision only if policy allows brief grace — **default: do not extend old safety decision** |
| Prediction arrives for trajectory T | Bind decision to `source_trajectory_id=T`; process once |
| Duplicate Prediction for same T | Ignore duplicate (Prediction already suppresses re-publish) |
| Trajectory advances to T+1 | Prior output is **stale** until new Prediction for T+1 |
| Prediction unavailable (readiness wait) | `NO_PREDICTION` — not SAFE |

### 8.3 Do not hold stale safety decisions indefinitely

If product later defines STOP, Decision must expire or downgrade when:

- trajectory ID changes
- prediction age exceeds policy (optional; not defined today)
- required dynamic validity is lost on profile switch

### 8.4 Should Decision subscribe to `/trajectory`?

**Recommended minimal inputs:**

| Input | Required? | Reason |
|-------|-----------|--------|
| `/predict_output` | **Yes** | Primary evidence |
| `/trajectory` (ID only) | **Recommended** | Detect active cycle vs stale output without inferring from Prediction header alone |

**Not required** if Decision maintains trajectory subscription solely to read
`trajectory_id` for staleness checks. Full trajectory geometry is **not** needed
if Decision does not recompute Prediction.

**Avoid** subscribing to `/tracked_objects`, `/geometry`, `/rover/state` unless
a future requirement needs pre-Prediction gating — that duplicates Prediction’s
readiness role.

---

## 9. Proposed Decision Interface (minimal, pre-approval)

**Do not implement yet.** Proposed new message in `safety_perception_msgs` (name
TBD). Fields derived from gaps above — **not** final product states.

### 9.1 Proposed `DecisionOutput` (draft)

```text
std_msgs/Header header

uint64 source_trajectory_id          # must match PredictionOutput + active trajectory
builtin_interfaces/Time prediction_stamp   # copy PredictionOutput.header.stamp

uint8 evidence_state
  # NO_PREDICTION=0
  # PREDICTION_STALE=1
  # PREDICTION_CURRENT=2

bool collision_candidates_present
bool rollover_baseline_present
bool dynamic_stability_moment_valid   # any rollover step
bool zmp_valid                        # any rollover step

uint32 earliest_collision_step_id     # optional; 0xFFFFFFFF = none
float64 earliest_collision_distance_m # min distance_along_route_m among collision steps; NaN if none
float64 minimum_collision_distance_m  # min min_distance_m across all candidates; NaN if none

uint32 minimum_ssm_step_id            # step with min static_stability_margin_m; optional
float64 minimum_static_stability_margin_m
float64 minimum_normalized_static_stability_margin

string summary_reason                 # human-readable audit string, not authority
```

### 9.2 Field justification

| Field | Why needed |
|-------|------------|
| `source_trajectory_id` | Cycle binding; reject stale Prediction |
| `prediction_stamp` | Audit / optional age policy |
| `evidence_state` | Distinguish UNKNOWN vs current evidence without claiming SAFE |
| `collision_candidates_present` | Primary collision signal available |
| `rollover_baseline_present` | Baseline rollover evidence available |
| `dynamic_stability_moment_valid` | Dynamic profile evidence gate |
| `zmp_valid` | Diagnostic availability (optional downstream use) |
| Earliest collision step / distances | Horizon ordering for future policy |
| Minimum SSM fields | Common rollover summary candidates — **threshold TBD** |
| `summary_reason` | Operator/debug; not normative |

### 9.3 Explicitly omitted until requirements exist

- `SAFE` / `CAUTION` / `SLOW` / `STOP` enums
- `decision_command` to controller
- Priority arbitration between collision vs rollover
- Threshold parameters in message

---

## 10. Proposed Architecture

```text
/predict_output  ──►  DecisionNode  ──►  /decision_output
       ▲                      ▲
       │                      │
  Prediction            /trajectory (ID tracking recommended)
```

Future (out of scope):

```text
/decision_output  ──►  decision_visualization  ──►  /decision_viz/*
/decision_output  ──►  vehicle safety interface (NOT NOW)
```

Rules:

- Decision **must not** re-run collision/rollover math
- Decision **must not** modify Prediction inputs
- RViz for Decision should be separate from `prediction_visualization`

---

## 11. Requirements Still Missing

| Gap | Impact |
|-----|--------|
| No approved discrete safety states | Cannot implement STOP/SLOW/SAFE |
| No collision candidate → action mapping | Proximity evidence exists; policy does not |
| No rollover metric authority | SSM vs stability moment vs ZMP — which drives Decision? |
| No rollover thresholds | Continuous metrics only |
| No horizon policy | Nearest step vs any-step along route |
| No collision–rollover priority | Simultaneous evidence handling undefined |
| No UNKNOWN vs SAFE policy when Prediction waits | Runtime can publish nothing for a cycle |
| No controller interface contract | Command vs status-only Decision |
| Physical validation of rollover/terrain | Metrics may not be field-trustworthy yet |
| Track ID persistence semantics | Objects may be frame-local hashes (`KNOWN_LIMITATIONS.md`) |

---

## 12. Questions for Team / Client

### Collision

1. Does **any** collision **candidate** within `collision_margin_m` imply STOP, or only below a smaller distance?
2. Is `collision_margin_m` (Prediction config, 0.20 m default) acceptable as the **detection** margin, or should Detection and Decision margins differ?
3. Which horizon matters: first candidate along route, any candidate, or only within N metres?
4. Should empty `collision_steps[]` ever be interpreted as “path clear,” or only “no candidates reported this cycle”?
5. Is object `confidence` required to gate decisions?

### Rollover

6. Which metric is **authoritative** for rollover Decision: static SSM, normalized SSM, stability moment, or ZMP?
7. What thresholds (if any) on each metric for CAUTION vs STOP?
8. Is negative `static_stability_margin_m` treated as rollover imminent in product policy?
9. When `stability_moment.valid=false` on dynamic profile, is Decision UNKNOWN, or may static SSM alone drive action?
10. How should missing geometry steps (rollover sparse vs trajectory) be treated?

### General / safety

11. Define **SAFE** vs **UNKNOWN** when Prediction has not completed for the active trajectory.
12. How long (if at all) may Decision retain a prior cycle’s output after trajectory ID changes?
13. If collision and rollover evidence conflict, which takes priority?
14. Should Decision output a **discrete command** to the controller, or only a **safety status** for a human/autonomy supervisor?
15. Is Decision allowed to command motion directly, or only advise upstream planning?
16. What traceability / logging is required for regulatory audit?

---

## 13. Test Plan (deterministic — no STOP/SAFE expectations yet)

Test pure Decision logic against **fixture `PredictionOutput` messages** (mock or
bag-derived), without RViz or SVO.

| Case | Input | Expected (evidence-level only) |
|------|-------|----------------------------------|
| T1 | No PredictionOutput, trajectory T active | `evidence_state=NO_PREDICTION` |
| T2 | Prediction for T, empty collision & rollover arrays | `PREDICTION_CURRENT`, flags false |
| T3 | Collision candidates only | `collision_candidates_present=true`, earliest step/distance populated |
| T4 | Rollover baseline only | `rollover_baseline_present=true`, SSM summary populated |
| T5 | Both collision + rollover | both flags true |
| T6 | `stability_moment.valid=false` everywhere | `dynamic_stability_moment_valid=false` |
| T7 | `zmp.valid=false` everywhere | `zmp_valid=false` |
| T8 | Prediction for T-1, active trajectory T | `PREDICTION_STALE` |
| T9 | Duplicate Prediction same ID | single Decision emission (idempotent) |
| T10 | Trajectory replaces before Prediction | `NO_PREDICTION` then fresh output |
| T11 | Non-finite / malformed numeric | fault / reject message |
| T12 | Boundary values | **deferred** until thresholds defined |

Replay fixture reference: `session_0924_dynamic_prediction_inputs` (IDs **1**, **11**).

---

## 14. Implementation Plan After Approval

1. **Product/safety workshop** — resolve §12 questions; approve discrete states (if any).
2. **Message design review** — finalize `DecisionOutput` fields; add to `safety_perception_msgs`.
3. **`decision_ros` package** (or equivalent) — subscribe `/predict_output` + `/trajectory`; pure `decision_logic.py` + thin node.
4. **Unit tests** — §13 table; no STOP/SAFE assertions until policy lands.
5. **Fixture replay validation** — Decision + Prediction + bag on isolated domain (mirror `run_prediction_viz.sh` pattern).
6. **Decision RViz** — separate viz package/config (do not overload `prediction_visualization`).
7. **Documentation** — update `VALIDATION_STATUS.md` when evidence-level Decision is validated; keep physical correctness PENDING.
8. **Controller interface** — only after explicit safety review.

**Do not start steps 2–8 until step 1 produces written requirements.**

---

## 15. Decision V0 Implementation (evidence-only)

**Branch:** `feature/decision-evidence-v0`  
**Package:** `ros2/decision_ros`  
**Message:** `safety_perception_msgs/msg/DecisionEvidence.msg`  
**Topic:** `/decision/evidence` (only — no `/decision`, `/vehicle/command`, `/stop`)

### 15.1 `DecisionEvidence` fields

```text
std_msgs/Header header
string source_trajectory_id
uint8 evidence_state          # NO_PREDICTION=0, PREDICTION_STALE=1, PREDICTION_CURRENT=2
bool collision_candidates_present
bool rollover_baseline_present
bool dynamic_stability_moment_valid
bool zmp_valid
bool nearest_collision_distance_valid
float64 nearest_collision_distance_m
bool minimum_normalized_ssm_valid
float64 minimum_normalized_static_stability_margin
bool minimum_stability_moment_valid
float64 minimum_stability_moment_nm
bool minimum_zmp_margin_valid
float64 minimum_zmp_margin_m
```

### 15.2 `evidence_state` semantics

| Constant | Value | Meaning |
|----------|------:|---------|
| `NO_PREDICTION` | 0 | Active trajectory has no matching `PredictionOutput` yet |
| `PREDICTION_STALE` | 1 | Latest `PredictionOutput.source_trajectory_id` ≠ active `trajectory_id` |
| `PREDICTION_CURRENT` | 2 | `PredictionOutput` matches active trajectory |

When state is not `PREDICTION_CURRENT`, evidence flags and aggregates are cleared
(not interpreted as safe/unknown risk).

### 15.3 Inputs / non-goals

- Subscribes: `/trajectory`, `/predict_output` only
- Does **not** subscribe to `/tracked_objects`, `/geometry`, `/rover/state`
- Does **not** re-run Prediction physics
- Does **not** assign SAFE / WARNING / STOP / controller commands

### 15.4 Decision V1 production policy (future)

Requires approved product/safety requirements (§11–§12): discrete states, collision
and rollover thresholds, horizon policy, collision–rollover priority, controller
interface. **Not implemented** as production safety policy.

### 15.5 Controller integration

**Not implemented.** Decision V0 publishes evidence only.

---

## 16. Decision Prototype V1 (STOP/GO)

**Branch:** `feature/decision-stop-go-v1`  
**Message:** `safety_perception_msgs/msg/DecisionOutput.msg`  
**Topic:** `/decision` (prototype policy output only — **no controller connection**)

### 16.1 Architecture

```text
Prediction → /predict_output → DecisionEvidence (V0) → /decision/evidence
                                                      ↓
                                            DecisionPolicy V1 → /decision
```

Decision V0 semantics remain unchanged. Policy consumes `/decision/evidence` only.

### 16.2 `DecisionOutput` fields

```text
std_msgs/Header header
string source_trajectory_id
uint8 decision                 # GO=0, STOP=1
uint8 reason
  # CURRENT_CLEAR=0
  # NO_CURRENT_PREDICTION=1
  # PREDICTION_STALE=2
  # COLLISION_CANDIDATE=3
  # ROLLOVER_EVIDENCE_INVALID=4
  # ROLLOVER_POLICY_TRIGGERED=5
bool prototype_policy
```

### 16.3 PROTOTYPE FAIL-SAFE ASSUMPTION

V1 allows only **GO** and **STOP**. **UNKNOWN / unavailable evidence maps to STOP.**

This is a **prototype integration/demo policy**, not approved production safety logic.

### 16.4 Prototype STOP/GO rules (deterministic)

1. `evidence_state != PREDICTION_CURRENT` → **STOP**
2. `collision_candidates_present == true` → **STOP** (proximity **candidates**, not confirmed impact)
3. Rollover policy enabled but required evidence unavailable → **STOP**
4. Rollover policy enabled and configured threshold triggered → **STOP**
5. Otherwise → **GO**

### 16.5 Rollover policy status

Default config (`ros2/decision_ros/config/decision_policy.yaml`):

```yaml
decision_policy:
  prototype_only: true
  stop_on_collision_candidate: true
  stop_on_missing_current_prediction: true
  rollover_policy:
    enabled: false
    metric: stability_moment
    # threshold: unset (NaN)
```

**Rollover STOP thresholds are NOT approved.** Default rollover policy is **disabled**.
No threshold is silently invented (e.g. `0`). Enabling rollover without a valid finite
threshold fails safe to **STOP** (`ROLLOVER_EVIDENCE_INVALID`).

V1 currently validates Decision plumbing and **collision prototype policy only**.

### 16.6 Non-goals (unchanged)

- No connection to vehicle controller
- No Prediction physics changes
- No WARNING / SLOW / DANGER states
- Physical terrain/rollover validation **PENDING**

### 16.7 Live E2E validation (integration/demo only)

**Evidence:** `prediction/logs/decision_live_e2e_20260901_084141/`  
**ROS_DOMAIN_ID:** 51 · concurrent full stack · **Result: PASS**

| Metric | Value |
|--------|------:|
| PredictionOutput IDs | 7, 12 |
| Decision transitions | 121 |
| GO | 2 |
| STOP | 119 |
| STOP `PREDICTION_STALE` | 112 |
| STOP `NO_CURRENT_PREDICTION` | 7 |

CURRENT evidence → GO / `CURRENT_CLEAR` for both live predictions (no collision
candidates; rollover policy disabled). Stale/no-prediction → STOP with correct reasons.
No contradictory decisions; no application-node crashes. Visualization subscribes to
`/decision`.

**PROTOTYPE ONLY — NOT APPROVED FOR VEHICLE CONTROL.**

Reasons:

- Fail-safe stale → STOP causes near-continuous STOP when trajectory cadence exceeds
  Prediction cadence (~98% STOP transitions; GO windows brief)
- No approved rollover threshold (rollover policy disabled)
- Collision candidate → STOP remains provisional (proximity candidates, not confirmed impact)
- Physical terrain/rollover correctness **PENDING**
- No controller interface (`/decision` is not connected to vehicle commands)

---

## References

| Document | Relevance |
|----------|-----------|
| `documents/PREDICTION_PYTHON_V1.md` | Evidence hierarchy, collision/rollover semantics |
| `documents/ROS_UPSTREAM_INTERFACE_CONTRACT.md` | ROS topics, validity, no Decision in output |
| `documents/DYNAMIC_ROLLOVER_METRICS.md` | Dynamic metric roles |
| `CURRENT_PREDICTION_SOURCE_AUDIT.md` | Field inventory, missing Decision msgs |
| `docs/integration/PREDICTION_VISUALIZATION_DESIGN.md` | ZMP frame, cycle binding |
| `docs/integration/VALIDATION_STATUS.md` | Validated vs PENDING status |
| `prediction_core/collision.py` | Collision candidate definition |
| `prediction_core/rollover.py` | Rollover step construction |
| `config/rover.mock.yaml` | `collision_margin_m` (Prediction-internal) |
