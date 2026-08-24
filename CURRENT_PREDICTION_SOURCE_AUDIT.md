# CURRENT PREDICTION SOURCE AUDIT

**Audit type:** READ-ONLY inspection  
**Audit date:** 2026-08-20  
**Audited tree:** `/Users/sonha/src/landfill-rover-folder/prediction/prediction-src`  
**Rule followed:** No feature work, no refactors, no fixes. Failures reported as-is.

---

# 1. Repository overview

| Item | Value |
|---|---|
| Absolute path | `/Users/sonha/src/landfill-rover-folder/prediction/prediction-src` |
| Git branch | `main` |
| Git status (this repo) | Clean; up to date with `origin/main` |
| HEAD | `420e342` — *Initial commit of landfill rover prediction core and ROS wrapper.* |
| Remote | `https://github.com/sonsonha/prediction-rover.git` |
| Python | 3.11.7 (`.venv`) |
| ROS distro | **Not available** (`ROS_DISTRO` unset; no `/opt/ros`, no `ros2`, no `colcon`) |

**Parent note:** `/Users/sonha/src/landfill-rover-folder/prediction` is a separate git repo and currently has many modified/untracked files, including copies under `prediction-src/`. The nested `prediction-src/.git` repo itself is clean.

## Relevant tree (build/cache pruned)

```text
prediction-src/
├── CURRENT_PREDICTION_SOURCE_AUDIT.md   # this file (created by audit)
├── Makefile
├── README.md
├── pyproject.toml
├── config/
│   ├── rover.example.yaml
│   └── rover.mock.yaml
├── documents/
│   └── ROS_UPSTREAM_INTERFACE_CONTRACT.md
├── mock/
│   ├── scenario_generator.py
│   └── scenarios/*.json
├── prediction_core/                     # ROS-independent algorithms
│   ├── models.py
│   ├── config.py
│   ├── geometry_utils.py
│   ├── collision.py
│   ├── rollover.py
│   ├── predictor.py
│   ├── serialization.py
│   └── cli.py
├── tests/                               # core pytest
├── visualization/
└── ros2/
    ├── README.md
    ├── safety_perception_msgs/          # ament_cmake interface package
    │   └── msg/*.msg
    └── prediction_ros/                  # ament_python runtime package
        ├── config/prediction.yaml
        ├── launch/prediction.launch.py
        ├── prediction_ros/
        │   ├── prediction_node.py
        │   ├── adapters.py
        │   ├── cache.py
        │   ├── coordinator.py
        │   ├── validation.py
        │   ├── mock_upstream_node.py
        │   └── message_types.py
        └── test/
```

**Names that do NOT exist as packages/dirs:**
- `prediction_core/` as a separate top-level alternate spelling — actual package dir is `prediction_core/`
- `prediction_ros/` at repo root — actual path is `ros2/prediction_ros/`
- No top-level `launch/`, `docs/` (docs live in `documents/` + READMEs)

---

# 2. What was ACTUALLY implemented?

| Feature | Implemented? | Exact file(s) | Notes |
|---|---|---|---|
| Collision core | **Yes** | `prediction_core/collision.py` | Discrete per-step rectangle vs object polygon; Shapely distance; margin gate |
| Static rollover | **Yes** | `prediction_core/rollover.py`, `geometry_utils.py` | Terrain-normal roll/pitch + gravity-projected CoM SSM |
| Raw SSM | **Yes** | `RolloverStep.static_stability_margin_m` | `min(front, rear, left, right)` signed edge margins |
| Normalized SSM | **Yes** | `RolloverStep.normalized_static_stability_margin` | Edge-wise `current_i/reference_i`, then `min`; **not clamped** |
| ROS Prediction Node | **Yes** | `ros2/prediction_ros/prediction_ros/prediction_node.py` | Requires built `rclpy` + `safety_perception_msgs` |
| ROS callbacks | **Yes** | same | trajectory / objects / geometry / state / external_wrench |
| Input cache | **Yes** | `cache.py` | Trajectory cycle cache + snapshot |
| Snapshot/thread safety | **Yes** | `cache.py` (`RLock`), `coordinator.py` (`RLock`) | Snapshot under cache lock; core predict **outside** coordinator lock |
| trajectory cycle ID | **Yes** | ROS `Trajectory.trajectory_id`; cache `trajectory_id` | **Not** a field on core `Trajectory` dataclass |
| geometry ↔ trajectory matching | **Yes** | `validation.py`, `GeometryArray.source_trajectory_id` | Hard requirement: IDs must match |
| frame validation | **Yes** | `validation.py` | Default expected frame `map` |
| timestamp/staleness validation | **Partial** | `validation.py`, `PredictionCore.stale_input_warnings` | Optional max-age params default **disabled** (`-1`); geometry stamp secondary check exists |
| partial geometry handling | **Yes** | `rollover.py` | Collision all steps; rollover skips missing step_ids and logs |
| duplicate prediction guard | **Yes** | `coordinator.py` | Once per `(frame_id, trajectory_id)` |
| typed custom ROS messages | **Yes** | `ros2/safety_perception_msgs/msg/*` | 14 `.msg` files |
| RoverState support | **Partial** | msg + adapter + cache + node sub | Cached; **core deletes/ignores state** |
| ExternalWrench support | **Partial** | msg + adapter + cache + node sub | Cached; **not passed into core** |
| JSON/String legacy adapter | **Yes (legacy only)** | `adapters.JsonAdapters` | Not used by `prediction_node` main path |
| launch file | **Yes** | `ros2/prediction_ros/launch/prediction.launch.py` | Needs `config_path` |
| YAML config | **Yes** | `config/rover.*.yaml`, `ros2/prediction_ros/config/prediction.yaml` | |
| mock upstream publisher | **Yes** | `mock_upstream_node.py` | Publishes traj + empty objects + matching geometry once |
| ROS tests | **Partial** | `ros2/prediction_ros/test/*` | Pure-Python coordinator/adapter/validation tests; **no rclpy integration test** |
| rosbag compatibility | **Partial (by design)** | typed topics/msgs | No bag-specific code; should replay if types/topics/QoS match. **Not executed here** |

---

# 3. Prediction Core API

## Entry point

```python
class PredictionCore:
    def __init__(self, config: RoverConfig) -> None: ...

    def predict(
        self,
        trajectory: Trajectory,
        tracked_objects: list[TrackedObject],
        geometry: list[GeometryStep],
        state: RoverState | None = None,
    ) -> PredictionOutput:
        # V1: `del state` — state is intentionally unused
```

Internal predictors:

```python
class CollisionPredictor:
    def predict(self, trajectory: Trajectory, tracked_objects: list[TrackedObject]) -> list[CollisionStep]

class RolloverPredictor:
    def predict(self, trajectory: Trajectory, geometry: list[GeometryStep]) -> list[RolloverStep]
```

There is **no** class named `StaticRolloverPredictor`.

## Exact current models (`prediction_core/models.py`)

```text
TrajectoryStep
  step_id: int
  x: float
  y: float
  yaw: float

Trajectory
  timestamp: float
  frame_id: str
  steps: list[TrajectoryStep]
  # NOTE: no trajectory_id field in core

TrackedObject
  timestamp: float
  track_id: int | str
  class_name: str
  footprint_polygon_xy: list[tuple[float,float]]
  height_m: float | None = None
  velocity_xy: tuple[float,float] | None = None
  confidence: float | None = None

GeometryStep
  timestamp: float
  step_id: int
  plane_id: int | str
  normal_xyz: tuple[float,float,float]
  centroid_xyz: tuple[float,float,float] | None = None
  confidence: float | None = None

RoverState
  timestamp: float
  x, y, yaw, roll, pitch: float | None = None
  velocity_xy: tuple[float,float] | None = None
  acceleration_xy: tuple[float,float] | None = None
  angular_velocity_xyz: tuple[float,float,float] | None = None

RoverConfig (config.py)
  mass_kg, body_length_m, body_width_m, body_height_m
  support_length_m, support_width_m, ground_clearance_m
  com_x_m, com_y_m, com_height_m
  prediction: PredictionConfig(collision_margin_m: float = 0.20)
```

---

# 4. Current PredictionOutput

```text
PredictionOutput
  timestamp: float
  source_trajectory_stamp: float
  collision_steps: list[CollisionStep]
  rollover_steps: list[RolloverStep]

CollisionStep
  step_id: int
  distance_along_route_m: float
  collision_objects: list[CollisionObject]

CollisionObject
  object_id: int | str
  object_class: str
  min_distance_m: float
  confidence_or_validity: float | None

RolloverStep
  step_id: int
  predicted_roll_deg: float
  predicted_pitch_deg: float
  static_stability_margin_m: float
  normalized_static_stability_margin: float
  terrain_id: int | str
  confidence_or_validity: float | None
```

### Field existence checklist

| Field | Exists? |
|---|---|
| `static_stability_margin_m` | **Yes** (raw SSM) |
| `normalized_static_stability_margin` | **Yes** |
| `predicted_roll_deg` | **Yes** |
| `predicted_pitch_deg` | **Yes** |
| `terrain_id` | **Yes** |
| `confidence` / `confidence_or_validity` | **Yes** as `confidence_or_validity` |
| `severity` | **No** |
| `safe` / `unsafe` | **No** |
| `stop` / `go` | **No** |

---

# 5. Current rollover implementation

## Flow (actual)

```text
Trajectory.step.yaw
+ GeometryStep.normal_xyz (upward-canonicalized)
+ RoverConfig support rectangle + CoM
        │
        ▼
terrain_roll_pitch_rad(normal, yaw)
        │
        ▼
predicted_roll_deg / predicted_pitch_deg
        │
        ▼
projected_com_on_support_xy(...)  # gravity line ∩ support plane z=0
        │
        ▼
support_edge_margins(projected_com) → front/rear/left/right
        │
        ▼
raw SSM = min(margins)
        │
        ▼
normalized SSM = min(current_i / reference_i)
  where reference_i = margins of configured CoM on flat support
```

## Exact formulas from `prediction_core/geometry_utils.py`

**Normal canonicalize:** unit length; flip if `nz < 0`; reject near-vertical `|nz| <= 1e-6`.

**Heading basis:**
```text
forward = (cos(yaw), sin(yaw), 0)
left    = (-sin(yaw), cos(yaw), 0)
```

**Roll / pitch:**
```text
pitch = atan2( -(n · forward), nz )
roll  = atan2( -(n · left),    nz )
```

**CoM projection:** build terrain-aligned rover frame `[forward_tangent, left, n]`; transform world gravity `(0,0,-1)` into rover frame; intersect CoM→gravity ray with plane `z=0`.

**Edge margins (support frame, +X forward, +Y left):**
```text
front = L/2 - x
rear  = x + L/2
left  = W/2 - y
right = y + W/2
```

**Normalized SSM:**
```text
normalized = min(
  front_cur/front_ref,
  rear_cur/rear_ref,
  left_cur/left_ref,
  right_cur/right_ref,
)
```
- Edge-wise: **yes**
- `min` of ratios: **yes**
- Negative preserved: **yes** (explicitly not clamped)
- Zero means tipping edge

---

# 6. Rover config

## Active mock values (`config/rover.mock.yaml`) — TEST ONLY

| Field | Value |
|---|---|
| mass_kg | 100.0 |
| body_length_m | 1.05 |
| body_width_m | 0.90 |
| body_height_m | 0.50 |
| support_length_m | 0.75 |
| support_width_m | 0.88 |
| ground_clearance_m | 0.15 |
| com_x_m | 0.0 |
| com_y_m | 0.0 |
| com_height_m | 0.33 |
| collision_margin_m | 0.20 |

`config/rover.example.yaml` is a zeroed template (not production values).

## Legacy YAML compatibility

**Yes.** `load_config()` still accepts `length_m`/`width_m`/`cg_*` and maps:
- body dims from legacy length/width
- support dims default to the same body dims if omitted
- `cg_*` → `com_*`

---

# 7. Coordinate / frame conventions

From code + docstrings in `geometry_utils.py` / README:

| Axis / quantity | Convention |
|---|---|
| world +X | East (map) |
| world +Y | North |
| world +Z | Up |
| rover +X | Forward |
| rover +Y | Left |
| rover +Z | Up (terrain normal up) |
| yaw = 0 | Facing world +X |
| +yaw | CCW about +Z |
| +pitch | Terrain rises forward → nose-up |
| +roll | Terrain rises to left (RH about +X) |

Terrain normals are normalized and forced into the upward hemisphere.

---

# 8. ROS interface packages

## Actual package names

| Expected name in prompt | Actual name |
|---|---|
| `safety_perception_msgs` | **`safety_perception_msgs`** |
| `prediction_ros` | **`prediction_ros`** (under `ros2/prediction_ros/`) |

## Every `.msg` that currently exists (exact fields)

### `Point2D.msg`
```text
float64 x
float64 y
```

### `TrajectoryStep.msg`
```text
uint32 step_id
float64 x
float64 y
float64 yaw
```

### `Trajectory.msg`
```text
std_msgs/Header header
uint64 trajectory_id
TrajectoryStep[] steps
```

### `GeometryStep.msg`
```text
uint32 step_id
string plane_id
geometry_msgs/Vector3 normal
float32 confidence
bool confidence_valid
```

### `GeometryArray.msg`
```text
std_msgs/Header header
uint64 source_trajectory_id
builtin_interfaces/Time source_trajectory_stamp
GeometryStep[] steps
```

### `TrackedObject.msg`
```text
uint64 track_id
string class_name
Point2D[] footprint_polygon_xy
float32 confidence
bool confidence_valid
geometry_msgs/Vector3 velocity
bool velocity_valid
```

### `TrackedObjectArray.msg`
```text
std_msgs/Header header
TrackedObject[] objects
```

### `RoverState.msg`
```text
std_msgs/Header header
geometry_msgs/Pose pose
bool pose_valid
geometry_msgs/Twist twist
bool twist_valid
geometry_msgs/Accel acceleration
bool acceleration_valid
```

### `ExternalWrench.msg`
```text
std_msgs/Header header
string source
geometry_msgs/Wrench wrench
geometry_msgs/Point application_point
bool application_point_valid
float32 confidence
bool confidence_valid
```

### `ExternalWrenchArray.msg`
```text
std_msgs/Header header
ExternalWrench[] wrenches
```

### `CollisionObject.msg`
```text
uint64 track_id
string object_class
float64 min_distance_m
float32 confidence
bool confidence_valid
```

### `CollisionStep.msg`
```text
uint32 step_id
float64 distance_along_route_m
CollisionObject[] collision_objects
```

### `RolloverStep.msg`
```text
uint32 step_id
float64 predicted_roll_deg
float64 predicted_pitch_deg
float64 static_stability_margin_m
float64 normalized_static_stability_margin
string terrain_id
float32 confidence
bool confidence_valid
```

### `PredictionOutput.msg`
```text
std_msgs/Header header
uint64 source_trajectory_id
CollisionStep[] collision_steps
RolloverStep[] rollover_steps
```

## Expected-but-missing relative to some prompt aliases

All 14 V1 typed input/output `.msg` files listed above exist on disk under `ros2/safety_perception_msgs/msg/`.

**Actually absent by design (not found on disk):**
- any `Decision*.msg` / stop-go / severity message
- any separate `prediction_msgs` package (repo uses `safety_perception_msgs`)

---

# 9. ROS topic interface

Defaults from `prediction_node.py` / `config/prediction.yaml`:

### Subscriptions

| Topic | Type | QoS |
|---|---|---|
| `/trajectory` | `safety_perception_msgs/msg/Trajectory` | Reliable, KEEP_LAST 10 |
| `/tracked_objects` | `TrackedObjectArray` | Best effort, KEEP_LAST 10 |
| `/geometry` | `GeometryArray` | Best effort, KEEP_LAST 10 |
| `/rover/state` | `RoverState` | Best effort, KEEP_LAST 10 |
| `/external_wrenches` | `ExternalWrenchArray` | Best effort, KEEP_LAST 10 |

### Publisher

| Topic | Type | QoS |
|---|---|---|
| `/predict_output` | `PredictionOutput` | Reliable, KEEP_LAST 10 |

### Configurable parameters (topic-related and policy)

`trajectory_topic`, `tracked_objects_topic`, `geometry_topic`, `state_topic`, `external_wrench_topic`, `prediction_output_topic`, `expected_frame_id`, `config_path` (**required**), `require_full_geometry_coverage`, `max_object_age_sec`, `max_geometry_age_sec`, `max_state_age_sec`.

---

# 10. Callback flow (actual)

```text
*_callback(msg)
  → RosAdapters.*_from_ros(msg)     # typed conversion; validity flags → Optional/None
  → PredictionInputCache.set_*(...)
  → PredictionNode._try_predict()
       → PredictionCoordinator.try_predict()
            → cache.snapshot()
            → InputValidator.inputs_compatible(snapshot)
            → duplicate-cycle check
            → PredictionCore.predict(...)   # outside coordinator lock
       → if output: RosAdapters.prediction_to_ros(...) → publish /predict_output
```

Exceptions in callbacks are caught and logged; they do not crash the node.

JSON parsing is **not** on this path.

---

# 11. Prediction trigger / readiness

Validator readiness (actual):

```text
REQUIRED:
  trajectory is not None AND trajectory_id is not None
  objects is not None          # [] is allowed
  geometry is not None
  geometry_source_trajectory_id == trajectory_id
  frames match expected_frame_id (default "map")
  geometry step_ids intersect trajectory step_ids
  (optional) full coverage if require_full_geometry_coverage
  (optional) age limits if max_*_age_sec >= 0

OPTIONAL:
  state
  external_wrenches
```

### `objects is None` vs `objects == []`

**Distinguished.**  
- `None` → not ready (“missing tracked objects batch”)  
- `[]` → ready empty detection batch

### Which callbacks call `_try_predict()`?

All five: trajectory, objects, geometry, state, external_wrench.

### Is Trajectory the cycle trigger?

**Yes.** New trajectory sets cycle id and **clears objects + geometry**. Prediction still waits until objects+matching geometry arrive again.

---

# 12. Trajectory cycle association

| Question | Answer |
|---|---|
| Is there `trajectory_id`? | **Yes**, on ROS `Trajectory.trajectory_id` (`uint64`). Stored in cache separately from core `Trajectory`. |
| `source_trajectory_id` on Geometry? | **Yes**, `GeometryArray.source_trajectory_id` |
| Geometry matched by ID? | **Yes**, must equal active `trajectory_id` |
| Timestamp also checked? | **Yes**, secondary: `source_trajectory_stamp` vs `trajectory.timestamp` within 1e-3 s if stamp present |
| Frame checked? | **Yes** |
| Geometry A + Trajectory B? | **Rejected**; no predict |
| Can B run with Geometry A? | **No** (after validation). Also new trajectory clears old geometry from cache. |
| Duplicate predict prevention? | `CycleKey(frame_id, trajectory_id)` remembered after success |

Fallback if IDs omitted in cache API: `int(timestamp)` / `int(source_trajectory_stamp)` — used by tests/helpers; ROS node passes real IDs.

**Objects are NOT ID-bound** to a trajectory (no `source_trajectory_id` on `TrackedObjectArray`). They are cleared when a new trajectory arrives, but not validated against trajectory_id.

---

# 13. Cache architecture

Class: `PredictionInputCache` in `cache.py`

Approximate state:

```text
_trajectory
_trajectory_id
_objects / _objects_frame_id
_geometry / _geometry_frame_id
_geometry_source_trajectory_id
_geometry_source_trajectory_stamp
_state / _state_frame_id
_external_wrenches / _external_wrenches_frame_id
```

| Property | Actual |
|---|---|
| Thread safety | `threading.RLock` on mutate/snapshot |
| Snapshot | immutable `PredictionSnapshot` dataclass |
| Retention | latest only; no multi-cycle geometry map |
| On new trajectory | clears objects + geometry (+ their meta); **does not clear state or wrenches** |
| Predict under lock? | Snapshot under cache lock; core predict **not** under cache lock; coordinator lock only for duplicate-cycle bookkeeping |

Possible stale-data issues:
1. State/wrenches can survive across trajectory cycles.
2. Objects lack trajectory_id association.
3. Small TOCTOU window between duplicate check and recording last cycle under concurrency.

---

# 14. ROS adapters

`RosAdapters` conversions:

| ROS → core | Mapping notes |
|---|---|
| Trajectory | header stamp/frame → timestamp/frame_id; steps → TrajectoryStep |
| TrackedObjectArray | empty list OK; `velocity_valid=false` → `velocity_xy=None`; `confidence_valid=false` → `confidence=None` |
| GeometryArray | `confidence_valid=false` → `confidence=None`; normal xyz copied |
| RoverState | `pose_valid=false` → x/y/yaw None; `twist_valid=false` → velocity/angular None; `acceleration_valid=false` → **acceleration_xy=None (not zero)** |
| ExternalWrenchArray | application_point_valid false → None |
| PredictionOutput ← core | fills typed output; confidence None → `confidence_valid=false` |

`JsonAdapters` remains for legacy JSON payloads/tests only.

---

# 15. RoverState and dynamic data status

| Quantity | Status |
|---|---|
| pose | **C/D** (msg+cache+core model) / **not E** (unused by algorithm) |
| linear velocity | **C/D** / not E |
| angular velocity | **C/D** / not E |
| linear acceleration | **C/D** / not E |
| angular acceleration | **B only in Accel.angular** if twist/accel valid; core model has no dedicated angular accel field used |
| external force | **B/C** (wrench force) / not D into core predict / not E |
| external torque | **B/C** / not E |
| application point | **B/C** / not E |

Legend: A none, B msg only, C cached, D converted to core model, E used by algorithm.

---

# 16. Current collision implementation

- Body footprint: axis-aligned rectangle in rover frame, rotated by step yaw, size `body_length_m × body_width_m`.
- Objects: metric footprint polygons (`Shapely`).
- Distance: `Polygon.distance` (boundary/area distance).
- Candidate if `min_distance <= collision_margin_m` (eps 1e-9).
- Velocity: **not used**.
- Swept path / segment interpolation: **not implemented**.
- **Gap exists:** obstacles between sparse trajectory samples can be missed; only discrete poses are checked.

---

# 17. Partial Geometry behavior

If trajectory steps `0..10` and geometry `0..5`:

| Path | Behavior |
|---|---|
| Collision | Runs **all** trajectory steps |
| Rollover | Emits steps with geometry only; missing IDs logged via `last_missing_step_ids` |
| Whole prediction | **Still runs** (default `require_full_geometry_coverage=false`) |
| Strict mode | If param true, validator blocks before predict |

---

# 18. Frame / timestamp / staleness handling

| Mechanism | Status |
|---|---|
| frame_id checking | Implemented against `expected_frame_id` |
| header.stamp | Used as trajectory/object/geometry/state timestamps |
| source_trajectory_stamp | Secondary geometry check |
| max age | Implemented but **off by default** |
| clock handling | Uses message stamps / `time.time()` for output timestamp; no sim-time special case |
| Bag implications | No bag-specific logic; relies on message content |

---

# 19. ROS bag readiness

**Assessment only (no bag created/played):**

If `ros2 bag play` publishes the same typed topics with matching QoS and frames/IDs, the node **should** receive them like live publishers.

Possible blockers in practice:
- Packages not built/sourced (`safety_perception_msgs`, `prediction_ros`)
- QoS mismatch (reliable vs best effort)
- Missing/mismatched `trajectory_id` / `source_trajectory_id`
- Wrong `frame_id`
- Objects never published (node waits forever)
- This environment cannot verify because ROS is absent

---

# 20. Launch / configuration

- Launch: `ros2/prediction_ros/launch/prediction.launch.py`
- Node params file: `ros2/prediction_ros/config/prediction.yaml`
- Rover geometry: `config/rover.mock.yaml` (or example)

Intended command (from sources; not executed here):

```bash
ros2 launch prediction_ros prediction.launch.py \
  config_path:=/Users/sonha/src/landfill-rover-folder/prediction/prediction-src/config/rover.mock.yaml
```

---

# 21. How to build from a CLEAN shell

**Environment blocker:** ROS 2 / colcon are **not installed** on the audited machine. Distro cannot be detected.

Intended commands based on package metadata:

```bash
cd /Users/sonha/src/landfill-rover-folder/prediction/prediction-src

# core python package
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[test]'

# ROS packages (requires sourcing a real distro first)
source /opt/ros/<INSTALLED_DISTRO>/setup.bash
cd ros2
colcon build --packages-select safety_perception_msgs prediction_ros
source install/setup.bash
```

Makefile target `make ros-build` encodes the same colcon invocation and refuses to run without `ROS_DISTRO`.

---

# 22. How to run

### Terminal 1 — Prediction node

```bash
source /opt/ros/<INSTALLED_DISTRO>/setup.bash
source /Users/sonha/src/landfill-rover-folder/prediction/prediction-src/ros2/install/setup.bash
ros2 run prediction_ros prediction_node --ros-args \
  -p config_path:=/Users/sonha/src/landfill-rover-folder/prediction/prediction-src/config/rover.mock.yaml
```

### Terminal 2 — mock upstream

```bash
ros2 run prediction_ros mock_upstream_node
```

### Terminal 3 — inspect

```bash
ros2 topic list
ros2 topic info /predict_output
ros2 topic echo /predict_output
```

---

# 23. End-to-end current behavior

**Not executed.** Blocker: no `ros2` / `rclpy` / built message package in this environment.

Expected sequence from source of `mock_upstream_node` + node logic:

```text
1) publish Trajectory(trajectory_id=1, 3 steps)
2) publish TrackedObjectArray(objects=[])
3) publish GeometryArray(source_trajectory_id=1, flat normals)
4) prediction_node caches each, try_predict on each callback
5) after all three present + ID match → PredictionCore.predict once
6) publish PredictionOutput on /predict_output
```

No representative live `/predict_output` sample is available from this audit host.

---

# 24. Tests

Command run:

```bash
.venv/bin/python -m pytest -q
```

**Result: 80 passed, 0 failed, 0 skipped** (0.07–0.09 s)

Includes:
- core tests under `tests/`
- ROS wrapper pure-Python tests under `ros2/prediction_ros/test/`

`colcon test`: **NOT RUN** (colcon unavailable).

---

# 25. Build status

| Package | Result |
|---|---|
| `safety_perception_msgs` | **NOT TESTED** — ROS/colcon unavailable |
| `prediction_ros` | **NOT TESTED** — ROS/colcon unavailable |
| `prediction_core` (pip package) | Import/tests OK in `.venv` |

Do not treat pytest green as ROS build green.

---

# 26. Git changes / source history

Inside `prediction-src`:

```text
git status --short  → empty
branch main @ 420e342 tracking origin/main
```

No uncommitted changes in the nested repo.

Parent repo `/Users/sonha/src/landfill-rover-folder/prediction` **does** show dirty state (modified tracked files under `prediction-src/`, plus untracked docs/ROS packages from parent’s perspective). That is a workspace nesting artifact, not uncommitted work inside the nested GitHub repo.

---

# 27. Current documentation

| Doc | One-sentence description |
|---|---|
| `README.md` | Core V1 algorithm/contracts overview, install/test, conventions |
| `ros2/README.md` | How to build/run typed ROS wrapper |
| `documents/ROS_UPSTREAM_INTERFACE_CONTRACT.md` | Upstream-facing topic/message/cycle/validity contract |

### Doc vs source disagreements / drift

1. README package-layout sketch still reads like an older tree in places (naming/path prose vs actual `prediction_core/`, `ros2/`, `visualization/`).
2. Interface contract and code agree on major typed topics/IDs, but **ROS build/run is unverified** here.
3. Contract says objects are required as a message; code matches (`None` waits, `[]` OK).
4. Some historical prompt names (`prediction_msgs`, JSON-primary transport) are obsolete relative to current source.

---

# 28. Known TODOs / gaps

## Blocking current build/run
- No ROS 2 distro / `colcon` / `ros2` on this machine
- `prediction_node` cannot start without built `safety_perception_msgs` + `rclpy`

## Blocking upstream integration
- Upstream must publish typed msgs with matching `trajectory_id` / `source_trajectory_id`
- Objects topic has no trajectory association field
- Real rover dimensions not in config (mock only)

## Static Prediction limitations
- Discrete-step collision only (no swept volume)
- Objects assumed static
- Rectangular support model only
- State ignored

## Future dynamic rollover work
- RoverState / ExternalWrench are interface-ready but algorithm-ignored
- No FASM/ZMP/LTR/force-moment model

## Nice-to-have cleanup
- Clear state/wrenches on new trajectory (or document retention)
- Add objects↔trajectory association if needed
- rclpy smoke / bag replay CI
- Align README layout text with actual paths

---

# 29. Static vs dynamic status

| Capability | Interface exists | Data cached | Algorithm uses it |
|---|---:|---:|---:|
| terrain normal | yes | yes | **yes** |
| trajectory yaw | yes | yes | **yes** |
| CoM/support polygon | YAML config | n/a (config) | **yes** |
| pose | yes | yes | no |
| linear velocity | yes | yes | no |
| angular velocity | yes | yes | no |
| linear acceleration | yes | yes | no |
| angular acceleration | via Accel msg | partial | no |
| external force | yes | yes | no |
| external torque | yes | yes | no |
| force application point | yes | yes | no |

---

# 30. Final summary

## CURRENT STATUS

- ROS-independent core implements static collision + quasi-static rollover with raw and normalized SSM.
- Typed ROS interface package `safety_perception_msgs` exists with 14 messages.
- `prediction_ros` implements cache/validate/coordinate/publish around typed topics.
- Trajectory cycle identity is `trajectory_id`; geometry must match `source_trajectory_id`.
- Empty object batches are valid; missing object messages are not.
- RoverState and ExternalWrench are subscribed/cached but unused by V1 algorithms.
- Pytest: **80 passed**.
- ROS build/run/bag echo: **not verified** (no ROS toolchain here).
- Mock/example rover YAML is explicitly non-production.

## CURRENT RUNTIME FLOW

```text
/trajectory ───────────────┐
/tracked_objects ──────────┤
/geometry ─────────────────┼─► PredictionNode callbacks
/rover/state (optional) ───┤         │
/external_wrenches (opt) ──┘         ▼
                              PredictionInputCache.snapshot
                                         │
                                         ▼
                              InputValidator (frame/ID/ready)
                                         │
                                         ▼
                              PredictionCoordinator (1× per cycle)
                                         │
                                         ▼
                              PredictionCore.predict
                               ├─ CollisionPredictor
                               └─ RolloverPredictor
                                         │
                                         ▼
                              /predict_output  (PredictionOutput)
```

## CURRENT REQUIRED INPUTS

- `Trajectory` (with `trajectory_id`)
- `TrackedObjectArray` message (may be empty)
- `GeometryArray` with `source_trajectory_id == trajectory_id`
- Matching `frame_id` (default `map`)
- Rover YAML via `config_path`

## CURRENT OPTIONAL / UNUSED INPUTS

- `RoverState` (optional; ignored by core)
- `ExternalWrenchArray` (optional; ignored by core)
- Object velocity/confidence when invalid → None
- Acceleration/twist when invalid → None (not zero)

## EXACT BUILD COMMAND

```bash
cd /Users/sonha/src/landfill-rover-folder/prediction/prediction-src
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[test]'

source /opt/ros/<INSTALLED_DISTRO>/setup.bash
cd ros2
colcon build --packages-select safety_perception_msgs prediction_ros
source install/setup.bash
```

## EXACT RUN COMMAND

```bash
source /opt/ros/<INSTALLED_DISTRO>/setup.bash
source /Users/sonha/src/landfill-rover-folder/prediction/prediction-src/ros2/install/setup.bash

ros2 run prediction_ros prediction_node --ros-args \
  -p config_path:=/Users/sonha/src/landfill-rover-folder/prediction/prediction-src/config/rover.mock.yaml

# other terminal
ros2 run prediction_ros mock_upstream_node

# other terminal
ros2 topic echo /predict_output
```

## TEST RESULTS

```text
pytest: 80 passed, 0 failed, 0 skipped
colcon test: NOT RUN (colcon unavailable)
colcon build: NOT RUN (ROS unavailable)
```

## MOST IMPORTANT TECHNICAL GAPS

1. ROS toolchain missing on audit host → node/build/bag unverified.
2. Core `Trajectory` has no `trajectory_id` (ID lives only in ROS cache layer).
3. Tracked objects are not tied to `trajectory_id` in the message contract.
4. State/external wrenches persist across trajectory cycles in cache.
5. Collision is discrete-step only; inter-sample gaps can miss obstacles.
6. Dynamic fields exist in interface but are algorithmically unused.
7. Rover parameters are mock estimates, not measured hardware values.
8. No rclpy/launch integration test in CI.
9. Parent git repo nesting is dirty/confusing vs clean nested GitHub repo.
10. README layout prose still partially drifts from the actual tree.

## FILES I SHOULD SEND TO ANOTHER ENGINEER

Minimum set:

1. `prediction_core/models.py`
2. `prediction_core/geometry_utils.py`
3. `prediction_core/collision.py`
4. `prediction_core/rollover.py`
5. `prediction_core/predictor.py`
6. `prediction_core/config.py`
7. `config/rover.mock.yaml`
8. `ros2/safety_perception_msgs/msg/*`
9. `ros2/prediction_ros/prediction_ros/prediction_node.py`
10. `ros2/prediction_ros/prediction_ros/cache.py`
11. `ros2/prediction_ros/prediction_ros/validation.py`
12. `ros2/prediction_ros/prediction_ros/coordinator.py`
13. `ros2/prediction_ros/prediction_ros/adapters.py`
14. `documents/ROS_UPSTREAM_INTERFACE_CONTRACT.md`
15. `CURRENT_PREDICTION_SOURCE_AUDIT.md` (this file)

---

*End of audit.*
