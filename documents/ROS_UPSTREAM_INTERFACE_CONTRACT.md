# ROS upstream interface contract (Prediction Python V1)

`safety_perception_msgs` is the canonical ROS 2 interface package for the
Landfill Rover safety pipeline. It is a **transport/adapter** layer around the
frozen Pure Python Prediction V1 (`PredictionRuntime` + `PredictionCore`).

Live Gazebo, a rover, and `ros2 bag play` all use the same topics and messages.

## Topics

| Topic | Type | Publisher | Static profile | Dynamic profile |
|---|---|---|---|---|
| `/trajectory` | `Trajectory` | planning | required | required |
| `/tracked_objects` | `TrackedObjectArray` | tracking | required | required |
| `/geometry` | `GeometryArray` | terrain | required | required |
| `/rover/state` | `RoverState` | state | optional | **required** (valid `acceleration_xyz`) |
| `/external_wrenches` | `ExternalWrenchArray` | dynamics | optional | optional |
| `/predict_output` | `PredictionOutput` | prediction | output | output |

Topic names are parameterized on the prediction node.

All current V1 inputs use metric SI units and normally use the `map` frame:
`+X` East, `+Y` North, `+Z` Up, and positive yaw CCW about `+Z`.

## Prediction profile parameter

```yaml
prediction_profile: static   # or dynamic
```

Launch override:

```bash
ros2 launch prediction_ros prediction.launch.py \
  config_path:=/abs/path/config/rover.mock.yaml \
  prediction_profile:=dynamic
```

### Static readiness

Required: trajectory, tracked-objects batch (`[]` valid), matching geometry,
frame/cycle validation. State and wrenches optional.

### Dynamic readiness

Everything static requires, **plus** `RoverState` with valid linear
`acceleration_xyz`. Velocity / angular rates are **not** required.
External wrench remains optional (no wait / timeout).

## Cycle and frame contract

`Trajectory.trajectory_id` identifies a planned trajectory cycle. A new ID
opens a new prediction cycle and clears cycle-bound:

- objects
- geometry
- state
- external_wrenches

`GeometryArray.source_trajectory_id` **must equal** the active
`Trajectory.trajectory_id`. Prediction performs no TF transforms.

Prediction runs **at most once** per trajectory cycle (canonical in
`PredictionRuntime`).

## Availability semantics

| ROS | Python | Meaning |
|---|---|---|
| no `TrackedObjectArray` yet | `objects=None` | unavailable → wait |
| `objects=[]` | `objects=[]` | known empty |
| `acceleration_valid=false` | `acceleration_xyz=None` | unavailable (never treated as zero) |
| `acceleration_valid=true`, linear=(0,0,0) | `acceleration_xyz=(0,0,0)` | valid zero |
| no `ExternalWrenchArray` yet | `external_wrenches=None` | wrench info unavailable |
| `wrenches=[]` | `external_wrenches=[]` | explicitly empty |
| `wrenches=[...]` | `external_wrenches=[...]` | wrenches present |

`None` / unavailable ≠ explicit empty / zero.

## RoverState acceleration

Required semantic for Prediction:

- kinematic CoM acceleration in the common `map` inertial frame
- units: m/s²
- **gravity NOT included** (Prediction introduces `g_world` itself)

Map:

```text
acceleration.linear.{x,y,z}  →  RoverState.acceleration_xyz
```

Do **not** feed raw accelerometer specific-force unless upstream has already
converted it to this kinematic definition.

Angular velocity / angular acceleration may be mapped into Python fields but
are unused by V1 algorithms and are **not** readiness requirements.

## External wrench

| ROS field | Python | Units |
|---|---|---|
| `wrench.force.{x,y,z}` | `force_xyz` | N |
| `wrench.torque.{x,y,z}` | `torque_xyz` | N·m |
| `application_point` if `application_point_valid` | `application_point_xyz` | m |
| `application_point_valid=false` | `application_point_xyz=None` | — |

Do not invent an application point. Do not assume force acts at CoM.

## Prediction output evidence

`/predict_output` publishes physical evidence only — **no** severity, safe/unsafe,
Stop/Go.

Per `RolloverStep`:

| Field | Role |
|---|---|
| roll / pitch | PRIMARY BASELINE |
| Static SSM / Normalized Static SSM | PRIMARY BASELINE |
| `stability_moment` (`StabilityMomentEvidence`) | PRIMARY DYNAMIC |
| `zmp` (`ZmpEvidence`) | DIAGNOSTIC / visualization |

`valid=false` on nested dynamic messages means unavailable — do not treat numeric
zeros as evidence.

### Edge naming

| Field | Meaning |
|---|---|
| `stability_moment.minimum_normalized_moment_edge` | edge with minimum **normalized** Stability Moment |
| `zmp.nearest_edge` | edge with minimum **raw** ZMP margin |

These can differ; do not rename both to a generic `critical_edge`.

Effective SSM and critical tip angle remain Python-side diagnostics and are
intentionally **not** in the primary ROS rollover message.

## Build and replay

After ROS 2 is sourced:

```bash
cd prediction-src/ros2
colcon build --packages-select safety_perception_msgs prediction_ros
source install/setup.bash
```

QoS (unchanged):

- `/trajectory`, `/predict_output`: Reliable KEEP_LAST 10
- objects / geometry / state / wrenches: Best effort KEEP_LAST 10

The node is rosbag-ready as normal typed topics — no bag-specific logic inside
`PredictionCore`.

Mock upstream:

```bash
ros2 run prediction_ros mock_upstream_node --ros-args -p demo_mode:=static
ros2 run prediction_ros mock_upstream_node --ros-args -p demo_mode:=dynamic
ros2 run prediction_ros mock_upstream_node --ros-args -p demo_mode:=dynamic_zero
```
