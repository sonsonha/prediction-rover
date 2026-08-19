# ROS upstream interface contract

`safety_perception_msgs` is the canonical ROS 2 interface package for the
Landfill Rover safety pipeline. It is transport-neutral: live Gazebo, a rover,
and `ros2 bag play` all use the same topics and messages.

## Topics

| Topic | Type | Publisher | Required by V1 Prediction |
|---|---|---|---|
| `/trajectory` | `safety_perception_msgs/msg/Trajectory` | planning | yes |
| `/tracked_objects` | `safety_perception_msgs/msg/TrackedObjectArray` | tracking | yes |
| `/geometry` | `safety_perception_msgs/msg/GeometryArray` | terrain | yes |
| `/rover/state` | `safety_perception_msgs/msg/RoverState` | Gazebo/rover state | no |
| `/external_wrenches` | `safety_perception_msgs/msg/ExternalWrenchArray` | Gazebo/dynamics | no |
| `/predict_output` | `safety_perception_msgs/msg/PredictionOutput` | prediction | output |

All current V1 inputs use metric SI units and normally use the `map` frame:
`+X` East, `+Y` North, `+Z` Up, and positive yaw CCW about `+Z`.

## Cycle and frame contract

`Trajectory.trajectory_id` identifies a planned trajectory cycle. A new ID
creates a new prediction cycle and clears cycle-bound object/geometry cache.
`GeometryArray.source_trajectory_id` **must equal** the active
`Trajectory.trajectory_id`; a mismatched geometry message is rejected.
`source_trajectory_stamp` is retained as a secondary timing/debug check.

`TrackedObjectArray.objects=[]` means tracking completed and found no objects.
No `TrackedObjectArray` message means tracking data is unavailable, so
Prediction waits. Trajectory, objects, and geometry headers must use the
configured frame (default `map`); Prediction performs no TF transforms.

## Availability semantics

Dynamic quantities use `*_valid` flags. A false flag means unavailable, not
zero. For example, `RoverState.acceleration_valid=false` becomes Python
`None`, while a valid `(0, 0, 0)` acceleration means a measured zero value.
The same distinction applies to object velocity, pose, twist, wrench
application point, and confidence fields.

## Current and future usage

Required now: trajectory steps, geometry normals associated to that trajectory,
object footprint polygons, and the separate rover physical YAML config.
Current static collision uses trajectory and footprint polygons. Current
static/quasi-static rollover uses trajectory yaw, terrain normal, support
polygon, and CoM; it outputs roll, pitch, raw SSM, and normalized SSM.

`RoverState` and `ExternalWrenchArray` are accepted and cached but ignored by
the V1 static algorithms. Future dynamic stability work may use valid pose,
twist, acceleration, angular velocity, external wrench, and application point
to calculate destabilizing moments. This interface introduces no Decision,
Stop/Go, severity, FASM, ZMP, LTR, or force-based rollover policy.

## Build and replay

After ROS 2 is sourced, build both packages:

```bash
cd prediction-src/ros2
colcon build --packages-select safety_perception_msgs prediction_ros
source install/setup.bash
```

The node has parameterized topic names, reliable trajectory/output QoS, and
best-effort keep-last-10 object/geometry/state/wrench QoS. Match publisher QoS
where required. Topic type, frame, ID, and timestamp correctness are all that
is needed for future `ros2 bag record` and replay.
