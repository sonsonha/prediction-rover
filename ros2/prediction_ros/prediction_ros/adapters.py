"""Convert ROS messages to ROS-independent prediction_core dataclasses.

JsonAdapters is retained only for development/backward compatibility. The
runtime node uses RosAdapters and safety_perception_msgs exclusively.
"""

from __future__ import annotations

import json
import math
from typing import Any

from prediction_core.models import (
    CollisionObject,
    CollisionStep,
    GeometryStep,
    PredictionOutput,
    RolloverStep,
    RoverState,
    TrackedObject,
    Trajectory,
    TrajectoryStep,
)

from .message_types import RosHeader


class JsonAdapters:
    """Parse and serialize the dev JSON-over-String contract."""

    @staticmethod
    def parse_json_string(raw: str) -> dict[str, Any]:
        payload = json.loads(raw)
        if not isinstance(payload, dict):
            raise ValueError("JSON payload must be an object")
        return payload

    @staticmethod
    def trajectory_from_payload(payload: dict[str, Any]) -> Trajectory:
        header = RosHeader.from_payload(payload)
        steps_raw = payload.get("steps")
        if not isinstance(steps_raw, list) or not steps_raw:
            raise ValueError("trajectory requires non-empty steps[]")
        steps = [
            TrajectoryStep(
                step_id=int(step["step_id"]),
                x=float(step["x"]),
                y=float(step["y"]),
                yaw=float(step["yaw"]),
            )
            for step in steps_raw
        ]
        timestamp = float(payload.get("timestamp", header.timestamp))
        frame_id = str(payload.get("frame_id", header.frame_id)).strip()
        return Trajectory(timestamp=timestamp, frame_id=frame_id, steps=steps)

    @staticmethod
    def objects_from_payload(payload: dict[str, Any]) -> tuple[list[TrackedObject], RosHeader, float | None]:
        header = RosHeader.from_payload(payload)
        objects_raw = payload.get("objects", [])
        if not isinstance(objects_raw, list):
            raise ValueError("tracked object payload requires objects[] list")
        objects = [
            TrackedObject(
                timestamp=float(obj.get("timestamp", payload.get("timestamp", header.timestamp))),
                track_id=obj["track_id"],
                class_name=str(obj["class_name"]),
                footprint_polygon_xy=[
                    (float(point[0]), float(point[1]))
                    for point in obj["footprint_polygon_xy"]
                ],
                height_m=None if obj.get("height_m") is None else float(obj["height_m"]),
                velocity_xy=(
                    None
                    if obj.get("velocity_xy") is None
                    else (float(obj["velocity_xy"][0]), float(obj["velocity_xy"][1]))
                ),
                confidence=None if obj.get("confidence") is None else float(obj["confidence"]),
            )
            for obj in objects_raw
        ]
        source_stamp = payload.get("source_trajectory_stamp")
        return objects, header, None if source_stamp is None else float(source_stamp)

    @staticmethod
    def geometry_from_payload(
        payload: dict[str, Any],
    ) -> tuple[list[GeometryStep], RosHeader, float | None]:
        header = RosHeader.from_payload(payload)
        geometry_raw = payload.get("geometry", [])
        if not isinstance(geometry_raw, list):
            raise ValueError("geometry payload requires geometry[] list")
        geometry = [
            GeometryStep(
                timestamp=float(item.get("timestamp", payload.get("timestamp", header.timestamp))),
                step_id=int(item["step_id"]),
                plane_id=item["plane_id"],
                normal_xyz=(
                    float(item["normal_xyz"][0]),
                    float(item["normal_xyz"][1]),
                    float(item["normal_xyz"][2]),
                ),
                centroid_xyz=(
                    None
                    if item.get("centroid_xyz") is None
                    else (
                        float(item["centroid_xyz"][0]),
                        float(item["centroid_xyz"][1]),
                        float(item["centroid_xyz"][2]),
                    )
                ),
                confidence=None if item.get("confidence") is None else float(item["confidence"]),
            )
            for item in geometry_raw
        ]
        source_stamp = payload.get("source_trajectory_stamp")
        return geometry, header, None if source_stamp is None else float(source_stamp)

    @staticmethod
    def state_from_payload(payload: dict[str, Any]) -> RoverState:
        header = RosHeader.from_payload(payload)
        optional = payload.get
        return RoverState(
            timestamp=float(optional("timestamp", header.timestamp)),
            x=None if optional("x") is None else float(optional("x")),
            y=None if optional("y") is None else float(optional("y")),
            yaw=None if optional("yaw") is None else float(optional("yaw")),
            roll=None if optional("roll") is None else float(optional("roll")),
            pitch=None if optional("pitch") is None else float(optional("pitch")),
            velocity_xy=(
                None
                if optional("velocity_xy") is None
                else (float(optional("velocity_xy")[0]), float(optional("velocity_xy")[1]))
            ),
            acceleration_xy=(
                None
                if optional("acceleration_xy") is None
                else (
                    float(optional("acceleration_xy")[0]),
                    float(optional("acceleration_xy")[1]),
                )
            ),
            angular_velocity_xyz=(
                None
                if optional("angular_velocity_xyz") is None
                else (
                    float(optional("angular_velocity_xyz")[0]),
                    float(optional("angular_velocity_xyz")[1]),
                    float(optional("angular_velocity_xyz")[2]),
                )
            ),
        )

    @staticmethod
    def prediction_to_payload(output: PredictionOutput, frame_id: str) -> dict[str, Any]:
        header = RosHeader(frame_id=frame_id, timestamp=output.timestamp)
        payload = header.to_payload()
        payload.update(
            {
                "timestamp": output.timestamp,
                "source_trajectory_stamp": output.source_trajectory_stamp,
                "collision_steps": [
                    {
                        "step_id": step.step_id,
                        "distance_along_route_m": step.distance_along_route_m,
                        "collision_objects": [
                            {
                                "object_id": candidate.object_id,
                                "object_class": candidate.object_class,
                                "min_distance_m": candidate.min_distance_m,
                                "confidence_or_validity": candidate.confidence_or_validity,
                            }
                            for candidate in step.collision_objects
                        ],
                    }
                    for step in output.collision_steps
                ],
                "rollover_steps": [
                    {
                        "step_id": step.step_id,
                        "predicted_roll_deg": step.predicted_roll_deg,
                        "predicted_pitch_deg": step.predicted_pitch_deg,
                        "static_stability_margin_m": step.static_stability_margin_m,
                        "normalized_static_stability_margin": step.normalized_static_stability_margin,
                        "terrain_id": step.terrain_id,
                        "confidence_or_validity": step.confidence_or_validity,
                    }
                    for step in output.rollover_steps
                ],
            }
        )
        return payload

    @staticmethod
    def prediction_to_json(output: PredictionOutput, frame_id: str) -> str:
        return json.dumps(JsonAdapters.prediction_to_payload(output, frame_id), sort_keys=True)


class RosAdapters:
    """Typed ROS message adapter with no ROS import at module import time."""

    @staticmethod
    def timestamp(time_msg: Any) -> float:
        return float(time_msg.sec) + float(time_msg.nanosec) * 1e-9

    @staticmethod
    def trajectory_from_ros(msg: Any) -> Trajectory:
        return Trajectory(
            timestamp=RosAdapters.timestamp(msg.header.stamp),
            frame_id=msg.header.frame_id,
            steps=[
                TrajectoryStep(step_id=step.step_id, x=step.x, y=step.y, yaw=step.yaw)
                for step in msg.steps
            ],
        )

    @staticmethod
    def objects_from_ros(msg: Any) -> list[TrackedObject]:
        timestamp = RosAdapters.timestamp(msg.header.stamp)
        return [
            TrackedObject(
                timestamp=timestamp,
                track_id=item.track_id,
                class_name=item.class_name,
                footprint_polygon_xy=[(point.x, point.y) for point in item.footprint_polygon_xy],
                velocity_xy=(item.velocity.x, item.velocity.y) if item.velocity_valid else None,
                confidence=float(item.confidence) if item.confidence_valid else None,
            )
            for item in msg.objects
        ]

    @staticmethod
    def geometry_from_ros(msg: Any) -> list[GeometryStep]:
        timestamp = RosAdapters.timestamp(msg.header.stamp)
        return [
            GeometryStep(
                timestamp=timestamp,
                step_id=item.step_id,
                plane_id=item.plane_id,
                normal_xyz=(item.normal.x, item.normal.y, item.normal.z),
                confidence=float(item.confidence) if item.confidence_valid else None,
            )
            for item in msg.steps
        ]

    @staticmethod
    def state_from_ros(msg: Any) -> RoverState:
        timestamp = RosAdapters.timestamp(msg.header.stamp)
        if msg.pose_valid:
            orientation = msg.pose.orientation
            yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
            )
            x, y = msg.pose.position.x, msg.pose.position.y
        else:
            x = y = yaw = None
        return RoverState(
            timestamp=timestamp,
            x=x,
            y=y,
            yaw=yaw,
            velocity_xy=(msg.twist.linear.x, msg.twist.linear.y) if msg.twist_valid else None,
            acceleration_xy=(
                (msg.acceleration.linear.x, msg.acceleration.linear.y)
                if msg.acceleration_valid
                else None
            ),
            angular_velocity_xyz=(
                (msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z)
                if msg.twist_valid
                else None
            ),
        )

    @staticmethod
    def external_wrenches_from_ros(msg: Any) -> list[Any]:
        from .cache import ExternalWrenchData

        return [
            ExternalWrenchData(
                source=item.source,
                frame_id=item.header.frame_id,
                force_xyz=(item.wrench.force.x, item.wrench.force.y, item.wrench.force.z),
                torque_xyz=(item.wrench.torque.x, item.wrench.torque.y, item.wrench.torque.z),
                application_point_xyz=(
                    (item.application_point.x, item.application_point.y, item.application_point.z)
                    if item.application_point_valid
                    else None
                ),
                confidence=float(item.confidence) if item.confidence_valid else None,
            )
            for item in msg.wrenches
        ]

    @staticmethod
    def prediction_to_ros(
        output: PredictionOutput,
        *,
        source_trajectory_id: int,
        frame_id: str,
        output_type: Any,
        collision_step_type: Any,
        collision_object_type: Any,
        rollover_step_type: Any,
    ) -> Any:
        message = output_type()
        message.header.frame_id = frame_id
        message.header.stamp.sec = int(output.timestamp)
        message.header.stamp.nanosec = int(round((output.timestamp % 1) * 1e9))
        message.source_trajectory_id = source_trajectory_id
        message.collision_steps = []
        for step in output.collision_steps:
            ros_step = collision_step_type()
            ros_step.step_id = step.step_id
            ros_step.distance_along_route_m = step.distance_along_route_m
            ros_step.collision_objects = []
            for candidate in step.collision_objects:
                ros_candidate = collision_object_type()
                ros_candidate.track_id = int(candidate.object_id)
                ros_candidate.object_class = candidate.object_class
                ros_candidate.min_distance_m = candidate.min_distance_m
                ros_candidate.confidence_valid = candidate.confidence_or_validity is not None
                if ros_candidate.confidence_valid:
                    ros_candidate.confidence = candidate.confidence_or_validity
                ros_step.collision_objects.append(ros_candidate)
            message.collision_steps.append(ros_step)
        message.rollover_steps = []
        for step in output.rollover_steps:
            ros_step = rollover_step_type()
            ros_step.step_id = step.step_id
            ros_step.predicted_roll_deg = step.predicted_roll_deg
            ros_step.predicted_pitch_deg = step.predicted_pitch_deg
            ros_step.static_stability_margin_m = step.static_stability_margin_m
            ros_step.normalized_static_stability_margin = step.normalized_static_stability_margin
            ros_step.terrain_id = str(step.terrain_id)
            ros_step.confidence_valid = step.confidence_or_validity is not None
            if ros_step.confidence_valid:
                ros_step.confidence = step.confidence_or_validity
            message.rollover_steps.append(ros_step)
        return message
