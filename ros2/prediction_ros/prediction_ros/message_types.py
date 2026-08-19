"""ROS interface documentation helpers.

Production runtime uses generated types from `safety_perception_msgs`.
`JsonAdapters` remains for legacy dev JSON-over-String migration only.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

CANONICAL_INTERFACE_PACKAGE = "safety_perception_msgs"


@dataclass(frozen=True)
class RosHeader:
    frame_id: str
    timestamp: float

    @staticmethod
    def from_payload(payload: dict[str, Any]) -> "RosHeader":
        header = payload.get("header")
        if not isinstance(header, dict):
            raise ValueError("payload requires header mapping")
        frame_id = str(header.get("frame_id", payload.get("frame_id", ""))).strip()
        if not frame_id:
            raise ValueError("header.frame_id must not be empty")
        stamp = header.get("stamp")
        if isinstance(stamp, dict):
            timestamp = float(stamp.get("sec", 0)) + float(stamp.get("nanosec", 0)) * 1e-9
        else:
            timestamp = float(payload.get("timestamp"))
        if not isinstance(timestamp, float) or timestamp != timestamp:
            raise ValueError("header timestamp must be finite")
        return RosHeader(frame_id=frame_id, timestamp=timestamp)

    def to_payload(self) -> dict[str, Any]:
        sec = int(self.timestamp)
        nanosec = int(round((self.timestamp - sec) * 1e9))
        return {
            "header": {
                "stamp": {"sec": sec, "nanosec": nanosec},
                "frame_id": self.frame_id,
            }
        }


# Mirrors `safety_perception_msgs/msg/*.msg` for quick reference in Python tests/docs.
CANONICAL_MESSAGES = {
    "Trajectory": [
        "std_msgs/Header header",
        "uint64 trajectory_id",
        "TrajectoryStep[] steps",
    ],
    "GeometryArray": [
        "std_msgs/Header header",
        "uint64 source_trajectory_id",
        "builtin_interfaces/Time source_trajectory_stamp",
        "GeometryStep[] steps",
    ],
    "TrackedObjectArray": [
        "std_msgs/Header header",
        "TrackedObject[] objects",
    ],
    "RoverState": [
        "std_msgs/Header header",
        "geometry_msgs/Pose pose",
        "bool pose_valid",
        "geometry_msgs/Twist twist",
        "bool twist_valid",
        "geometry_msgs/Accel acceleration",
        "bool acceleration_valid",
    ],
    "PredictionOutput": [
        "std_msgs/Header header",
        "uint64 source_trajectory_id",
        "CollisionStep[] collision_steps",
        "RolloverStep[] rollover_steps",
    ],
}

# Deprecated alias kept for older imports/tests.
PROPOSED_PRODUCTION_MESSAGES = CANONICAL_MESSAGES
