"""Internal prediction contracts, independent of ROS message definitions."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import TypeAlias


Identifier: TypeAlias = int | str
Point2: TypeAlias = tuple[float, float]
Vector2: TypeAlias = tuple[float, float]
Vector3: TypeAlias = tuple[float, float, float]


def _require_finite(name: str, *values: float) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} must contain only finite values")


def _require_optional_finite(name: str, value: float | None) -> None:
    if value is not None:
        _require_finite(name, value)


@dataclass(frozen=True)
class TrajectoryStep:
    step_id: int
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        _require_finite("trajectory step coordinates/yaw", self.x, self.y, self.yaw)


@dataclass(frozen=True)
class Trajectory:
    timestamp: float
    frame_id: str
    steps: list[TrajectoryStep]

    def __post_init__(self) -> None:
        _require_finite("trajectory timestamp", self.timestamp)
        if not self.frame_id.strip():
            raise ValueError("trajectory frame_id must not be empty")
        if not self.steps:
            raise ValueError("trajectory must contain at least one step")
        ids = [step.step_id for step in self.steps]
        if len(ids) != len(set(ids)):
            raise ValueError("trajectory step_id values must be unique")

    def cumulative_distances_m(self) -> list[float]:
        """Return planar route distance at every step, measured from steps[0]."""
        distances = [0.0]
        for previous, current in zip(self.steps, self.steps[1:]):
            distances.append(
                distances[-1] + math.hypot(current.x - previous.x, current.y - previous.y)
            )
        return distances


@dataclass(frozen=True)
class TrackedObject:
    timestamp: float
    track_id: Identifier
    class_name: str
    footprint_polygon_xy: list[Point2]
    height_m: float | None = None
    velocity_xy: Vector2 | None = None
    confidence: float | None = None

    def __post_init__(self) -> None:
        _require_finite("tracked object timestamp", self.timestamp)
        if not isinstance(self.track_id, (int, str)) or isinstance(self.track_id, bool):
            raise ValueError("track_id must be an int or str")
        if not self.class_name.strip():
            raise ValueError("tracked object class_name must not be empty")
        if len(self.footprint_polygon_xy) < 3:
            raise ValueError(f"object {self.track_id!r} polygon requires at least 3 points")
        for point in self.footprint_polygon_xy:
            if len(point) != 2:
                raise ValueError(f"object {self.track_id!r} polygon points must be (x, y)")
            _require_finite(f"object {self.track_id!r} polygon", *point)
        if len(set(self.footprint_polygon_xy)) < 3:
            raise ValueError(f"object {self.track_id!r} polygon requires 3 unique points")
        _require_optional_finite("tracked object height_m", self.height_m)
        if self.height_m is not None and self.height_m < 0:
            raise ValueError("tracked object height_m must be non-negative")
        if self.velocity_xy is not None:
            _require_finite("tracked object velocity_xy", *self.velocity_xy)
        _require_optional_finite("tracked object confidence", self.confidence)


@dataclass(frozen=True)
class GeometryStep:
    timestamp: float
    step_id: int
    plane_id: Identifier
    normal_xyz: Vector3
    centroid_xyz: Vector3 | None = None
    confidence: float | None = None

    def __post_init__(self) -> None:
        _require_finite("geometry timestamp", self.timestamp)
        _require_finite("terrain normal", *self.normal_xyz)
        if math.sqrt(sum(component * component for component in self.normal_xyz)) <= 1e-12:
            raise ValueError(f"geometry step {self.step_id} has a zero normal")
        if self.centroid_xyz is not None:
            _require_finite("geometry centroid_xyz", *self.centroid_xyz)
        _require_optional_finite("geometry confidence", self.confidence)


@dataclass(frozen=True)
class RoverState:
    timestamp: float
    x: float | None = None
    y: float | None = None
    yaw: float | None = None
    roll: float | None = None
    pitch: float | None = None
    velocity_xy: Vector2 | None = None
    acceleration_xy: Vector2 | None = None
    angular_velocity_xyz: Vector3 | None = None

    def __post_init__(self) -> None:
        _require_finite("rover state timestamp", self.timestamp)
        for name in ("x", "y", "yaw", "roll", "pitch"):
            _require_optional_finite(f"rover state {name}", getattr(self, name))
        for name in ("velocity_xy", "acceleration_xy", "angular_velocity_xyz"):
            value = getattr(self, name)
            if value is not None:
                _require_finite(f"rover state {name}", *value)


@dataclass(frozen=True)
class CollisionObject:
    object_id: Identifier
    object_class: str
    min_distance_m: float
    confidence_or_validity: float | None


@dataclass(frozen=True)
class CollisionStep:
    step_id: int
    distance_along_route_m: float
    collision_objects: list[CollisionObject]


@dataclass(frozen=True)
class RolloverStep:
    step_id: int
    predicted_roll_deg: float
    predicted_pitch_deg: float
    static_stability_margin_m: float
    normalized_static_stability_margin: float
    terrain_id: Identifier
    confidence_or_validity: float | None


@dataclass(frozen=True)
class PredictionOutput:
    timestamp: float
    source_trajectory_stamp: float
    collision_steps: list[CollisionStep] = field(default_factory=list)
    rollover_steps: list[RolloverStep] = field(default_factory=list)

