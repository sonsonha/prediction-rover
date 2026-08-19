"""Load deterministic JSON scenarios into the internal prediction contracts."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

from prediction_core.models import GeometryStep, TrackedObject, Trajectory, TrajectoryStep


@dataclass(frozen=True)
class Scenario:
    name: str
    trajectory: Trajectory
    tracked_objects: list[TrackedObject]
    geometry: list[GeometryStep]
    expectations: dict[str, Any]


def _point2(value: list[float]) -> tuple[float, float]:
    return float(value[0]), float(value[1])


def _point3(value: list[float]) -> tuple[float, float, float]:
    return float(value[0]), float(value[1]), float(value[2])


def load_scenario(path: str | Path) -> Scenario:
    scenario_path = Path(path)
    raw = json.loads(scenario_path.read_text(encoding="utf-8"))
    timestamp = float(raw["timestamp"])
    trajectory = Trajectory(
        timestamp=timestamp,
        frame_id=raw.get("frame_id", "map"),
        steps=[
            TrajectoryStep(
                step_id=int(step["step_id"]),
                x=float(step["x"]),
                y=float(step["y"]),
                yaw=float(step["yaw"]),
            )
            for step in raw["trajectory"]
        ],
    )
    objects = [
        TrackedObject(
            timestamp=float(obj.get("timestamp", timestamp)),
            track_id=obj["track_id"],
            class_name=obj["class_name"],
            footprint_polygon_xy=[_point2(point) for point in obj["footprint_polygon_xy"]],
            height_m=None if obj.get("height_m") is None else float(obj["height_m"]),
            velocity_xy=(
                None if obj.get("velocity_xy") is None else _point2(obj["velocity_xy"])
            ),
            confidence=None if obj.get("confidence") is None else float(obj["confidence"]),
        )
        for obj in raw.get("tracked_objects", [])
    ]
    geometry = [
        GeometryStep(
            timestamp=float(item.get("timestamp", timestamp)),
            step_id=int(item["step_id"]),
            plane_id=item["plane_id"],
            normal_xyz=_point3(item["normal_xyz"]),
            centroid_xyz=(
                None if item.get("centroid_xyz") is None else _point3(item["centroid_xyz"])
            ),
            confidence=(
                None if item.get("confidence") is None else float(item["confidence"])
            ),
        )
        for item in raw.get("geometry", [])
    ]
    # Compact deterministic mocks may apply one exact plane to every route step.
    if not geometry and "terrain_normal_xyz" in raw:
        normal = _point3(raw["terrain_normal_xyz"])
        plane_id = raw.get("terrain_plane_id", f"{raw.get('name', scenario_path.stem)}-plane")
        confidence = raw.get("terrain_confidence", 1.0)
        geometry = [
            GeometryStep(
                timestamp=timestamp,
                step_id=step.step_id,
                plane_id=plane_id,
                normal_xyz=normal,
                confidence=None if confidence is None else float(confidence),
            )
            for step in trajectory.steps
        ]
    return Scenario(
        name=raw.get("name", scenario_path.stem),
        trajectory=trajectory,
        tracked_objects=objects,
        geometry=geometry,
        expectations=raw.get("expectations", {}),
    )
