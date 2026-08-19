"""Top-down collision evidence visualization."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from shapely.ops import nearest_points

from prediction_core.collision import validated_object_polygon
from prediction_core.config import RoverConfig
from prediction_core.geometry_utils import rover_rectangle
from prediction_core.models import PredictionOutput, TrackedObject, Trajectory


def save_collision_topdown(
    trajectory: Trajectory,
    tracked_objects: list[TrackedObject],
    config: RoverConfig,
    output: PredictionOutput,
    path: str | Path,
) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(figsize=(10, 7))
    axes.plot(
        [step.x for step in trajectory.steps],
        [step.y for step in trajectory.steps],
        color="tab:blue",
        marker="o",
        markersize=3,
        label="trajectory",
    )
    collision_ids = {item.step_id for item in output.collision_steps}
    collision_lookup = {item.step_id: item for item in output.collision_steps}
    object_lookup = {obj.track_id: obj for obj in tracked_objects}

    for step in trajectory.steps:
        physical = rover_rectangle(step, config.body_length_m, config.body_width_m)
        safety = rover_rectangle(
            step,
            config.body_length_m,
            config.body_width_m,
            config.prediction.collision_margin_m,
        )
        sx, sy = safety.exterior.xy
        axes.plot(sx, sy, color="tab:orange", alpha=0.18, linewidth=0.8)
        px, py = physical.exterior.xy
        axes.plot(
            px,
            py,
            color="red" if step.step_id in collision_ids else "tab:blue",
            alpha=0.55,
            linewidth=1.0,
        )
        if step.step_id in collision_ids:
            axes.scatter(step.x, step.y, color="red", s=28, zorder=5)

    for tracked_object in tracked_objects:
        polygon = validated_object_polygon(tracked_object)
        ox, oy = polygon.exterior.xy
        axes.fill(ox, oy, color="tab:purple", alpha=0.28)
        axes.plot(ox, oy, color="tab:purple", linewidth=1.4)
        center = polygon.centroid
        axes.text(center.x, center.y, f"{tracked_object.track_id}: {tracked_object.class_name}")

    # Draw exact nearest-boundary segments for every reported candidate.
    steps_by_id = {step.step_id: step for step in trajectory.steps}
    for step_id, collision_step in collision_lookup.items():
        rover = rover_rectangle(
            steps_by_id[step_id], config.body_length_m, config.body_width_m
        )
        for candidate in collision_step.collision_objects:
            object_polygon = validated_object_polygon(object_lookup[candidate.object_id])
            rover_point, object_point = nearest_points(rover, object_polygon)
            axes.plot(
                [rover_point.x, object_point.x],
                [rover_point.y, object_point.y],
                color="black",
                linestyle=":",
                linewidth=1.0,
            )

    axes.set_title("Collision prediction: physical and expanded rover footprints")
    axes.set_xlabel("World X / East (m)")
    axes.set_ylabel("World Y / North (m)")
    axes.set_aspect("equal", adjustable="datalim")
    axes.grid(True, alpha=0.25)
    axes.legend(loc="best")
    figure.tight_layout()
    figure.savefig(destination, dpi=150)
    plt.close(figure)

