import math

import pytest

from prediction_core.config import PredictionConfig
from prediction_core.models import GeometryStep, TrackedObject, Trajectory, TrajectoryStep


def test_cumulative_distance_follows_curved_route() -> None:
    trajectory = Trajectory(
        timestamp=1.0,
        frame_id="map",
        steps=[
            TrajectoryStep(0, 0.0, 0.0, 0.0),
            TrajectoryStep(1, 3.0, 0.0, math.pi / 2),
            TrajectoryStep(2, 3.0, 4.0, math.pi / 2),
        ],
    )
    assert trajectory.cumulative_distances_m() == pytest.approx([0.0, 3.0, 7.0])


def test_empty_trajectory_rejected() -> None:
    with pytest.raises(ValueError, match="at least one step"):
        Trajectory(timestamp=1.0, frame_id="map", steps=[])


def test_nan_coordinate_rejected() -> None:
    with pytest.raises(ValueError, match="finite"):
        TrajectoryStep(0, math.nan, 0.0, 0.0)


def test_polygon_requires_three_unique_points() -> None:
    with pytest.raises(ValueError, match="3 unique"):
        TrackedObject(1.0, "bad", "debris", [(0.0, 0.0), (1.0, 0.0), (0.0, 0.0)])


def test_zero_normal_rejected() -> None:
    with pytest.raises(ValueError, match="zero normal"):
        GeometryStep(1.0, 0, "plane", (0.0, 0.0, 0.0))


def test_negative_collision_margin_rejected() -> None:
    with pytest.raises(ValueError, match="non-negative"):
        PredictionConfig(-0.01)

