import math

import pytest

from prediction_core.collision import CollisionPredictor
from prediction_core.config import RoverConfig
from prediction_core.models import TrackedObject, Trajectory, TrajectoryStep


def trajectory(*steps: TrajectoryStep) -> Trajectory:
    return Trajectory(10.0, "map", list(steps) or [TrajectoryStep(0, 0.0, 0.0, 0.0)])


def rectangle_object(
    object_id: str, x_min: float, y_min: float, x_max: float, y_max: float
) -> TrackedObject:
    return TrackedObject(
        timestamp=10.0,
        track_id=object_id,
        class_name="debris",
        footprint_polygon_xy=[
            (x_min, y_min),
            (x_max, y_min),
            (x_max, y_max),
            (x_min, y_max),
        ],
        confidence=0.8,
    )


def predict(mock_config: RoverConfig, objects: list[TrackedObject], *steps: TrajectoryStep):
    return CollisionPredictor(mock_config).predict(trajectory(*steps), objects)


def test_no_objects(mock_config: RoverConfig) -> None:
    assert predict(mock_config, []) == []


def test_direct_overlap_distance_is_zero(mock_config: RoverConfig) -> None:
    result = predict(mock_config, [rectangle_object("overlap", -0.1, -0.1, 0.1, 0.1)])
    assert result[0].collision_objects[0].min_distance_m == pytest.approx(0.0)


@pytest.mark.parametrize(
    ("gap", "expected"), [(0.20, True), (0.199, True), (0.201, False)]
)
def test_margin_boundary(mock_config: RoverConfig, gap: float, expected: bool) -> None:
    body_upper_edge = mock_config.body_width_m / 2
    obj = rectangle_object("margin", -0.1, body_upper_edge + gap, 0.1, 0.8 + gap)
    result = predict(mock_config, [obj])
    assert bool(result) is expected
    if expected:
        assert result[0].collision_objects[0].min_distance_m == pytest.approx(gap)


def test_zero_margin_only_reports_touch_or_overlap(mock_config: RoverConfig) -> None:
    zero_margin = RoverConfig(
        mass_kg=mock_config.mass_kg,
        body_length_m=mock_config.body_length_m,
        body_width_m=mock_config.body_width_m,
        body_height_m=mock_config.body_height_m,
        support_length_m=mock_config.support_length_m,
        support_width_m=mock_config.support_width_m,
        ground_clearance_m=mock_config.ground_clearance_m,
        com_x_m=mock_config.com_x_m,
        com_y_m=mock_config.com_y_m,
        com_height_m=mock_config.com_height_m,
        prediction=type(mock_config.prediction)(0.0),
    )
    body_upper_edge = mock_config.body_width_m / 2
    touching = rectangle_object("touch", -0.1, body_upper_edge, 0.1, 0.6)
    separated = rectangle_object("gap", 0.2, body_upper_edge + 0.001, 0.3, 0.6)
    assert len(predict(zero_margin, [touching])) == 1
    assert predict(zero_margin, [separated]) == []


def test_multiple_objects_in_one_step(mock_config: RoverConfig) -> None:
    result = predict(
        mock_config,
        [
            rectangle_object("a", -0.2, -0.2, 0.2, 0.2),
            rectangle_object("b", 0.65, -0.1, 0.75, 0.1),
        ],
    )
    assert {candidate.object_id for candidate in result[0].collision_objects} == {"a", "b"}


def test_same_object_can_collide_across_steps(mock_config: RoverConfig) -> None:
    obj = rectangle_object("long", -0.1, -0.2, 1.1, 0.2)
    result = predict(
        mock_config,
        [obj],
        TrajectoryStep(0, 0.0, 0.0, 0.0),
        TrajectoryStep(1, 0.5, 0.0, 0.0),
        TrajectoryStep(2, 1.0, 0.0, 0.0),
    )
    assert [step.step_id for step in result] == [0, 1, 2]


def test_rotated_rover(mock_config: RoverConfig) -> None:
    obj = rectangle_object("north", -0.1, 0.7, 0.1, 0.8)
    assert predict(mock_config, [obj], TrajectoryStep(0, 0.0, 0.0, 0.0)) == []
    rotated = predict(
        mock_config, [obj], TrajectoryStep(0, 0.0, 0.0, math.pi / 2)
    )
    assert rotated[0].collision_objects[0].min_distance_m == pytest.approx(0.175)


def test_curved_trajectory_evaluates_each_yaw(mock_config: RoverConfig) -> None:
    obj = rectangle_object("curve", 0.9, 0.6, 1.1, 0.8)
    result = predict(
        mock_config,
        [obj],
        TrajectoryStep(0, 0.0, 0.0, 0.0),
        TrajectoryStep(1, 1.0, 0.0, math.pi / 2),
        TrajectoryStep(2, 1.0, 1.0, math.pi / 2),
    )
    assert {step.step_id for step in result} == {1, 2}
    assert result[-1].distance_along_route_m == pytest.approx(2.0)


def test_self_intersecting_polygon_rejected(mock_config: RoverConfig) -> None:
    bow_tie = TrackedObject(
        10.0, "bad", "debris", [(-1.0, -1.0), (1.0, 1.0), (-1.0, 1.0), (1.0, -1.0)]
    )
    with pytest.raises(ValueError, match="invalid"):
        predict(mock_config, [bow_tie])


def test_duplicate_object_ids_rejected(mock_config: RoverConfig) -> None:
    with pytest.raises(ValueError, match="unique"):
        predict(
            mock_config,
            [
                rectangle_object("same", -1, -1, 0, 0),
                rectangle_object("same", 1, 1, 2, 2),
            ],
        )

