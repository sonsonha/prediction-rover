import math

import pytest

from prediction_core.geometry_utils import rover_rectangle, terrain_roll_pitch_rad
from prediction_core.models import TrajectoryStep


def slope_normal_x(degrees: float) -> tuple[float, float, float]:
    angle = math.radians(degrees)
    return -math.sin(angle), 0.0, math.cos(angle)


def test_coordinate_and_attitude_conventions() -> None:
    roll, pitch = terrain_roll_pitch_rad((0.0, 0.0, 1.0), yaw=0.0)
    assert math.degrees(roll) == pytest.approx(0.0, abs=1e-10)
    assert math.degrees(pitch) == pytest.approx(0.0, abs=1e-10)

    roll, pitch = terrain_roll_pitch_rad(slope_normal_x(15), yaw=0.0)
    assert math.degrees(pitch) == pytest.approx(15.0)
    assert math.degrees(roll) == pytest.approx(0.0, abs=1e-10)

    # +90-degree yaw makes world +X point toward rover right, hence -roll.
    roll, pitch = terrain_roll_pitch_rad(slope_normal_x(15), yaw=math.pi / 2)
    assert math.degrees(roll) == pytest.approx(-15.0)
    assert math.degrees(pitch) == pytest.approx(0.0, abs=1e-10)

    _, downhill_pitch = terrain_roll_pitch_rad(slope_normal_x(-15), yaw=0.0)
    assert math.degrees(downhill_pitch) == pytest.approx(-15.0)

    opposite_roll, _ = terrain_roll_pitch_rad(slope_normal_x(-15), yaw=math.pi / 2)
    assert math.degrees(opposite_roll) == pytest.approx(15.0)


def test_expanded_rectangle_adds_margin_to_every_side() -> None:
    step = TrajectoryStep(0, 2.0, 3.0, math.pi / 3)
    physical = rover_rectangle(step, 1.2, 0.8)
    safe = rover_rectangle(step, 1.2, 0.8, 0.2)
    assert physical.area == pytest.approx(1.2 * 0.8)
    assert safe.area == pytest.approx(1.6 * 1.2)

