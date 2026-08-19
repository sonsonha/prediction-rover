import math
from dataclasses import replace

import pytest

from prediction_core.config import RoverConfig
from prediction_core.models import GeometryStep, Trajectory, TrajectoryStep
from prediction_core.rollover import RolloverPredictor


def normal_x(degrees: float, scale: float = 1.0) -> tuple[float, float, float]:
    angle = math.radians(degrees)
    return -scale * math.sin(angle), 0.0, scale * math.cos(angle)


def result(mock_config: RoverConfig, degrees: float, yaw: float = 0.0, scale: float = 1.0):
    trajectory = Trajectory(1.0, "map", [TrajectoryStep(0, 0.0, 0.0, yaw)])
    geometry = [GeometryStep(1.0, 0, "plane", normal_x(degrees, scale), confidence=0.9)]
    return RolloverPredictor(mock_config).predict(trajectory, geometry)[0]


def test_flat_roll_pitch_and_ssm(mock_config: RoverConfig) -> None:
    output = result(mock_config, 0)
    assert output.predicted_roll_deg == pytest.approx(0.0)
    assert output.predicted_pitch_deg == pytest.approx(0.0)
    assert output.static_stability_margin_m == pytest.approx(
        min(mock_config.support_length_m / 2, mock_config.support_width_m / 2)
    )
    assert output.normalized_static_stability_margin == pytest.approx(1.0)


def test_uphill_and_downhill_sign(mock_config: RoverConfig) -> None:
    assert result(mock_config, 15).predicted_pitch_deg == pytest.approx(15.0)
    assert result(mock_config, -15).predicted_pitch_deg == pytest.approx(-15.0)


def test_uphill_15_raw_and_normalized_ssm(mock_config: RoverConfig) -> None:
    output = result(mock_config, 15)
    assert output.static_stability_margin_m == pytest.approx(0.2865767665)
    assert output.normalized_static_stability_margin == pytest.approx(0.7642047107)


def test_left_and_right_side_slope_sign(mock_config: RoverConfig) -> None:
    assert result(mock_config, 15, math.pi / 2).predicted_roll_deg == pytest.approx(-15.0)
    assert result(mock_config, -15, math.pi / 2).predicted_roll_deg == pytest.approx(15.0)


def test_side_slope_15_raw_and_normalized_ssm(mock_config: RoverConfig) -> None:
    output = result(mock_config, 15, math.pi / 2)
    assert output.static_stability_margin_m == pytest.approx(0.3515767665)
    assert output.normalized_static_stability_margin == pytest.approx(0.7990381057)


def test_yaw_rotation_changes_pitch_into_roll(mock_config: RoverConfig) -> None:
    output = result(mock_config, 15, math.pi / 2)
    assert abs(output.predicted_roll_deg) == pytest.approx(15.0)
    assert output.predicted_pitch_deg == pytest.approx(0.0, abs=1e-10)


def test_normal_magnitude_does_not_change_result(mock_config: RoverConfig) -> None:
    unit = result(mock_config, 20, scale=1.0)
    scaled = result(mock_config, 20, scale=7.5)
    assert scaled.predicted_pitch_deg == pytest.approx(unit.predicted_pitch_deg)
    assert scaled.static_stability_margin_m == pytest.approx(unit.static_stability_margin_m)


def test_near_vertical_normal_rejected(mock_config: RoverConfig) -> None:
    trajectory = Trajectory(1.0, "map", [TrajectoryStep(0, 0.0, 0.0, 0.0)])
    geometry = [GeometryStep(1.0, 0, "wall", (1.0, 0.0, 1e-10))]
    with pytest.raises(ValueError, match="near-vertical"):
        RolloverPredictor(mock_config).predict(trajectory, geometry)


def test_ssm_decreases_with_side_slope(mock_config: RoverConfig) -> None:
    margins = [
        result(mock_config, degrees, math.pi / 2).static_stability_margin_m
        for degrees in (0, 5, 10, 15, 20, 25)
    ]
    # At shallow slope the longitudinal edge remains closest, so the margin is
    # physically flat until lateral gravity projection becomes the limiting edge.
    assert all(current >= following for current, following in zip(margins, margins[1:]))
    assert margins[2] > margins[3] > margins[4] > margins[5]
    assert all(margin > 0 for margin in margins)


def test_normalized_ssm_decreases_immediately_with_side_slope(mock_config: RoverConfig) -> None:
    margins = [
        result(mock_config, degrees, math.pi / 2).normalized_static_stability_margin
        for degrees in (0, 5, 10, 15, 20, 25)
    ]
    assert all(current > following for current, following in zip(margins, margins[1:]))


def test_side_slope_ssm_zero_at_geometric_tip_boundary(mock_config: RoverConfig) -> None:
    tipping_angle = math.degrees(
        math.atan(
            (mock_config.support_width_m / 2 - abs(mock_config.com_y_m))
            / mock_config.com_height_m
        )
    )
    output = result(mock_config, tipping_angle, math.pi / 2)
    assert output.static_stability_margin_m == pytest.approx(0.0, abs=1e-9)
    assert output.normalized_static_stability_margin == pytest.approx(0.0, abs=1e-9)


def test_longitudinal_ssm_zero_at_geometric_tip_boundary(mock_config: RoverConfig) -> None:
    tipping_angle = math.degrees(
        math.atan(
            (mock_config.support_length_m / 2 - abs(mock_config.com_x_m))
            / mock_config.com_height_m
        )
    )
    output = result(mock_config, tipping_angle)
    assert output.static_stability_margin_m == pytest.approx(0.0, abs=1e-9)
    assert output.normalized_static_stability_margin == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("yaw", (0.0, math.pi / 2))
def test_normalized_ssm_is_negative_beyond_tipping(
    mock_config: RoverConfig, yaw: float
) -> None:
    axis_reference = (
        mock_config.support_length_m / 2 if yaw == 0 else mock_config.support_width_m / 2
    )
    tipping_angle = math.degrees(math.atan(axis_reference / mock_config.com_height_m))
    output = result(mock_config, tipping_angle + 1.0, yaw)
    assert output.normalized_static_stability_margin < 0


def test_offset_com_uses_asymmetric_edge_references(mock_config: RoverConfig) -> None:
    offset_config = replace(mock_config, com_x_m=0.05, com_y_m=-0.03)
    flat = result(offset_config, 0)
    assert flat.normalized_static_stability_margin == pytest.approx(1.0)

    uphill = result(offset_config, 15)
    expected_rear_ratio = (
        offset_config.support_length_m / 2
        - offset_config.com_height_m * math.tan(math.radians(15))
        + offset_config.com_x_m
    ) / (offset_config.support_length_m / 2 + offset_config.com_x_m)
    assert uphill.normalized_static_stability_margin == pytest.approx(expected_rear_ratio)


@pytest.mark.parametrize(
    ("com_x_m", "com_y_m"),
    ((0.375, 0.0), (0.0, -0.44), (0.376, 0.0), (0.0, 0.441)),
)
def test_com_on_or_outside_support_edge_is_rejected(
    mock_config: RoverConfig, com_x_m: float, com_y_m: float
) -> None:
    with pytest.raises(ValueError, match="CoM"):
        replace(mock_config, com_x_m=com_x_m, com_y_m=com_y_m)

