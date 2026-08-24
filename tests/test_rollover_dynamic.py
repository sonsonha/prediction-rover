"""Extended rollover evidence: tip angles, effective SSM, moments, ZMP."""

from __future__ import annotations

import math

import pytest

from prediction_core.config import RoverConfig
from prediction_core.models import (
    ExternalWrench,
    GeometryStep,
    RoverState,
    Trajectory,
    TrajectoryStep,
)
from prediction_core.rollover import (
    GRAVITY_WORLD_M_S2,
    RolloverPredictor,
    compute_critical_tip_evidence,
)


def _flat_case(config: RoverConfig, *, yaw: float = 0.0):
    trajectory = Trajectory(
        timestamp=1.0,
        frame_id="map",
        steps=[TrajectoryStep(0, 0.0, 0.0, yaw)],
    )
    geometry = [
        GeometryStep(
            timestamp=1.0,
            step_id=0,
            plane_id="flat",
            normal_xyz=(0.0, 0.0, 1.0),
            confidence=1.0,
        )
    ]
    return RolloverPredictor(config), trajectory, geometry


def test_critical_tip_angles_match_geometry(mock_config: RoverConfig) -> None:
    tip = compute_critical_tip_evidence(mock_config)
    lateral = math.degrees(
        math.atan((mock_config.support_width_m / 2.0) / mock_config.com_height_m)
    )
    longitudinal = math.degrees(
        math.atan((mock_config.support_length_m / 2.0) / mock_config.com_height_m)
    )
    assert tip.left_deg == pytest.approx(lateral)
    assert tip.right_deg == pytest.approx(lateral)
    assert tip.front_deg == pytest.approx(longitudinal)
    assert tip.rear_deg == pytest.approx(longitudinal)
    assert tip.minimum_deg == pytest.approx(min(lateral, longitudinal))
    assert tip.critical_edge in {"front", "rear"}


def test_static_regression_unchanged_with_extended_fields(mock_config: RoverConfig) -> None:
    predictor, trajectory, geometry = _flat_case(mock_config)
    step = predictor.predict(trajectory, geometry)[0]
    assert step.predicted_roll_deg == pytest.approx(0.0)
    assert step.predicted_pitch_deg == pytest.approx(0.0)
    assert step.static_stability_margin_m == pytest.approx(0.375)
    assert step.normalized_static_stability_margin == pytest.approx(1.0)
    assert step.critical_tip is not None
    assert step.dynamic_stability is not None
    assert step.dynamic_stability.valid is False
    assert step.dynamic_stability.acceleration_available is False


def test_zero_acceleration_matches_static(mock_config: RoverConfig) -> None:
    predictor, trajectory, geometry = _flat_case(mock_config)
    state = RoverState(timestamp=1.0, acceleration_xyz=(0.0, 0.0, 0.0))
    step = predictor.predict(
        trajectory, geometry, state=state, external_wrenches=[]
    )[0]
    dynamic = step.dynamic_stability
    assert dynamic is not None and dynamic.valid
    assert dynamic.effective_ssm_m == pytest.approx(step.static_stability_margin_m)
    assert dynamic.normalized_effective_ssm == pytest.approx(
        step.normalized_static_stability_margin
    )
    assert dynamic.zmp_margin_m == pytest.approx(step.static_stability_margin_m)
    assert dynamic.normalized_zmp_margin == pytest.approx(1.0)
    assert dynamic.normalized_minimum_stability_moment == pytest.approx(1.0)
    assert dynamic.zmp_xy[0] == pytest.approx(0.0, abs=1e-9)
    assert dynamic.zmp_xy[1] == pytest.approx(0.0, abs=1e-9)
    assert dynamic.external_wrench_included is True


def test_missing_acceleration_keeps_static_only(mock_config: RoverConfig) -> None:
    predictor, trajectory, geometry = _flat_case(mock_config)
    step = predictor.predict(
        trajectory,
        geometry,
        state=RoverState(timestamp=1.0),
        external_wrenches=[],
    )[0]
    assert step.static_stability_margin_m == pytest.approx(0.375)
    assert step.critical_tip is not None
    dynamic = step.dynamic_stability
    assert dynamic is not None
    assert dynamic.acceleration_available is False
    assert dynamic.valid is False
    assert dynamic.effective_ssm_m is None
    assert dynamic.zmp_xy is None
    assert dynamic.edge_stability_moments_nm is None
    assert dynamic.external_wrench_available is True
    assert dynamic.external_wrench_included is False


def test_external_wrenches_none_vs_empty(mock_config: RoverConfig) -> None:
    predictor, trajectory, geometry = _flat_case(mock_config)
    state = RoverState(timestamp=1.0, acceleration_xyz=(0.0, 0.0, 0.0))
    without = predictor.predict(trajectory, geometry, state=state, external_wrenches=None)[0]
    empty = predictor.predict(trajectory, geometry, state=state, external_wrenches=[])[0]
    assert without.dynamic_stability is not None
    assert empty.dynamic_stability is not None
    assert without.dynamic_stability.external_wrench_available is False
    assert without.dynamic_stability.external_wrench_included is False
    assert empty.dynamic_stability.external_wrench_available is True
    assert empty.dynamic_stability.external_wrench_included is True
    assert without.dynamic_stability.effective_ssm_m == pytest.approx(
        empty.dynamic_stability.effective_ssm_m
    )


def test_lateral_acceleration_shifts_toward_expected_edge(mock_config: RoverConfig) -> None:
    predictor, trajectory, geometry = _flat_case(mock_config, yaw=0.0)
    # +Y acceleration (map left): d'Alembert force -m a pushes support point to -Y (right).
    leftward = predictor.predict(
        trajectory,
        geometry,
        state=RoverState(timestamp=1.0, acceleration_xyz=(0.0, 2.0, 0.0)),
        external_wrenches=[],
    )[0].dynamic_stability
    rightward = predictor.predict(
        trajectory,
        geometry,
        state=RoverState(timestamp=1.0, acceleration_xyz=(0.0, -2.0, 0.0)),
        external_wrenches=[],
    )[0].dynamic_stability
    assert leftward is not None and rightward is not None
    assert leftward.effective_gravity_projection_xy[1] < 0.0
    assert rightward.effective_gravity_projection_xy[1] > 0.0
    assert leftward.critical_edge == "right"
    assert rightward.critical_edge == "left"
    assert leftward.effective_ssm_m < 0.375
    assert rightward.effective_ssm_m < 0.375
    assert leftward.zmp_margin_m < 0.375
    assert leftward.edge_stability_moments_nm["right"] < leftward.edge_stability_moments_nm["left"]


def test_braking_and_forward_acceleration_flip_longitudinal_edge(
    mock_config: RoverConfig,
) -> None:
    predictor, trajectory, geometry = _flat_case(mock_config, yaw=0.0)
    # +X accel (forward): inertial force -X tips toward rear.
    accel = predictor.predict(
        trajectory,
        geometry,
        state=RoverState(timestamp=1.0, acceleration_xyz=(2.0, 0.0, 0.0)),
        external_wrenches=[],
    )[0].dynamic_stability
    brake = predictor.predict(
        trajectory,
        geometry,
        state=RoverState(timestamp=1.0, acceleration_xyz=(-2.0, 0.0, 0.0)),
        external_wrenches=[],
    )[0].dynamic_stability
    assert accel is not None and brake is not None
    assert accel.effective_gravity_projection_xy[0] < 0.0
    assert brake.effective_gravity_projection_xy[0] > 0.0
    assert accel.critical_edge == "rear"
    assert brake.critical_edge == "front"


def test_external_force_height_increases_overturning(mock_config: RoverConfig) -> None:
    predictor, trajectory, geometry = _flat_case(mock_config)
    state = RoverState(timestamp=1.0, acceleration_xyz=(0.0, 0.0, 0.0))
    low = ExternalWrench(
        source="push_low",
        force_xyz=(0.0, 200.0, 0.0),
        torque_xyz=(0.0, 0.0, 0.0),
        application_point_xyz=(0.0, 0.0, 0.05),
    )
    high = ExternalWrench(
        source="push_high",
        force_xyz=(0.0, 200.0, 0.0),
        torque_xyz=(0.0, 0.0, 0.0),
        application_point_xyz=(0.0, 0.0, 0.80),
    )
    low_step = predictor.predict(
        trajectory, geometry, state=state, external_wrenches=[low]
    )[0].dynamic_stability
    high_step = predictor.predict(
        trajectory, geometry, state=state, external_wrenches=[high]
    )[0].dynamic_stability
    assert low_step is not None and high_step is not None
    assert high_step.minimum_stability_moment_nm < low_step.minimum_stability_moment_nm
    assert high_step.zmp_margin_m < low_step.zmp_margin_m


def test_pure_external_torque_moves_zmp_and_moment(mock_config: RoverConfig) -> None:
    predictor, trajectory, geometry = _flat_case(mock_config)
    state = RoverState(timestamp=1.0, acceleration_xyz=(0.0, 0.0, 0.0))
    torque = ExternalWrench(
        source="couple",
        force_xyz=(0.0, 0.0, 0.0),
        torque_xyz=(50.0, 0.0, 0.0),  # about +X => affects left/right
    )
    dynamic = predictor.predict(
        trajectory, geometry, state=state, external_wrenches=[torque]
    )[0].dynamic_stability
    assert dynamic is not None and dynamic.valid
    assert dynamic.zmp_xy is not None
    assert abs(dynamic.zmp_xy[1]) > 1e-6
    assert dynamic.normalized_minimum_stability_moment < 1.0


def test_beyond_tipping_produces_negative_margins(mock_config: RoverConfig) -> None:
    predictor, trajectory, geometry = _flat_case(mock_config)
    # Large lateral acceleration to push ZMP outside the support polygon.
    dynamic = predictor.predict(
        trajectory,
        geometry,
        state=RoverState(timestamp=1.0, acceleration_xyz=(0.0, 20.0, 0.0)),
        external_wrenches=[],
    )[0].dynamic_stability
    assert dynamic is not None and dynamic.valid
    assert dynamic.zmp_margin_m is not None and dynamic.zmp_margin_m < 0.0
    assert dynamic.minimum_stability_moment_nm is not None
    assert dynamic.minimum_stability_moment_nm < 0.0
    assert dynamic.normalized_zmp_margin is not None and dynamic.normalized_zmp_margin < 0.0


def test_zmp_and_moment_critical_edges_agree_for_pure_translation(
    mock_config: RoverConfig,
) -> None:
    predictor, trajectory, geometry = _flat_case(mock_config)
    dynamic = predictor.predict(
        trajectory,
        geometry,
        state=RoverState(timestamp=1.0, acceleration_xyz=(0.0, 1.5, 0.0)),
        external_wrenches=[],
    )[0].dynamic_stability
    assert dynamic is not None and dynamic.valid
    assert dynamic.critical_edge == "right"
    # Criticality is defined by normalized restoring moment, not raw N·m.
    refs = {
        "front": mock_config.mass_kg * 9.80665 * (mock_config.support_length_m / 2),
        "rear": mock_config.mass_kg * 9.80665 * (mock_config.support_length_m / 2),
        "left": mock_config.mass_kg * 9.80665 * (mock_config.support_width_m / 2),
        "right": mock_config.mass_kg * 9.80665 * (mock_config.support_width_m / 2),
    }
    normalized = {
        edge: dynamic.edge_stability_moments_nm[edge] / refs[edge]
        for edge in refs
    }
    assert min(normalized, key=normalized.get) == "right"


def test_force_without_application_point_is_not_invented(mock_config: RoverConfig) -> None:
    predictor, trajectory, geometry = _flat_case(mock_config)
    state = RoverState(timestamp=1.0, acceleration_xyz=(0.0, 0.0, 0.0))
    wrench = ExternalWrench(
        source="unknown_point",
        force_xyz=(0.0, 300.0, 0.0),
        torque_xyz=(0.0, 0.0, 0.0),
        application_point_xyz=None,
    )
    dynamic = predictor.predict(
        trajectory, geometry, state=state, external_wrenches=[wrench]
    )[0].dynamic_stability
    assert dynamic is not None and dynamic.valid
    # Without a point, force cannot contribute a moment arm; margins stay near static.
    assert dynamic.normalized_minimum_stability_moment == pytest.approx(1.0, abs=1e-9)
    assert any("application_point_xyz missing" in item for item in dynamic.assumptions)


def test_gravity_constant_documented(mock_config: RoverConfig) -> None:
    del mock_config
    assert GRAVITY_WORLD_M_S2 == (0.0, 0.0, -9.80665)
