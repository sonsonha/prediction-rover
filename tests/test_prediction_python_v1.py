"""Prediction Python V1 regression invariants (evidence only; no Decision)."""

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
from prediction_core.rollover import RolloverPredictor, compute_critical_tip_evidence
from prediction_core.version import PREDICTION_PYTHON_VERSION


def _normal(roll_deg: float = 0.0, pitch_deg: float = 0.0) -> tuple[float, float, float]:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    nx = -math.tan(pitch)
    ny = -math.tan(roll)
    nz = 1.0
    mag = math.hypot(math.hypot(nx, ny), nz)
    return nx / mag, ny / mag, nz / mag


def _predict(
    config: RoverConfig,
    *,
    normal: tuple[float, float, float],
    accel: tuple[float, float, float] | None,
    wrenches: list[ExternalWrench] | None = None,
    yaw: float = 0.0,
):
    predictor = RolloverPredictor(config)
    trajectory = Trajectory(
        timestamp=1.0,
        frame_id="map",
        steps=[TrajectoryStep(0, 0.0, 0.0, yaw)],
    )
    geometry = [
        GeometryStep(
            timestamp=1.0,
            step_id=0,
            plane_id="v1",
            normal_xyz=normal,
            confidence=1.0,
        )
    ]
    state = RoverState(timestamp=1.0, acceleration_xyz=accel)
    return predictor.predict(
        trajectory, geometry, state=state, external_wrenches=wrenches
    )[0]


def test_version_marker():
    assert PREDICTION_PYTHON_VERSION == "1.0"


def test_flat_static_baseline(mock_config: RoverConfig) -> None:
    step = _predict(mock_config, normal=(0.0, 0.0, 1.0), accel=(0.0, 0.0, 0.0), wrenches=[])
    assert step.predicted_roll_deg == pytest.approx(0.0, abs=1e-9)
    assert step.predicted_pitch_deg == pytest.approx(0.0, abs=1e-9)
    assert step.static_stability_margin_m == pytest.approx(0.375, abs=1e-9)
    assert step.normalized_static_stability_margin == pytest.approx(1.0, abs=1e-9)
    assert step.nearest_static_edge in {"front", "rear"}
    dyn = step.dynamic_stability
    assert dyn is not None and dyn.valid
    assert dyn.normalized_minimum_stability_moment == pytest.approx(1.0, abs=1e-9)
    assert dyn.minimum_normalized_moment_edge == dyn.critical_edge


def test_side_slope_15deg_baseline(mock_config: RoverConfig) -> None:
    step = _predict(
        mock_config, normal=_normal(roll_deg=-15.0), accel=(0.0, 0.0, 0.0), wrenches=[]
    )
    assert step.predicted_roll_deg == pytest.approx(-15.0, abs=1e-6)
    assert step.static_stability_margin_m == pytest.approx(0.3515767664977295, abs=1e-9)
    assert step.normalized_static_stability_margin == pytest.approx(0.799038105676658, abs=1e-9)


def test_uphill_15deg_baseline(mock_config: RoverConfig) -> None:
    step = _predict(
        mock_config, normal=_normal(pitch_deg=15.0), accel=(0.0, 0.0, 0.0), wrenches=[]
    )
    assert step.predicted_pitch_deg == pytest.approx(15.0, abs=1e-6)
    assert step.static_stability_margin_m == pytest.approx(0.2865767664977295, abs=1e-9)
    assert step.normalized_static_stability_margin == pytest.approx(0.764204710660612, abs=1e-9)


def test_critical_tip_angles(mock_config: RoverConfig) -> None:
    tip = compute_critical_tip_evidence(mock_config)
    assert tip.front_deg == pytest.approx(48.65222278030633, abs=1e-9)
    assert tip.left_deg == pytest.approx(53.13010235415598, abs=1e-9)
    assert tip.minimum_tip_angle_edge == tip.critical_edge


def test_zero_accel_matches_static_projection(mock_config: RoverConfig) -> None:
    step = _predict(mock_config, normal=(0.0, 0.0, 1.0), accel=(0.0, 0.0, 0.0), wrenches=[])
    dyn = step.dynamic_stability
    assert dyn is not None and dyn.valid
    assert dyn.effective_ssm_m == pytest.approx(step.static_stability_margin_m)
    assert dyn.zmp_margin_m == pytest.approx(dyn.effective_ssm_m)


def test_acceleration_changes_moment_not_static(mock_config: RoverConfig) -> None:
    static = _predict(mock_config, normal=(0.0, 0.0, 1.0), accel=(0.0, 0.0, 0.0), wrenches=[])
    dynamic = _predict(mock_config, normal=(0.0, 0.0, 1.0), accel=(0.0, 2.0, 0.0), wrenches=[])
    assert dynamic.static_stability_margin_m == pytest.approx(static.static_stability_margin_m)
    assert dynamic.dynamic_stability is not None and static.dynamic_stability is not None
    assert (
        dynamic.dynamic_stability.minimum_stability_moment_nm
        < static.dynamic_stability.minimum_stability_moment_nm
    )


def test_external_force_height_increases_overturning(mock_config: RoverConfig) -> None:
    low = _predict(
        mock_config,
        normal=(0.0, 0.0, 1.0),
        accel=(0.0, 0.0, 0.0),
        wrenches=[
            ExternalWrench(
                source="low",
                force_xyz=(0.0, 250.0, 0.0),
                torque_xyz=(0.0, 0.0, 0.0),
                application_point_xyz=(0.0, 0.0, 0.05),
            )
        ],
    )
    high = _predict(
        mock_config,
        normal=(0.0, 0.0, 1.0),
        accel=(0.0, 0.0, 0.0),
        wrenches=[
            ExternalWrench(
                source="high",
                force_xyz=(0.0, 250.0, 0.0),
                torque_xyz=(0.0, 0.0, 0.0),
                application_point_xyz=(0.0, 0.0, 0.90),
            )
        ],
    )
    assert low.static_stability_margin_m == pytest.approx(high.static_stability_margin_m)
    assert low.dynamic_stability is not None and high.dynamic_stability is not None
    assert (
        high.dynamic_stability.minimum_stability_moment_nm
        < low.dynamic_stability.minimum_stability_moment_nm
    )
    assert high.dynamic_stability.zmp_margin_m < low.dynamic_stability.zmp_margin_m
    assert low.dynamic_stability.effective_ssm_m == high.dynamic_stability.effective_ssm_m


def test_pure_torque_moves_moment_not_static(mock_config: RoverConfig) -> None:
    base = _predict(mock_config, normal=(0.0, 0.0, 1.0), accel=(0.0, 0.0, 0.0), wrenches=[])
    torque = _predict(
        mock_config,
        normal=(0.0, 0.0, 1.0),
        accel=(0.0, 0.0, 0.0),
        wrenches=[
            ExternalWrench(
                source="couple",
                force_xyz=(0.0, 0.0, 0.0),
                torque_xyz=(80.0, 0.0, 0.0),
            )
        ],
    )
    assert torque.static_stability_margin_m == pytest.approx(base.static_stability_margin_m)
    assert torque.dynamic_stability is not None and base.dynamic_stability is not None
    assert torque.dynamic_stability.effective_ssm_m == base.dynamic_stability.effective_ssm_m
    assert torque.dynamic_stability.zmp_xy != base.dynamic_stability.zmp_xy


def test_missing_acceleration_keeps_static_invalidates_dynamic(
    mock_config: RoverConfig,
) -> None:
    step = _predict(mock_config, normal=(0.0, 0.0, 1.0), accel=None, wrenches=[])
    assert step.static_stability_margin_m == pytest.approx(0.375)
    assert step.dynamic_stability is not None
    assert step.dynamic_stability.valid is False
    assert step.dynamic_stability.minimum_stability_moment_nm is None
    assert step.dynamic_stability.zmp_margin_m is None


def test_beyond_tip_signed_evidence_negative(mock_config: RoverConfig) -> None:
    step = _predict(
        mock_config, normal=(0.0, 0.0, 1.0), accel=(0.0, 20.0, 0.0), wrenches=[]
    )
    dyn = step.dynamic_stability
    assert dyn is not None and dyn.valid
    assert dyn.zmp_margin_m is not None and dyn.zmp_margin_m < 0.0
    assert dyn.minimum_stability_moment_nm is not None and dyn.minimum_stability_moment_nm < 0.0
