"""Invariant checks from the rollover method-comparison study (no new algorithms)."""

from __future__ import annotations

import math

import numpy as np

from prediction_core.geometry_utils import support_edge_margins, terrain_frame
from prediction_core.models import (
    ExternalWrench,
    GeometryStep,
    RoverState,
    Trajectory,
    TrajectoryStep,
)
from prediction_core.rollover import GRAVITY_WORLD_M_S2, RolloverPredictor


def _predict(predictor, *, normal, accel, wrenches):
    trajectory = Trajectory(
        timestamp=1.0,
        frame_id="map",
        steps=[TrajectoryStep(0, 0.0, 0.0, 0.0)],
    )
    geometry = [
        GeometryStep(
            timestamp=1.0,
            step_id=0,
            plane_id="cmp",
            normal_xyz=normal,
            confidence=1.0,
        )
    ]
    state = RoverState(timestamp=1.0, acceleration_xyz=accel)
    return predictor.predict(
        trajectory,
        geometry,
        state=state,
        external_wrenches=wrenches,
    )[0]


def test_effective_ssm_matches_zmp_for_gravity_plus_translational_accel(mock_config):
    predictor = RolloverPredictor(mock_config)
    cases = [
        ((0.0, 0.0, 1.0), (0.0, 0.0, 0.0)),
        ((0.0, 0.0, 1.0), (0.0, 2.0, 0.0)),
        ((0.0, 0.0, 1.0), (2.0, -1.0, 0.0)),
        ((-0.1, 0.2, math.sqrt(1.0 - 0.01 - 0.04)), (0.0, 1.5, 0.0)),
    ]
    for normal, accel in cases:
        step = _predict(predictor, normal=normal, accel=accel, wrenches=[])
        dyn = step.dynamic_stability
        assert dyn is not None and dyn.valid
        assert dyn.effective_gravity_projection_xy is not None
        assert dyn.zmp_xy is not None
        assert abs(dyn.effective_gravity_projection_xy[0] - dyn.zmp_xy[0]) < 1e-12
        assert abs(dyn.effective_gravity_projection_xy[1] - dyn.zmp_xy[1]) < 1e-12
        assert abs(dyn.effective_ssm_m - dyn.zmp_margin_m) < 1e-12


def test_moment_equals_minus_fz_times_zmp_edge_distance_wrench_free(mock_config):
    predictor = RolloverPredictor(mock_config)
    normal = (0.0, 0.0, 1.0)
    accel = (0.0, 3.0, 0.0)
    step = _predict(predictor, normal=normal, accel=accel, wrenches=[])
    dyn = step.dynamic_stability
    assert dyn is not None and dyn.valid and dyn.critical_edge is not None
    assert dyn.zmp_xy is not None and dyn.edge_stability_moments_nm is not None
    rotation = terrain_frame(normal, 0.0)
    f_support = rotation.T @ np.asarray(dyn.effective_force_xyz_n, dtype=float)
    fz = float(f_support[2])
    dist = getattr(
        support_edge_margins(
            dyn.zmp_xy, mock_config.support_length_m, mock_config.support_width_m
        ),
        f"{dyn.critical_edge}_m",
    )
    predicted = (-fz) * dist
    actual = dyn.edge_stability_moments_nm[dyn.critical_edge]
    assert abs(actual - predicted) < 1e-9


def test_external_force_height_moves_zmp_not_effective_ssm(mock_config):
    predictor = RolloverPredictor(mock_config)
    force = (0.0, 250.0, 0.0)
    low = _predict(
        predictor,
        normal=(0.0, 0.0, 1.0),
        accel=(0.0, 0.0, 0.0),
        wrenches=[
            ExternalWrench(
                source="low",
                force_xyz=force,
                torque_xyz=(0.0, 0.0, 0.0),
                application_point_xyz=(0.0, 0.0, 0.05),
            )
        ],
    )
    high = _predict(
        predictor,
        normal=(0.0, 0.0, 1.0),
        accel=(0.0, 0.0, 0.0),
        wrenches=[
            ExternalWrench(
                source="high",
                force_xyz=force,
                torque_xyz=(0.0, 0.0, 0.0),
                application_point_xyz=(0.0, 0.0, 0.90),
            )
        ],
    )
    assert low.dynamic_stability is not None and high.dynamic_stability is not None
    assert low.dynamic_stability.effective_ssm_m == high.dynamic_stability.effective_ssm_m
    assert low.dynamic_stability.zmp_margin_m != high.dynamic_stability.zmp_margin_m
    assert (
        low.dynamic_stability.minimum_stability_moment_nm
        != high.dynamic_stability.minimum_stability_moment_nm
    )


def test_none_wrench_vs_empty_list_semantics(mock_config):
    predictor = RolloverPredictor(mock_config)
    none_case = _predict(
        predictor,
        normal=(0.0, 0.0, 1.0),
        accel=(0.0, 0.0, 0.0),
        wrenches=None,
    )
    empty_case = _predict(
        predictor,
        normal=(0.0, 0.0, 1.0),
        accel=(0.0, 0.0, 0.0),
        wrenches=[],
    )
    assert none_case.dynamic_stability is not None
    assert empty_case.dynamic_stability is not None
    assert none_case.dynamic_stability.external_wrench_available is False
    assert empty_case.dynamic_stability.external_wrench_available is True
    assert none_case.dynamic_stability.external_wrench_included is False
    assert empty_case.dynamic_stability.external_wrench_included is True
    assert none_case.dynamic_stability.zmp_margin_m == empty_case.dynamic_stability.zmp_margin_m
    assert none_case.static_stability_margin_m == empty_case.static_stability_margin_m


def test_acceleration_none_keeps_static_invalidates_dynamic(mock_config):
    predictor = RolloverPredictor(mock_config)
    step = _predict(
        predictor,
        normal=(0.0, 0.0, 1.0),
        accel=None,
        wrenches=[],
    )
    assert step.critical_tip is not None
    assert step.static_stability_margin_m > 0.0
    assert step.dynamic_stability is not None
    assert step.dynamic_stability.valid is False
    assert step.dynamic_stability.effective_ssm_m is None
    assert step.dynamic_stability.zmp_margin_m is None


def test_flat_normalized_moment_is_one_and_gravity_constant(mock_config):
    assert GRAVITY_WORLD_M_S2 == (0.0, 0.0, -9.80665)
    predictor = RolloverPredictor(mock_config)
    step = _predict(
        predictor,
        normal=(0.0, 0.0, 1.0),
        accel=(0.0, 0.0, 0.0),
        wrenches=[],
    )
    dyn = step.dynamic_stability
    assert dyn is not None
    assert abs(dyn.normalized_minimum_stability_moment - 1.0) < 1e-12
    assert abs(step.normalized_static_stability_margin - 1.0) < 1e-12


def test_pure_torque_moves_zmp_not_effective_ssm(mock_config):
    predictor = RolloverPredictor(mock_config)
    baseline = _predict(
        predictor,
        normal=(0.0, 0.0, 1.0),
        accel=(0.0, 0.0, 0.0),
        wrenches=[],
    )
    torque = _predict(
        predictor,
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
    assert baseline.dynamic_stability is not None and torque.dynamic_stability is not None
    assert torque.static_stability_margin_m == baseline.static_stability_margin_m
    assert torque.dynamic_stability.effective_ssm_m == baseline.dynamic_stability.effective_ssm_m
    assert torque.dynamic_stability.zmp_xy != baseline.dynamic_stability.zmp_xy
    assert (
        torque.dynamic_stability.minimum_stability_moment_nm
        != baseline.dynamic_stability.minimum_stability_moment_nm
    )
