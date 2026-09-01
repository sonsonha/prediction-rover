"""Unit tests for pure Decision V0 evidence conversion."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from decision_ros.evidence import (
    NO_PREDICTION,
    PREDICTION_CURRENT,
    PREDICTION_STALE,
    build_decision_evidence,
)


def _collision_obj(min_distance_m: float):
    return SimpleNamespace(min_distance_m=min_distance_m)


def _collision_step(objects):
    return SimpleNamespace(collision_objects=objects)


def _stability_moment(*, valid: bool, minimum_stability_moment_nm: float = 0.0):
    return SimpleNamespace(valid=valid, minimum_stability_moment_nm=minimum_stability_moment_nm)


def _zmp(*, valid: bool, margin_m: float = 0.0):
    return SimpleNamespace(valid=valid, margin_m=margin_m)


def _rollover_step(
    *,
    normalized_static_stability_margin: float = 0.0,
    stability_moment=None,
    zmp=None,
):
    return SimpleNamespace(
        normalized_static_stability_margin=normalized_static_stability_margin,
        stability_moment=stability_moment or _stability_moment(valid=False),
        zmp=zmp or _zmp(valid=False),
    )


def _prediction(
    source_trajectory_id: int,
    *,
    collision_steps=None,
    rollover_steps=None,
):
    return SimpleNamespace(
        source_trajectory_id=source_trajectory_id,
        collision_steps=collision_steps or [],
        rollover_steps=rollover_steps or [],
    )


def test_no_active_trajectory():
    out = build_decision_evidence(None, None)
    assert out.evidence_state == NO_PREDICTION
    assert out.source_trajectory_id == ""
    assert out.collision_candidates_present is False


def test_active_trajectory_no_prediction():
    out = build_decision_evidence(11, None)
    assert out.evidence_state == NO_PREDICTION
    assert out.source_trajectory_id == "11"
    assert out.collision_candidates_present is False


def test_stale_prediction_output():
    pred = _prediction(1, collision_steps=[_collision_step([_collision_obj(0.1)])])
    out = build_decision_evidence(11, pred)
    assert out.evidence_state == PREDICTION_STALE
    assert out.source_trajectory_id == "11"
    assert out.collision_candidates_present is False
    assert out.nearest_collision_distance_valid is False


def test_matching_prediction_output():
    pred = _prediction(11, rollover_steps=[_rollover_step(normalized_static_stability_margin=0.5)])
    out = build_decision_evidence(11, pred)
    assert out.evidence_state == PREDICTION_CURRENT
    assert out.source_trajectory_id == "11"
    assert out.rollover_baseline_present is True


def test_collision_candidate_present():
    pred = _prediction(
        1,
        collision_steps=[_collision_step([_collision_obj(0.15), _collision_obj(0.05)])],
    )
    out = build_decision_evidence(1, pred)
    assert out.collision_candidates_present is True
    assert out.nearest_collision_distance_valid is True
    assert out.nearest_collision_distance_m == pytest.approx(0.05)


def test_no_collision_candidate():
    pred = _prediction(1, collision_steps=[])
    out = build_decision_evidence(1, pred)
    assert out.collision_candidates_present is False
    assert out.nearest_collision_distance_valid is False
    assert math.isnan(out.nearest_collision_distance_m)


def test_rollover_baseline_present():
    pred = _prediction(1, rollover_steps=[_rollover_step()])
    out = build_decision_evidence(1, pred)
    assert out.rollover_baseline_present is True


def test_stability_moment_valid():
    pred = _prediction(
        1,
        rollover_steps=[
            _rollover_step(stability_moment=_stability_moment(valid=True, minimum_stability_moment_nm=12.0))
        ],
    )
    out = build_decision_evidence(1, pred)
    assert out.dynamic_stability_moment_valid is True
    assert out.minimum_stability_moment_valid is True
    assert out.minimum_stability_moment_nm == pytest.approx(12.0)


def test_stability_moment_invalid():
    pred = _prediction(
        1,
        rollover_steps=[_rollover_step(stability_moment=_stability_moment(valid=False))],
    )
    out = build_decision_evidence(1, pred)
    assert out.dynamic_stability_moment_valid is False
    assert out.minimum_stability_moment_valid is False
    assert math.isnan(out.minimum_stability_moment_nm)


def test_zmp_valid():
    pred = _prediction(
        1,
        rollover_steps=[_rollover_step(zmp=_zmp(valid=True, margin_m=0.03))],
    )
    out = build_decision_evidence(1, pred)
    assert out.zmp_valid is True
    assert out.minimum_zmp_margin_valid is True
    assert out.minimum_zmp_margin_m == pytest.approx(0.03)


def test_zmp_invalid():
    pred = _prediction(1, rollover_steps=[_rollover_step(zmp=_zmp(valid=False))])
    out = build_decision_evidence(1, pred)
    assert out.zmp_valid is False
    assert out.minimum_zmp_margin_valid is False
    assert math.isnan(out.minimum_zmp_margin_m)


def test_negative_ssm_preserved():
    pred = _prediction(
        1,
        rollover_steps=[
            _rollover_step(normalized_static_stability_margin=0.2),
            _rollover_step(normalized_static_stability_margin=-0.7),
        ],
    )
    out = build_decision_evidence(1, pred)
    assert out.minimum_normalized_ssm_valid is True
    assert out.minimum_normalized_static_stability_margin == pytest.approx(-0.7)


def test_negative_stability_moment_preserved():
    pred = _prediction(
        1,
        rollover_steps=[
            _rollover_step(
                stability_moment=_stability_moment(valid=True, minimum_stability_moment_nm=-3.5)
            )
        ],
    )
    out = build_decision_evidence(1, pred)
    assert out.minimum_stability_moment_valid is True
    assert out.minimum_stability_moment_nm == pytest.approx(-3.5)


def test_new_trajectory_invalidates_previous_current_evidence():
    pred = _prediction(
        1,
        collision_steps=[_collision_step([_collision_obj(0.1)])],
        rollover_steps=[_rollover_step(normalized_static_stability_margin=0.4)],
    )
    current = build_decision_evidence(1, pred)
    assert current.evidence_state == PREDICTION_CURRENT
    assert current.collision_candidates_present is True

    stale = build_decision_evidence(11, pred)
    assert stale.evidence_state == PREDICTION_STALE
    assert stale.collision_candidates_present is False
    assert stale.rollover_baseline_present is False


def test_duplicate_prediction_output_no_contradictory_state():
    pred = _prediction(
        11,
        collision_steps=[_collision_step([_collision_obj(0.2)])],
        rollover_steps=[_rollover_step(normalized_static_stability_margin=0.1)],
    )
    first = build_decision_evidence(11, pred)
    second = build_decision_evidence(11, pred)
    assert first == second
    assert first.evidence_state == PREDICTION_CURRENT
