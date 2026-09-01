"""Unit tests for Decision Prototype V1 STOP/GO policy."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from decision_ros.evidence import NO_PREDICTION, PREDICTION_CURRENT, PREDICTION_STALE
from decision_ros.policy import (
    COLLISION_CANDIDATE,
    CURRENT_CLEAR,
    GO,
    METRIC_STABILITY_MOMENT,
    NO_CURRENT_PREDICTION,
    PREDICTION_STALE as REASON_PREDICTION_STALE,
    ROLLOVER_EVIDENCE_INVALID,
    ROLLOVER_POLICY_TRIGGERED,
    STOP,
    PolicyConfig,
    RolloverPolicyConfig,
    evaluate_policy,
)


def _evidence(
    *,
    evidence_state: int = PREDICTION_CURRENT,
    source_trajectory_id: str = "1",
    collision_candidates_present: bool = False,
    rollover_baseline_present: bool = True,
    dynamic_stability_moment_valid: bool = True,
    zmp_valid: bool = True,
    minimum_stability_moment_valid: bool = True,
    minimum_stability_moment_nm: float = 5.0,
    minimum_normalized_ssm_valid: bool = True,
    minimum_normalized_static_stability_margin: float = 0.5,
    minimum_zmp_margin_valid: bool = True,
    minimum_zmp_margin_m: float = 0.1,
):
    return SimpleNamespace(
        evidence_state=evidence_state,
        source_trajectory_id=source_trajectory_id,
        collision_candidates_present=collision_candidates_present,
        rollover_baseline_present=rollover_baseline_present,
        dynamic_stability_moment_valid=dynamic_stability_moment_valid,
        zmp_valid=zmp_valid,
        minimum_stability_moment_valid=minimum_stability_moment_valid,
        minimum_stability_moment_nm=minimum_stability_moment_nm,
        minimum_normalized_ssm_valid=minimum_normalized_ssm_valid,
        minimum_normalized_static_stability_margin=minimum_normalized_static_stability_margin,
        minimum_zmp_margin_valid=minimum_zmp_margin_valid,
        minimum_zmp_margin_m=minimum_zmp_margin_m,
    )


def _default_config(**overrides):
    base = PolicyConfig()
    if not overrides:
        return base
    rollover_overrides = {
        k.split(".", 1)[1]: v
        for k, v in overrides.items()
        if k.startswith("rollover_policy.")
    }
    top = {
        k: v for k, v in overrides.items() if not k.startswith("rollover_policy.")
    }
    rollover = RolloverPolicyConfig(
        enabled=rollover_overrides.get("enabled", base.rollover_policy.enabled),
        metric=rollover_overrides.get("metric", base.rollover_policy.metric),
        threshold=rollover_overrides.get(
            "threshold", base.rollover_policy.threshold
        ),
    )
    return PolicyConfig(
        prototype_only=top.get("prototype_only", base.prototype_only),
        stop_on_collision_candidate=top.get(
            "stop_on_collision_candidate", base.stop_on_collision_candidate
        ),
        stop_on_missing_current_prediction=top.get(
            "stop_on_missing_current_prediction", base.stop_on_missing_current_prediction
        ),
        rollover_policy=rollover,
    )


def test_current_no_collision_rollover_disabled_go():
    result = evaluate_policy(_evidence(), _default_config())
    assert result.decision == GO
    assert result.reason == CURRENT_CLEAR
    assert result.prototype_policy is True


def test_no_prediction_stop():
    result = evaluate_policy(
        _evidence(evidence_state=NO_PREDICTION, source_trajectory_id="3"),
        _default_config(),
    )
    assert result.decision == STOP
    assert result.reason == NO_CURRENT_PREDICTION


def test_stale_stop():
    result = evaluate_policy(
        _evidence(evidence_state=PREDICTION_STALE, source_trajectory_id="5"),
        _default_config(),
    )
    assert result.decision == STOP
    assert result.reason == REASON_PREDICTION_STALE


def test_collision_candidate_stop():
    result = evaluate_policy(
        _evidence(collision_candidates_present=True),
        _default_config(),
    )
    assert result.decision == STOP
    assert result.reason == COLLISION_CANDIDATE


def test_invalid_rollover_evidence_does_not_become_go_when_required():
    config = _default_config(
        **{
            "rollover_policy.enabled": True,
            "rollover_policy.threshold": 1.0,
            "rollover_policy.metric": METRIC_STABILITY_MOMENT,
        }
    )
    result = evaluate_policy(
        _evidence(minimum_stability_moment_valid=False),
        config,
    )
    assert result.decision == STOP
    assert result.reason == ROLLOVER_EVIDENCE_INVALID


def test_rollover_disabled_does_not_invent_threshold():
    config = _default_config()
    assert config.rollover_policy.enabled is False
    assert config.rollover_policy.threshold is None
    result = evaluate_policy(_evidence(minimum_stability_moment_nm=-99.0), config)
    assert result.decision == GO
    assert result.reason == CURRENT_CLEAR


def test_rollover_enabled_without_valid_configuration_fails_safe():
    config = _default_config(**{"rollover_policy.enabled": True})
    assert config.rollover_policy.threshold is None
    result = evaluate_policy(_evidence(), config)
    assert result.decision == STOP
    assert result.reason == ROLLOVER_EVIDENCE_INVALID


def test_rollover_enabled_threshold_triggered():
    config = _default_config(
        **{
            "rollover_policy.enabled": True,
            "rollover_policy.threshold": 2.0,
            "rollover_policy.metric": METRIC_STABILITY_MOMENT,
        }
    )
    result = evaluate_policy(
        _evidence(minimum_stability_moment_nm=1.5),
        config,
    )
    assert result.decision == STOP
    assert result.reason == ROLLOVER_POLICY_TRIGGERED


def test_duplicate_evidence_deterministic():
    evidence = _evidence(source_trajectory_id="11")
    config = _default_config()
    first = evaluate_policy(evidence, config)
    second = evaluate_policy(evidence, config)
    assert first == second


def test_new_trajectory_removes_old_go():
    current = evaluate_policy(
        _evidence(evidence_state=PREDICTION_CURRENT, source_trajectory_id="1"),
        _default_config(),
    )
    assert current.decision == GO
    stale = evaluate_policy(
        _evidence(evidence_state=PREDICTION_STALE, source_trajectory_id="2"),
        _default_config(),
    )
    assert stale.decision == STOP
    assert stale.reason == REASON_PREDICTION_STALE


def test_prototype_policy_flag_true():
    result = evaluate_policy(_evidence(), _default_config(prototype_only=True))
    assert result.prototype_policy is True
