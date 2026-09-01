"""Pure Decision Prototype V1 STOP/GO policy (demo / integration only)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from decision_ros.evidence import (
    NO_PREDICTION as EVIDENCE_NO_PREDICTION,
    PREDICTION_CURRENT as EVIDENCE_CURRENT,
    PREDICTION_STALE as EVIDENCE_STALE,
)

GO = 0
STOP = 1

CURRENT_CLEAR = 0
NO_CURRENT_PREDICTION = 1
PREDICTION_STALE = 2
COLLISION_CANDIDATE = 3
ROLLOVER_EVIDENCE_INVALID = 4
ROLLOVER_POLICY_TRIGGERED = 5

REASON_NAMES = {
    CURRENT_CLEAR: "CURRENT_CLEAR",
    NO_CURRENT_PREDICTION: "NO_CURRENT_PREDICTION",
    PREDICTION_STALE: "PREDICTION_STALE",
    COLLISION_CANDIDATE: "COLLISION_CANDIDATE",
    ROLLOVER_EVIDENCE_INVALID: "ROLLOVER_EVIDENCE_INVALID",
    ROLLOVER_POLICY_TRIGGERED: "ROLLOVER_POLICY_TRIGGERED",
}

DECISION_NAMES = {GO: "GO", STOP: "STOP"}

METRIC_STABILITY_MOMENT = "stability_moment"
METRIC_NORMALIZED_SSM = "normalized_ssm"
METRIC_ZMP_MARGIN = "zmp_margin"


@dataclass(frozen=True)
class RolloverPolicyConfig:
    enabled: bool = False
    metric: str = METRIC_STABILITY_MOMENT
    threshold: float | None = None


@dataclass(frozen=True)
class PolicyConfig:
    prototype_only: bool = True
    stop_on_collision_candidate: bool = True
    stop_on_missing_current_prediction: bool = True
    rollover_policy: RolloverPolicyConfig = RolloverPolicyConfig()


@dataclass(frozen=True)
class PolicyResult:
    decision: int
    reason: int
    source_trajectory_id: str
    prototype_policy: bool


def _stop(
    reason: int, source_trajectory_id: str, prototype_policy: bool
) -> PolicyResult:
    return PolicyResult(
        decision=STOP,
        reason=reason,
        source_trajectory_id=source_trajectory_id,
        prototype_policy=prototype_policy,
    )


def _go(source_trajectory_id: str, prototype_policy: bool) -> PolicyResult:
    return PolicyResult(
        decision=GO,
        reason=CURRENT_CLEAR,
        source_trajectory_id=source_trajectory_id,
        prototype_policy=prototype_policy,
    )


def _rollover_stop_reason(evidence: Any, rollover: RolloverPolicyConfig) -> int | None:
    """Return a STOP reason when rollover policy fails or triggers; None if pass."""
    threshold = rollover.threshold
    if threshold is None or not math.isfinite(float(threshold)):
        return ROLLOVER_EVIDENCE_INVALID

    metric = str(rollover.metric)
    if metric == METRIC_STABILITY_MOMENT:
        if not bool(evidence.minimum_stability_moment_valid):
            return ROLLOVER_EVIDENCE_INVALID
        value = float(evidence.minimum_stability_moment_nm)
    elif metric == METRIC_NORMALIZED_SSM:
        if not bool(evidence.minimum_normalized_ssm_valid):
            return ROLLOVER_EVIDENCE_INVALID
        value = float(evidence.minimum_normalized_static_stability_margin)
    elif metric == METRIC_ZMP_MARGIN:
        if not bool(evidence.minimum_zmp_margin_valid):
            return ROLLOVER_EVIDENCE_INVALID
        value = float(evidence.minimum_zmp_margin_m)
    else:
        return ROLLOVER_EVIDENCE_INVALID

    if not math.isfinite(value):
        return ROLLOVER_EVIDENCE_INVALID

    if value < float(threshold):
        return ROLLOVER_POLICY_TRIGGERED

    return None


def evaluate_policy(evidence: Any, config: PolicyConfig) -> PolicyResult:
    """Evaluate prototype STOP/GO policy from DecisionEvidence fields."""
    source_id = str(evidence.source_trajectory_id)
    prototype = bool(config.prototype_only)
    state = int(evidence.evidence_state)

    if state != EVIDENCE_CURRENT:
        if not config.stop_on_missing_current_prediction:
            return _go(source_id, prototype)
        if state == EVIDENCE_STALE:
            return _stop(PREDICTION_STALE, source_id, prototype)
        return _stop(NO_CURRENT_PREDICTION, source_id, prototype)

    if config.stop_on_collision_candidate and bool(evidence.collision_candidates_present):
        return _stop(COLLISION_CANDIDATE, source_id, prototype)

    rollover = config.rollover_policy
    if rollover.enabled:
        rollover_reason = _rollover_stop_reason(evidence, rollover)
        if rollover_reason is not None:
            return _stop(rollover_reason, source_id, prototype)

    return _go(source_id, prototype)
