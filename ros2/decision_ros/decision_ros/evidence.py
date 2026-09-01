"""Pure Decision V0 evidence conversion (no policy / no SAFE-STOP semantics)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

NO_PREDICTION = 0
PREDICTION_STALE = 1
PREDICTION_CURRENT = 2

_NAN = float("nan")


@dataclass(frozen=True)
class EvidenceFields:
    evidence_state: int
    source_trajectory_id: str
    collision_candidates_present: bool
    rollover_baseline_present: bool
    dynamic_stability_moment_valid: bool
    zmp_valid: bool
    nearest_collision_distance_valid: bool
    nearest_collision_distance_m: float
    minimum_normalized_ssm_valid: bool
    minimum_normalized_static_stability_margin: float
    minimum_stability_moment_valid: bool
    minimum_stability_moment_nm: float
    minimum_zmp_margin_valid: bool
    minimum_zmp_margin_m: float


def _empty_fields(
    evidence_state: int, source_trajectory_id: str
) -> EvidenceFields:
    return EvidenceFields(
        evidence_state=evidence_state,
        source_trajectory_id=source_trajectory_id,
        collision_candidates_present=False,
        rollover_baseline_present=False,
        dynamic_stability_moment_valid=False,
        zmp_valid=False,
        nearest_collision_distance_valid=False,
        nearest_collision_distance_m=_NAN,
        minimum_normalized_ssm_valid=False,
        minimum_normalized_static_stability_margin=_NAN,
        minimum_stability_moment_valid=False,
        minimum_stability_moment_nm=_NAN,
        minimum_zmp_margin_valid=False,
        minimum_zmp_margin_m=_NAN,
    )


def _collision_candidates_present(prediction: Any) -> bool:
    for step in prediction.collision_steps:
        if step.collision_objects:
            return True
    return False


def _nearest_collision_distance_m(prediction: Any) -> tuple[bool, float]:
    nearest = _NAN
    found = False
    for step in prediction.collision_steps:
        for obj in step.collision_objects:
            dist = float(obj.min_distance_m)
            if not math.isfinite(dist):
                continue
            if not found or dist < nearest:
                nearest = dist
                found = True
    return found, nearest


def _rollover_baseline_present(prediction: Any) -> bool:
    return bool(prediction.rollover_steps)


def _dynamic_stability_moment_valid(prediction: Any) -> bool:
    for step in prediction.rollover_steps:
        if bool(step.stability_moment.valid):
            return True
    return False


def _zmp_valid(prediction: Any) -> bool:
    for step in prediction.rollover_steps:
        if bool(step.zmp.valid):
            return True
    return False


def _minimum_normalized_ssm(prediction: Any) -> tuple[bool, float]:
    minimum = _NAN
    found = False
    for step in prediction.rollover_steps:
        value = float(step.normalized_static_stability_margin)
        if not math.isfinite(value):
            continue
        if not found or value < minimum:
            minimum = value
            found = True
    return found, minimum


def _minimum_stability_moment_nm(prediction: Any) -> tuple[bool, float]:
    minimum = _NAN
    found = False
    for step in prediction.rollover_steps:
        sm = step.stability_moment
        if not bool(sm.valid):
            continue
        value = float(sm.minimum_stability_moment_nm)
        if not math.isfinite(value):
            continue
        if not found or value < minimum:
            minimum = value
            found = True
    return found, minimum


def _minimum_zmp_margin_m(prediction: Any) -> tuple[bool, float]:
    minimum = _NAN
    found = False
    for step in prediction.rollover_steps:
        zmp = step.zmp
        if not bool(zmp.valid):
            continue
        value = float(zmp.margin_m)
        if not math.isfinite(value):
            continue
        if not found or value < minimum:
            minimum = value
            found = True
    return found, minimum


def _evidence_from_prediction(
    active_trajectory_id: int, prediction: Any
) -> EvidenceFields:
    nearest_valid, nearest_dist = _nearest_collision_distance_m(prediction)
    ssm_valid, min_ssm = _minimum_normalized_ssm(prediction)
    sm_valid, min_sm = _minimum_stability_moment_nm(prediction)
    zmp_margin_valid, min_zmp = _minimum_zmp_margin_m(prediction)
    return EvidenceFields(
        evidence_state=PREDICTION_CURRENT,
        source_trajectory_id=str(int(prediction.source_trajectory_id)),
        collision_candidates_present=_collision_candidates_present(prediction),
        rollover_baseline_present=_rollover_baseline_present(prediction),
        dynamic_stability_moment_valid=_dynamic_stability_moment_valid(prediction),
        zmp_valid=_zmp_valid(prediction),
        nearest_collision_distance_valid=nearest_valid,
        nearest_collision_distance_m=nearest_dist,
        minimum_normalized_ssm_valid=ssm_valid,
        minimum_normalized_static_stability_margin=min_ssm,
        minimum_stability_moment_valid=sm_valid,
        minimum_stability_moment_nm=min_sm,
        minimum_zmp_margin_valid=zmp_margin_valid,
        minimum_zmp_margin_m=min_zmp,
    )


def build_decision_evidence(
    active_trajectory_id: int | None,
    prediction: Any | None,
) -> EvidenceFields:
    """Convert active trajectory + optional PredictionOutput to evidence fields."""
    if active_trajectory_id is None:
        return _empty_fields(NO_PREDICTION, "")

    active_id = int(active_trajectory_id)
    active_str = str(active_id)

    if prediction is None:
        return _empty_fields(NO_PREDICTION, active_str)

    pred_id = int(prediction.source_trajectory_id)
    if pred_id != active_id:
        return _empty_fields(PREDICTION_STALE, active_str)

    return _evidence_from_prediction(active_id, prediction)
