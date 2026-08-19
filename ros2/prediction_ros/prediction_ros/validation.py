"""Input compatibility checks for the ROS runtime wrapper."""

from __future__ import annotations

from dataclasses import dataclass

from .cache import PredictionSnapshot


@dataclass(frozen=True)
class ValidationConfig:
    expected_frame_id: str = "map"
    require_full_geometry_coverage: bool = False
    max_object_age_sec: float | None = None
    max_geometry_age_sec: float | None = None
    max_state_age_sec: float | None = None
    timestamp_tolerance_sec: float = 1e-3


@dataclass(frozen=True)
class ValidationResult:
    ok: bool
    reason: str = ""


class InputValidator:
    """Validate that cached inputs belong to the active trajectory cycle."""

    def __init__(self, config: ValidationConfig) -> None:
        self.config = config

    def inputs_compatible(self, snapshot: PredictionSnapshot) -> ValidationResult:
        if snapshot.trajectory is None or snapshot.trajectory_id is None:
            return ValidationResult(False, "missing trajectory")
        if snapshot.objects is None:
            return ValidationResult(False, "missing tracked objects batch")
        if snapshot.geometry is None:
            return ValidationResult(False, "missing geometry batch")

        trajectory = snapshot.trajectory
        if trajectory.frame_id != self.config.expected_frame_id:
            return ValidationResult(False, "trajectory frame mismatch")

        for label, frame_id in (
            ("objects", snapshot.objects_frame_id),
            ("geometry", snapshot.geometry_frame_id),
            ("state", snapshot.state_frame_id),
            ("external wrenches", snapshot.external_wrenches_frame_id),
        ):
            if frame_id is not None and frame_id != self.config.expected_frame_id:
                return ValidationResult(False, f"{label} frame mismatch")

        if snapshot.geometry_source_trajectory_id != snapshot.trajectory_id:
            return ValidationResult(
                False,
                f"geometry belongs to trajectory_id={snapshot.geometry_source_trajectory_id}; "
                f"waiting for trajectory_id={snapshot.trajectory_id}",
            )
        if (
            snapshot.geometry_source_trajectory_stamp is not None
            and abs(snapshot.geometry_source_trajectory_stamp - trajectory.timestamp)
            > self.config.timestamp_tolerance_sec
        ):
            return ValidationResult(False, "geometry source timestamp does not match active trajectory")

        if self.config.max_object_age_sec is not None and snapshot.objects:
            age = trajectory.timestamp - min(obj.timestamp for obj in snapshot.objects)
            if age > self.config.max_object_age_sec:
                return ValidationResult(False, "stale tracked objects")

        if self.config.max_geometry_age_sec is not None and snapshot.geometry:
            age = trajectory.timestamp - min(step.timestamp for step in snapshot.geometry)
            if age > self.config.max_geometry_age_sec:
                return ValidationResult(False, "stale geometry")

        if self.config.max_state_age_sec is not None and snapshot.state is not None:
            age = trajectory.timestamp - snapshot.state.timestamp
            if age > self.config.max_state_age_sec:
                return ValidationResult(False, "stale state")

        trajectory_step_ids = {step.step_id for step in trajectory.steps}
        geometry_step_ids = {step.step_id for step in snapshot.geometry}
        if not geometry_step_ids & trajectory_step_ids:
            return ValidationResult(False, "geometry step_ids unrelated to active trajectory")

        if self.config.require_full_geometry_coverage and not trajectory_step_ids.issubset(
            geometry_step_ids
        ):
            return ValidationResult(False, "geometry coverage incomplete")

        return ValidationResult(True, "compatible")
