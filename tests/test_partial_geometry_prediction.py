"""Synthetic Prediction fixture: 20 traj steps, geometry only on 0..11."""

from __future__ import annotations

from prediction_core.config import RoverConfig
from prediction_core.models import GeometryStep, RoverState, Trajectory, TrajectoryStep
from prediction_core.runtime import PredictionRuntime
from prediction_core.validation import PredictionProfile


TRAJECTORY_ID = 92024
N_TRAJ = 20
VALID_GEOM = list(range(12))  # steps 0..11 ≈ 2.75 s at dt=0.25


def _trajectory(stamp: float = 100.0) -> Trajectory:
    return Trajectory(
        timestamp=stamp,
        frame_id="map",
        steps=[
            TrajectoryStep(step_id=i, x=float(i) * 0.5, y=0.0, yaw=0.0)
            for i in range(N_TRAJ)
        ],
    )


def _partial_geometry(stamp: float = 100.0) -> list[GeometryStep]:
    return [
        GeometryStep(
            timestamp=stamp,
            step_id=i,
            plane_id=f"plane-{i}",
            normal_xyz=(0.0, 0.0, 1.0),
            confidence=0.9,
        )
        for i in VALID_GEOM
    ]


def _state(stamp: float = 100.0) -> RoverState:
    return RoverState(
        timestamp=stamp,
        x=0.0,
        y=0.0,
        yaw=0.0,
        velocity_xy=(0.5, 0.0),
        acceleration_xyz=(0.0, 0.0, 0.0),
    )


def test_dynamic_prediction_accepts_sparse_geometry_20_of_12(mock_config: RoverConfig) -> None:
    runtime = PredictionRuntime(
        mock_config,
        profile=PredictionProfile.DYNAMIC,
        expected_frame_id="map",
        require_full_geometry_coverage=False,
    )
    runtime.on_trajectory(_trajectory(), trajectory_id=TRAJECTORY_ID)
    runtime.on_objects([], frame_id="map")
    runtime.on_state(_state(), frame_id="map")
    result = runtime.on_geometry(
        _partial_geometry(),
        frame_id="map",
        source_trajectory_id=TRAJECTORY_ID,
        source_trajectory_stamp=100.0,
    )

    assert result.output is not None
    assert result.cycle_key is not None
    assert result.cycle_key.trajectory_id == TRAJECTORY_ID

    rollover_ids = [step.step_id for step in result.output.rollover_steps]
    assert rollover_ids == VALID_GEOM
    assert len(rollover_ids) == 12
    assert len(rollover_ids) / N_TRAJ == 0.6

    # Contiguous supported horizon: steps 0..11 → 11 * 0.25 s = 2.75 s
    assert (len(VALID_GEOM) - 1) * 0.25 == 2.75

    # No array-index misjoin: missing traj steps 12..19 must not appear
    assert all(sid <= 11 for sid in rollover_ids)
    assert 12 not in rollover_ids
    assert 19 not in rollover_ids

    # Collision still evaluates trajectory independently (empty objects → no candidates)
    assert result.output.collision_steps == []

    missing = runtime.core.rollover_predictor.last_missing_step_ids
    assert missing == list(range(12, 20))


def test_collision_still_runs_on_all_trajectory_steps_with_partial_geometry(
    mock_config: RoverConfig,
) -> None:
    """Collision does not depend on GeometryArray coverage."""
    from prediction_core.models import TrackedObject

    runtime = PredictionRuntime(
        mock_config,
        profile=PredictionProfile.DYNAMIC,
        expected_frame_id="map",
        require_full_geometry_coverage=False,
    )
    runtime.on_trajectory(_trajectory(), trajectory_id=TRAJECTORY_ID)
    # Object overlapping a far future step (15 → x=7.5) that has NO geometry.
    obj = TrackedObject(
        timestamp=100.0,
        track_id="obj-far",
        class_name="debris",
        footprint_polygon_xy=[
            (7.0, -0.5),
            (8.0, -0.5),
            (8.0, 0.5),
            (7.0, 0.5),
        ],
        confidence=0.8,
    )
    runtime.on_objects([obj], frame_id="map")
    runtime.on_state(_state(), frame_id="map")
    result = runtime.on_geometry(
        _partial_geometry(),
        frame_id="map",
        source_trajectory_id=TRAJECTORY_ID,
        source_trajectory_stamp=100.0,
    )
    assert result.output is not None
    assert [s.step_id for s in result.output.rollover_steps] == VALID_GEOM
    collision_ids = {s.step_id for s in result.output.collision_steps}
    assert collision_ids
    # At least one collision candidate is outside geometry-supported steps.
    assert any(sid >= 12 for sid in collision_ids)
