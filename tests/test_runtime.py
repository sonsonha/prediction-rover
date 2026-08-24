"""Pure-Python PredictionRuntime event / cycle / readiness tests."""

from __future__ import annotations

from prediction_core.cache import ExternalWrenchData
from prediction_core.config import RoverConfig
from prediction_core.models import GeometryStep, RoverState, TrackedObject, Trajectory, TrajectoryStep
from prediction_core.runtime import PredictionRuntime
from prediction_core.validation import PredictionReadiness


def _trajectory(
    stamp: float = 100.0,
    *,
    frame_id: str = "map",
    step_ids: list[int] | None = None,
) -> Trajectory:
    step_ids = step_ids or [0, 1, 2]
    return Trajectory(
        timestamp=stamp,
        frame_id=frame_id,
        steps=[
            TrajectoryStep(step_id=step_id, x=float(step_id), y=0.0, yaw=0.0)
            for step_id in step_ids
        ],
    )


def _geometry(
    stamp: float = 100.0,
    *,
    step_ids: list[int] | None = None,
    normal: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> list[GeometryStep]:
    step_ids = step_ids or [0, 1, 2]
    return [
        GeometryStep(
            timestamp=stamp,
            step_id=step_id,
            plane_id=f"plane-{step_id}",
            normal_xyz=normal,
        )
        for step_id in step_ids
    ]


def _runtime(mock_config: RoverConfig) -> PredictionRuntime:
    return PredictionRuntime(mock_config, expected_frame_id="map")


def test_trajectory_only_no_prediction(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    result = runtime.on_trajectory(_trajectory(), trajectory_id=42)
    assert result.output is None
    assert result.readiness.ready is False
    assert "missing tracked objects batch" in result.readiness.reasons
    assert "missing geometry batch" in result.readiness.reasons


def test_trajectory_plus_objects_no_prediction(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    runtime.on_trajectory(_trajectory(), trajectory_id=42)
    result = runtime.on_objects([], frame_id="map")
    assert result.output is None
    assert "missing geometry batch" in result.readiness.reasons


def test_trajectory_plus_geometry_waits_for_objects(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    runtime.on_trajectory(_trajectory(), trajectory_id=42)
    result = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=42,
        source_trajectory_stamp=100.0,
    )
    assert result.output is None
    assert result.readiness.ready is False
    assert "missing tracked objects batch" in result.readiness.reasons
    assert runtime.snapshot().objects is None


def test_empty_objects_and_matching_geometry_predict_once(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    runtime.on_trajectory(_trajectory(), trajectory_id=42)
    runtime.on_objects([], frame_id="map")
    result = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=42,
        source_trajectory_stamp=100.0,
    )
    assert result.output is not None
    assert result.cycle_key is not None
    assert result.cycle_key.trajectory_id == 42
    assert len(result.output.rollover_steps) == 3


def test_wrong_geometry_source_trajectory_id_blocks(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    runtime.on_trajectory(_trajectory(), trajectory_id=42)
    runtime.on_objects([], frame_id="map")
    result = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=41,
        source_trajectory_stamp=100.0,
    )
    assert result.output is None
    assert any("trajectory_id=41" in reason for reason in result.readiness.reasons)


def test_new_trajectory_clears_cycle_bound_geometry(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    runtime.on_trajectory(_trajectory(100.0), trajectory_id=42)
    runtime.on_objects([], frame_id="map")
    runtime.on_geometry(
        _geometry(100.0),
        frame_id="map",
        source_trajectory_id=42,
        source_trajectory_stamp=100.0,
    )
    assert runtime.try_predict().output is not None or runtime.last_predicted_cycle is not None

    # Also seed dynamic inputs so we can assert they clear on the next cycle.
    runtime.on_state(
        RoverState(timestamp=100.5, acceleration_xyz=(0.0, 0.0, 0.0)),
        frame_id="map",
    )
    runtime.on_external_wrenches([], frame_id="map")

    result = runtime.on_trajectory(_trajectory(200.0), trajectory_id=43)
    snap = runtime.snapshot()
    assert snap.objects is None
    assert snap.geometry is None
    assert snap.state is None
    assert snap.external_wrenches is None
    assert result.output is None
    assert "missing tracked objects batch" in result.readiness.reasons
    assert "missing geometry batch" in result.readiness.reasons


def test_new_trajectory_does_not_reuse_old_objects(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    runtime.on_trajectory(_trajectory(100.0), trajectory_id=42)
    runtime.on_objects(
        [
            TrackedObject(
                timestamp=100.0,
                track_id="pipe-1",
                class_name="pipe",
                footprint_polygon_xy=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
            )
        ],
        frame_id="map",
    )
    runtime.on_geometry(
        _geometry(100.0),
        frame_id="map",
        source_trajectory_id=42,
        source_trajectory_stamp=100.0,
    )
    assert runtime.try_predict().output is not None or runtime.last_predicted_cycle is not None

    runtime.on_trajectory(_trajectory(200.0), trajectory_id=43)
    runtime.on_geometry(
        _geometry(200.0),
        frame_id="map",
        source_trajectory_id=43,
        source_trajectory_stamp=200.0,
    )
    result = runtime.try_predict()
    assert result.output is None
    assert runtime.snapshot().objects is None
    assert "missing tracked objects batch" in result.readiness.reasons


def test_frame_mismatch_no_prediction(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    runtime.on_trajectory(_trajectory(), trajectory_id=42)
    runtime.on_objects([], frame_id="odom")
    result = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=42,
        source_trajectory_stamp=100.0,
    )
    assert result.output is None
    assert any("frame mismatch" in reason for reason in result.readiness.reasons)


def test_duplicate_events_same_cycle_no_second_prediction(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    runtime.on_trajectory(_trajectory(), trajectory_id=42)
    runtime.on_objects([], frame_id="map")
    first = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=42,
        source_trajectory_stamp=100.0,
    )
    assert first.output is not None

    second = runtime.on_objects([], frame_id="map")
    assert second.output is None
    assert second.duplicate_cycle is True

    third = runtime.on_state(
        RoverState(timestamp=100.5, x=0.0, y=0.0, yaw=0.0),
        frame_id="map",
    )
    assert third.output is None
    assert third.duplicate_cycle is True

    fourth = runtime.on_external_wrenches([], frame_id="map")
    assert fourth.output is None
    assert fourth.duplicate_cycle is True


def test_state_absent_static_v1_still_runs(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    runtime.on_trajectory(_trajectory(), trajectory_id=42)
    runtime.on_objects([], frame_id="map")
    result = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=42,
        source_trajectory_stamp=100.0,
    )
    assert result.output is not None
    assert runtime.snapshot().state is None


def test_external_wrenches_absent_static_v1_still_runs(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    runtime.on_trajectory(_trajectory(), trajectory_id=42)
    runtime.on_objects([], frame_id="map")
    result = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=42,
        source_trajectory_stamp=100.0,
    )
    assert result.output is not None
    assert runtime.snapshot().external_wrenches is None


def test_external_wrenches_empty_list_does_not_block(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    runtime.on_trajectory(_trajectory(), trajectory_id=7)
    runtime.on_objects([], frame_id="map")
    runtime.on_external_wrenches([], frame_id="map")
    assert runtime.snapshot().external_wrenches == []
    result = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=7,
        source_trajectory_stamp=100.0,
    )
    assert result.output is not None


def test_external_wrenches_present_do_not_repredict_same_cycle(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    runtime.on_trajectory(_trajectory(), trajectory_id=7)
    runtime.on_objects([], frame_id="map")
    first = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=7,
        source_trajectory_stamp=100.0,
    )
    assert first.output is not None
    second = runtime.on_external_wrenches(
        [
            ExternalWrenchData(
                source="test",
                frame_id="map",
                force_xyz=(1.0, 0.0, 0.0),
                torque_xyz=(0.0, 0.0, 0.0),
                application_point_xyz=None,
                confidence=None,
            )
        ],
        frame_id="map",
    )
    assert second.output is None
    assert second.duplicate_cycle is True


def test_permutation_trajectory_geometry_objects(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    runtime.on_trajectory(_trajectory(), trajectory_id=10)
    runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=10,
        source_trajectory_stamp=100.0,
    )
    result = runtime.on_objects([], frame_id="map")
    assert result.output is not None


def test_permutation_trajectory_objects_geometry(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    runtime.on_trajectory(_trajectory(), trajectory_id=11)
    runtime.on_objects([], frame_id="map")
    result = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=11,
        source_trajectory_stamp=100.0,
    )
    assert result.output is not None


def test_permutation_objects_geometry_trajectory_clears_then_needs_refresh(
    mock_config: RoverConfig,
) -> None:
    runtime = _runtime(mock_config)
    runtime.on_objects([], frame_id="map")
    runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=12,
        source_trajectory_stamp=100.0,
    )
    cleared = runtime.on_trajectory(_trajectory(), trajectory_id=12)
    assert cleared.output is None
    assert runtime.snapshot().objects is None
    assert runtime.snapshot().geometry is None

    runtime.on_objects([], frame_id="map")
    result = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=12,
        source_trajectory_stamp=100.0,
    )
    assert result.output is not None


def test_permutation_geometry_trajectory_objects_clears_geometry(
    mock_config: RoverConfig,
) -> None:
    runtime = _runtime(mock_config)
    runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=13,
        source_trajectory_stamp=100.0,
    )
    runtime.on_trajectory(_trajectory(), trajectory_id=13)
    assert runtime.snapshot().geometry is None
    objects_only = runtime.on_objects([], frame_id="map")
    assert objects_only.output is None
    result = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=13,
        source_trajectory_stamp=100.0,
    )
    assert result.output is not None


def test_new_cycle_can_predict_again(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    runtime.on_trajectory(_trajectory(100.0), trajectory_id=42)
    runtime.on_objects([], frame_id="map")
    first = runtime.on_geometry(
        _geometry(100.0),
        frame_id="map",
        source_trajectory_id=42,
        source_trajectory_stamp=100.0,
    )
    assert first.output is not None

    runtime.on_trajectory(_trajectory(200.0), trajectory_id=43)
    runtime.on_objects([], frame_id="map")
    second = runtime.on_geometry(
        _geometry(200.0),
        frame_id="map",
        source_trajectory_id=43,
        source_trajectory_stamp=200.0,
    )
    assert second.output is not None
    assert second.cycle_key is not None
    assert second.cycle_key.trajectory_id == 43


def test_readiness_exposes_reasons(mock_config: RoverConfig) -> None:
    runtime = _runtime(mock_config)
    readiness = runtime.readiness()
    assert isinstance(readiness, PredictionReadiness)
    assert readiness.ready is False
    assert "missing trajectory" in readiness.reasons
