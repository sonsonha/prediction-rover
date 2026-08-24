"""PredictionRuntime static/dynamic readiness profile tests."""

from __future__ import annotations

from prediction_core.cache import ExternalWrenchData
from prediction_core.config import RoverConfig
from prediction_core.models import GeometryStep, RoverState, Trajectory, TrajectoryStep
from prediction_core.runtime import PredictionRuntime
from prediction_core.validation import PredictionProfile


def _trajectory(stamp: float = 100.0) -> Trajectory:
    return Trajectory(
        timestamp=stamp,
        frame_id="map",
        steps=[
            TrajectoryStep(step_id=0, x=0.0, y=0.0, yaw=0.0),
            TrajectoryStep(step_id=1, x=1.0, y=0.0, yaw=0.0),
        ],
    )


def _geometry(stamp: float = 100.0) -> list[GeometryStep]:
    return [
        GeometryStep(
            timestamp=stamp,
            step_id=step_id,
            plane_id=f"plane-{step_id}",
            normal_xyz=(0.0, 0.0, 1.0),
        )
        for step_id in (0, 1)
    ]


def _state(
    stamp: float = 100.5,
    *,
    acceleration_xyz: tuple[float, float, float] | None = (0.0, 0.0, 0.0),
) -> RoverState:
    return RoverState(timestamp=stamp, acceleration_xyz=acceleration_xyz)


def _feed_static_ready(runtime: PredictionRuntime, *, trajectory_id: int = 1) -> None:
    runtime.on_trajectory(_trajectory(), trajectory_id=trajectory_id)
    runtime.on_objects([], frame_id="map")


def test_a_static_default_predicts_without_state(mock_config: RoverConfig) -> None:
    runtime = PredictionRuntime(mock_config)
    assert runtime.profile == PredictionProfile.STATIC
    _feed_static_ready(runtime)
    result = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=1,
        source_trajectory_stamp=100.0,
    )
    assert result.output is not None
    assert result.readiness.ready is True


def test_b_explicit_static_same_behavior(mock_config: RoverConfig) -> None:
    runtime = PredictionRuntime(mock_config, profile="static")
    assert runtime.profile == PredictionProfile.STATIC
    _feed_static_ready(runtime)
    result = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=1,
        source_trajectory_stamp=100.0,
    )
    assert result.output is not None


def test_c_dynamic_waits_for_state_then_predicts(mock_config: RoverConfig) -> None:
    runtime = PredictionRuntime(mock_config, profile=PredictionProfile.DYNAMIC)
    _feed_static_ready(runtime)
    geo = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=1,
        source_trajectory_stamp=100.0,
    )
    assert geo.output is None
    assert "missing rover state" in geo.readiness.reasons
    assert any("rover state" in message for message in geo.messages)

    result = runtime.on_state(_state(acceleration_xyz=(0.0, 1.0, 0.0)), frame_id="map")
    assert result.output is not None
    dyn = result.output.rollover_steps[0].dynamic_stability
    assert dyn is not None
    assert dyn.acceleration_available is True


def test_d_dynamic_invalid_acceleration_blocks(mock_config: RoverConfig) -> None:
    runtime = PredictionRuntime(mock_config, profile="dynamic")
    _feed_static_ready(runtime)
    runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=1,
        source_trajectory_stamp=100.0,
    )
    result = runtime.on_state(_state(acceleration_xyz=None), frame_id="map")
    assert result.output is None
    assert "rover acceleration unavailable" in result.readiness.reasons


def test_e_dynamic_zero_acceleration_is_valid(mock_config: RoverConfig) -> None:
    runtime = PredictionRuntime(mock_config, profile=PredictionProfile.DYNAMIC)
    _feed_static_ready(runtime)
    runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=1,
        source_trajectory_stamp=100.0,
    )
    result = runtime.on_state(_state(acceleration_xyz=(0.0, 0.0, 0.0)), frame_id="map")
    assert result.output is not None
    dyn = result.output.rollover_steps[0].dynamic_stability
    assert dyn is not None
    assert dyn.valid is True
    assert dyn.acceleration_available is True


def test_f_dynamic_wrench_none_still_predicts(mock_config: RoverConfig) -> None:
    runtime = PredictionRuntime(mock_config, profile="dynamic")
    _feed_static_ready(runtime)
    runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=1,
        source_trajectory_stamp=100.0,
    )
    result = runtime.on_state(_state(acceleration_xyz=(0.0, 0.5, 0.0)), frame_id="map")
    assert result.output is not None
    assert runtime.snapshot().external_wrenches is None
    dyn = result.output.rollover_steps[0].dynamic_stability
    assert dyn is not None
    assert dyn.external_wrench_available is False
    assert dyn.external_wrench_included is False


def test_g_dynamic_empty_wrench_semantics(mock_config: RoverConfig) -> None:
    runtime = PredictionRuntime(mock_config, profile=PredictionProfile.DYNAMIC)
    _feed_static_ready(runtime)
    runtime.on_state(_state(acceleration_xyz=(0.0, 0.0, 0.0)), frame_id="map")
    runtime.on_external_wrenches([], frame_id="map")
    result = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=1,
        source_trajectory_stamp=100.0,
    )
    assert result.output is not None
    assert runtime.snapshot().external_wrenches == []
    dyn = result.output.rollover_steps[0].dynamic_stability
    assert dyn is not None
    assert dyn.external_wrench_available is True
    # Explicit empty batch: wrench channel is available and considered (included).
    assert dyn.external_wrench_included is True


def test_h_no_duplicate_prediction_after_dynamic(mock_config: RoverConfig) -> None:
    runtime = PredictionRuntime(mock_config, profile="dynamic")
    _feed_static_ready(runtime)
    runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=1,
        source_trajectory_stamp=100.0,
    )
    first = runtime.on_state(_state(acceleration_xyz=(1.0, 0.0, 0.0)), frame_id="map")
    assert first.output is not None

    second_state = runtime.on_state(_state(101.0, acceleration_xyz=(2.0, 0.0, 0.0)), frame_id="map")
    assert second_state.output is None
    assert second_state.duplicate_cycle is True

    wrench = runtime.on_external_wrenches(
        [
            ExternalWrenchData(
                source="boom",
                frame_id="map",
                force_xyz=(100.0, 0.0, 0.0),
                torque_xyz=(0.0, 0.0, 0.0),
                application_point_xyz=(0.0, 0.0, 1.0),
                confidence=1.0,
            )
        ],
        frame_id="map",
    )
    assert wrench.output is None
    assert wrench.duplicate_cycle is True


def test_i_new_trajectory_clears_state_for_dynamic(mock_config: RoverConfig) -> None:
    runtime = PredictionRuntime(mock_config, profile=PredictionProfile.DYNAMIC)
    _feed_static_ready(runtime, trajectory_id=10)
    runtime.on_state(_state(acceleration_xyz=(0.0, 1.0, 0.0)), frame_id="map")
    runtime.on_external_wrenches([], frame_id="map")
    first = runtime.on_geometry(
        _geometry(),
        frame_id="map",
        source_trajectory_id=10,
        source_trajectory_stamp=100.0,
    )
    assert first.output is not None

    # New cycle: prior state/wrench must not satisfy readiness.
    opened = runtime.on_trajectory(_trajectory(200.0), trajectory_id=11)
    snap = runtime.snapshot()
    assert snap.objects is None
    assert snap.geometry is None
    assert snap.state is None
    assert snap.external_wrenches is None
    assert opened.output is None
    assert "missing tracked objects batch" in opened.readiness.reasons
    assert "missing geometry batch" in opened.readiness.reasons
    assert "missing rover state" in opened.readiness.reasons

    runtime.on_objects([], frame_id="map")
    geo = runtime.on_geometry(
        _geometry(200.0),
        frame_id="map",
        source_trajectory_id=11,
        source_trajectory_stamp=200.0,
    )
    assert geo.output is None
    assert "missing rover state" in geo.readiness.reasons

    result = runtime.on_state(
        RoverState(timestamp=200.5, acceleration_xyz=(0.0, 0.0, 0.0)),
        frame_id="map",
    )
    assert result.output is not None
    assert result.cycle_key is not None
    assert result.cycle_key.trajectory_id == 11
