from pathlib import Path

import pytest

from mock.scenario_generator import load_scenario
from prediction_core.config import RoverConfig
from prediction_core.models import GeometryStep, TrackedObject, Trajectory, TrajectoryStep
from prediction_core.predictor import PredictionCore


ROOT = Path(__file__).resolve().parents[1]


def test_combined_collision_and_rollover(mock_config: RoverConfig) -> None:
    scenario = load_scenario(ROOT / "mock/scenarios/pipe_collision.json")
    output = PredictionCore(mock_config).predict(
        scenario.trajectory, scenario.tracked_objects, scenario.geometry
    )
    assert output.source_trajectory_stamp == scenario.trajectory.timestamp
    assert output.collision_steps
    assert len(output.rollover_steps) == len(scenario.trajectory.steps)


def test_missing_geometry_skips_only_rollover(mock_config: RoverConfig, caplog) -> None:
    scenario = load_scenario(ROOT / "mock/scenarios/pipe_collision.json")
    core = PredictionCore(mock_config)
    output = core.predict(scenario.trajectory, scenario.tracked_objects, scenario.geometry[:-1])
    assert output.collision_steps
    assert len(output.rollover_steps) == len(scenario.trajectory.steps) - 1
    assert core.rollover_predictor.last_missing_step_ids == [10]
    assert "missing geometry" in caplog.text


def test_stale_timestamp_policy_is_opt_in(mock_config: RoverConfig) -> None:
    trajectory = Trajectory(100.0, "map", [TrajectoryStep(0, 0, 0, 0)])
    tracked_object = TrackedObject(
        90.0, "old", "debris", [(0, 0), (1, 0), (0, 1)]
    )
    geometry = GeometryStep(99.0, 0, "plane", (0, 0, 1))
    assert PredictionCore.stale_input_warnings(trajectory, [tracked_object], [geometry]) == []
    warnings = PredictionCore.stale_input_warnings(
        trajectory,
        [tracked_object],
        [geometry],
        max_object_age_s=5.0,
        max_geometry_age_s=0.5,
    )
    assert len(warnings) == 2


@pytest.mark.parametrize(
    "scenario_name",
    [
        "flat_empty",
        "pipe_collision",
        "near_margin_collision",
        "outside_margin",
        "uphill_15deg",
        "side_slope_15deg",
    ],
)
def test_all_mock_scenarios_have_complete_rollover(
    mock_config: RoverConfig, scenario_name: str
) -> None:
    scenario = load_scenario(ROOT / f"mock/scenarios/{scenario_name}.json")
    output = PredictionCore(mock_config).predict(
        scenario.trajectory, scenario.tracked_objects, scenario.geometry
    )
    assert len(output.rollover_steps) == len(scenario.trajectory.steps)
    if scenario_name == "flat_empty":
        assert output.collision_steps == []
    elif scenario_name == "pipe_collision":
        assert output.collision_steps
    elif scenario_name == "near_margin_collision":
        minimum = min(
            candidate.min_distance_m
            for step in output.collision_steps
            for candidate in step.collision_objects
        )
        assert minimum == pytest.approx(0.15)
    elif scenario_name == "outside_margin":
        assert output.collision_steps == []
    elif scenario_name == "uphill_15deg":
        assert output.rollover_steps[0].predicted_pitch_deg == pytest.approx(15.0)
    elif scenario_name == "side_slope_15deg":
        assert abs(output.rollover_steps[0].predicted_roll_deg) == pytest.approx(15.0)
