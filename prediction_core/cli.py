"""Command-line runner for deterministic mock scenarios."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
from typing import Sequence

from mock.scenario_generator import Scenario, load_scenario
from visualization.rollover_plot import save_rollover_profile
from visualization.topdown import save_collision_topdown

from .config import load_config
from .predictor import PredictionCore
from .serialization import write_json


LOGGER = logging.getLogger(__name__)


def run_scenario(scenario_path: Path, config_path: Path, output_dir: Path) -> str:
    scenario: Scenario = load_scenario(scenario_path)
    config = load_config(config_path)
    predictor = PredictionCore(config)
    prediction = predictor.predict(
        scenario.trajectory, scenario.tracked_objects, scenario.geometry
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    write_json(output_dir / "prediction_output.json", prediction)
    save_collision_topdown(
        scenario.trajectory,
        scenario.tracked_objects,
        config,
        prediction,
        output_dir / "collision_topdown.png",
    )
    save_rollover_profile(
        scenario.trajectory,
        prediction,
        output_dir / "rollover_profile.png",
        config=config,
        geometry=scenario.geometry,
    )
    candidate_count = sum(len(step.collision_objects) for step in prediction.collision_steps)
    summary = "\n".join(
        [
            f"scenario: {scenario.name}",
            f"trajectory_steps: {len(scenario.trajectory.steps)}",
            f"collision_steps: {len(prediction.collision_steps)}",
            f"collision_candidates: {candidate_count}",
            f"rollover_steps: {len(prediction.rollover_steps)}",
            f"missing_geometry_step_ids: {predictor.rollover_predictor.last_missing_step_ids}",
            "decision_status: intentionally not computed",
        ]
    ) + "\n"
    (output_dir / "run_summary.txt").write_text(summary, encoding="utf-8")
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    one = subparsers.add_parser("run-scenario", help="run one JSON mock")
    one.add_argument("scenario", type=Path)
    one.add_argument("--config", type=Path, required=True)
    one.add_argument("--output", type=Path, required=True)
    all_mocks = subparsers.add_parser("run-all-mocks", help="run all bundled JSON mocks")
    all_mocks.add_argument("--config", type=Path, required=True)
    all_mocks.add_argument("--output", type=Path, required=True)
    all_mocks.add_argument(
        "--scenarios-dir", type=Path, default=Path("mock/scenarios")
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    arguments = build_parser().parse_args(argv)
    if arguments.command == "run-scenario":
        print(run_scenario(arguments.scenario, arguments.config, arguments.output), end="")
        return 0
    scenario_paths = sorted(arguments.scenarios_dir.glob("*.json"))
    if not scenario_paths:
        raise SystemExit(f"no JSON scenarios found in {arguments.scenarios_dir}")
    for scenario_path in scenario_paths:
        output_dir = arguments.output / scenario_path.stem
        LOGGER.info("running %s", scenario_path)
        print(run_scenario(scenario_path, arguments.config, output_dir), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

