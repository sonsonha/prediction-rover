"""Export Prediction V1 demo artifacts (JSON + plots + summary)."""

from __future__ import annotations

from pathlib import Path

from prediction_core.config import RoverConfig
from prediction_core.models import GeometryStep, PredictionOutput, TrackedObject, Trajectory
from prediction_core.serialization import write_json
from visualization.rollover_plot import save_rollover_profile
from visualization.topdown import save_collision_topdown


def export_prediction_artifacts(
    *,
    output_dir: Path,
    prediction: PredictionOutput,
    trajectory: Trajectory,
    tracked_objects: list[TrackedObject],
    geometry: list[GeometryStep],
    config: RoverConfig,
    profile: str,
    scenario_name: str,
    cycle_id: int | None = None,
) -> dict[str, Path]:
    """Write human-readable summary + JSON + PNG plots for one prediction."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    paths: dict[str, Path] = {}
    paths["prediction_json"] = output_dir / "prediction_output.json"
    write_json(
        paths["prediction_json"],
        {
            "scenario": scenario_name,
            "prediction_profile": profile,
            "cycle_id": cycle_id,
            "prediction": prediction,
        },
    )

    paths["collision_png"] = output_dir / "collision_topdown.png"
    save_collision_topdown(
        trajectory,
        tracked_objects,
        config,
        prediction,
        paths["collision_png"],
    )

    paths["rollover_png"] = output_dir / "rollover_profile.png"
    save_rollover_profile(
        trajectory,
        prediction,
        paths["rollover_png"],
        config=config,
        geometry=geometry,
        title=(
            f"{scenario_name}  |  profile={profile}"
            + (f"  |  cycle={cycle_id}" if cycle_id is not None else "")
        ),
    )

    paths["summary_txt"] = output_dir / "run_summary.txt"
    paths["summary_txt"].write_text(
        _summary_text(
            scenario_name=scenario_name,
            profile=profile,
            cycle_id=cycle_id,
            prediction=prediction,
            paths=paths,
        ),
        encoding="utf-8",
    )
    return paths


def _summary_text(
    *,
    scenario_name: str,
    profile: str,
    cycle_id: int | None,
    prediction: PredictionOutput,
    paths: dict[str, Path],
) -> str:
    candidate_count = sum(len(step.collision_objects) for step in prediction.collision_steps)
    lines = [
        f"scenario: {scenario_name}",
        f"profile: {profile}",
        f"cycle_id: {cycle_id if cycle_id is not None else 'n/a'}",
        f"trajectory_stamp: {prediction.source_trajectory_stamp}",
        f"collision_steps: {len(prediction.collision_steps)}",
        f"collision_candidates: {candidate_count}",
        f"rollover_steps: {len(prediction.rollover_steps)}",
        "decision_status: intentionally not computed",
        "",
        "artifacts:",
    ]
    for key, path in paths.items():
        lines.append(f"  {key}: {path}")

    if prediction.rollover_steps:
        step = prediction.rollover_steps[0]
        dyn = step.dynamic_stability
        lines.extend(
            [
                "",
                f"step[{step.step_id}] snapshot:",
                f"  roll_deg: {step.predicted_roll_deg:.2f}",
                f"  pitch_deg: {step.predicted_pitch_deg:.2f}",
                f"  static_ssm_m: {step.static_stability_margin_m:.4f}",
                f"  normalized_static_ssm: {step.normalized_static_stability_margin:.4f}",
                f"  nearest_static_edge: {step.nearest_static_edge}",
            ]
        )
        if dyn is not None and dyn.valid:
            lines.extend(
                [
                    f"  stability_moment_nm: {dyn.minimum_stability_moment_nm}",
                    f"  normalized_moment: {dyn.normalized_minimum_stability_moment}",
                    f"  limiting_moment_edge: {dyn.minimum_normalized_moment_edge or dyn.critical_edge}",
                    f"  zmp_margin_m: {dyn.zmp_margin_m}",
                    f"  effective_ssm_m: {dyn.effective_ssm_m}",
                    f"  acceleration_available: {dyn.acceleration_available}",
                    f"  external_wrench_available: {dyn.external_wrench_available}",
                    f"  external_wrench_included: {dyn.external_wrench_included}",
                ]
            )
        elif dyn is not None:
            lines.append(f"  dynamic_validity: {dyn.validity_reason}")
    return "\n".join(lines) + "\n"
