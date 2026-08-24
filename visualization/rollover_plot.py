"""Roll, pitch, and SSM profile visualization without decision thresholds."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from prediction_core.config import RoverConfig
from prediction_core.geometry_utils import projected_com_on_support_xy
from prediction_core.models import GeometryStep
from prediction_core.models import PredictionOutput, Trajectory


def save_rollover_profile(
    trajectory: Trajectory,
    output: PredictionOutput,
    path: str | Path,
    *,
    config: RoverConfig | None = None,
    geometry: list[GeometryStep] | None = None,
) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    distance_by_id = dict(
        zip((step.step_id for step in trajectory.steps), trajectory.cumulative_distances_m())
    )
    distances = [distance_by_id[step.step_id] for step in output.rollover_steps]
    roll = [step.predicted_roll_deg for step in output.rollover_steps]
    pitch = [step.predicted_pitch_deg for step in output.rollover_steps]
    margins = [step.static_stability_margin_m for step in output.rollover_steps]
    normalized_margins = [
        step.normalized_static_stability_margin for step in output.rollover_steps
    ]

    figure = plt.figure(figsize=(12, 9))
    grid = figure.add_gridspec(3, 2, width_ratios=(1.3, 1))
    attitude_axes = figure.add_subplot(grid[0, 0])
    raw_margin_axes = figure.add_subplot(grid[1, 0], sharex=attitude_axes)
    normalized_margin_axes = figure.add_subplot(grid[2, 0], sharex=attitude_axes)
    support_axes = figure.add_subplot(grid[:, 1])
    attitude_axes.plot(distances, roll, marker="o", label="roll (deg)")
    attitude_axes.plot(distances, pitch, marker="o", label="pitch (deg)")
    attitude_axes.set_ylabel("Predicted attitude (deg)")
    attitude_axes.grid(True, alpha=0.25)
    attitude_axes.legend(loc="best")
    raw_margin_axes.plot(
        distances, margins, color="tab:green", marker="o", label="raw SSM (m)"
    )
    raw_margin_axes.set_ylabel("Raw SSM (m)")
    raw_margin_axes.grid(True, alpha=0.25)
    raw_margin_axes.legend(loc="best")
    normalized_margin_axes.plot(
        distances,
        normalized_margins,
        color="tab:purple",
        marker="o",
        label="normalized SSM",
    )
    normalized_margin_axes.set_xlabel("Distance along route (m)")
    normalized_margin_axes.set_ylabel("Normalized SSM")
    normalized_margin_axes.grid(True, alpha=0.25)
    normalized_margin_axes.legend(loc="best")
    _draw_support_topdown(support_axes, trajectory, output, config, geometry)
    figure.suptitle("Quasi-static rollover evidence (no decision threshold)")
    figure.tight_layout()
    figure.savefig(destination, dpi=150)
    plt.close(figure)


def _draw_support_topdown(
    axes: plt.Axes,
    trajectory: Trajectory,
    output: PredictionOutput,
    config: RoverConfig | None,
    geometry: list[GeometryStep] | None,
) -> None:
    """Draw the support polygon and gravity-projected CoM for one valid step."""
    axes.set_title("Support polygon / CoM projection")
    axes.set_xlabel("Rover +X forward (m)")
    axes.set_ylabel("Rover +Y left (m)")
    axes.set_aspect("equal", adjustable="box")
    axes.grid(True, alpha=0.25)
    if config is None or geometry is None or not output.rollover_steps:
        axes.text(0.5, 0.5, "Geometry/config unavailable", ha="center", va="center")
        return

    first = output.rollover_steps[0]
    step_by_id = {step.step_id: step for step in trajectory.steps}
    terrain_by_step = {item.step_id: item for item in geometry}
    step = step_by_id.get(first.step_id)
    terrain = terrain_by_step.get(first.step_id)
    if step is None or terrain is None:
        axes.text(0.5, 0.5, "No matching geometry", ha="center", va="center")
        return

    projected = projected_com_on_support_xy(
        terrain.normal_xyz,
        step.yaw,
        config.com_x_m,
        config.com_y_m,
        config.com_height_m,
    )
    half_length = config.support_length_m / 2
    half_width = config.support_width_m / 2
    xs = [-half_length, half_length, half_length, -half_length, -half_length]
    ys = [-half_width, -half_width, half_width, half_width, -half_width]
    axes.plot(xs, ys, color="tab:blue", label="track support polygon")
    axes.scatter(config.com_x_m, config.com_y_m, color="tab:green", s=55, label="CoM")
    axes.scatter(*projected, color="tab:red", marker="x", s=65, label="static gravity projection")
    axes.plot(
        [config.com_x_m, projected[0]],
        [config.com_y_m, projected[1]],
        color="black",
        linestyle=":",
        label="gravity ray",
    )
    dynamic = first.dynamic_stability
    if dynamic is not None and dynamic.effective_gravity_projection_xy is not None:
        axes.scatter(
            *dynamic.effective_gravity_projection_xy,
            color="tab:orange",
            marker="+",
            s=70,
            label="effective-gravity projection",
        )
    if dynamic is not None and dynamic.zmp_xy is not None:
        axes.scatter(
            *dynamic.zmp_xy,
            color="tab:purple",
            marker="*",
            s=80,
            label="point-mass ZMP",
        )
    edge_candidates = (
        (half_length, projected[1], half_length - projected[0], "front edge"),
        (-half_length, projected[1], projected[0] + half_length, "rear edge"),
        (projected[0], half_width, half_width - projected[1], "left edge"),
        (projected[0], -half_width, projected[1] + half_width, "right edge"),
    )
    edge_x, edge_y, _, edge_name = min(edge_candidates, key=lambda candidate: candidate[2])
    axes.plot(
        [projected[0], edge_x],
        [projected[1], edge_y],
        color="tab:red",
        linewidth=2,
        label=f"nearest tipping edge ({edge_name})",
    )
    lines = [
        f"roll {first.predicted_roll_deg:.1f}°",
        f"pitch {first.predicted_pitch_deg:.1f}°",
        f"raw static SSM {first.static_stability_margin_m:.3f} m",
        f"normalized static SSM {first.normalized_static_stability_margin:.3f}",
    ]
    if first.critical_tip is not None:
        lines.append(
            f"critical tip {first.critical_tip.minimum_deg:.1f}° ({first.critical_tip.critical_edge})"
        )
    dynamic = first.dynamic_stability
    if dynamic is not None and dynamic.valid:
        if dynamic.effective_ssm_m is not None:
            lines.append(f"effective SSM {dynamic.effective_ssm_m:.3f} m")
        if dynamic.zmp_margin_m is not None:
            lines.append(f"ZMP margin {dynamic.zmp_margin_m:.3f} m")
        if dynamic.minimum_stability_moment_nm is not None:
            lines.append(f"min moment {dynamic.minimum_stability_moment_nm:.1f} N·m")
    axes.annotate(
        "\n".join(lines),
        xy=(0.03, 0.97),
        xycoords="axes fraction",
        va="top",
    )
    axes.legend(loc="best")

