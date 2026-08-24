#!/usr/bin/env python3
"""Compare implemented pure-Python rollover metrics. No new algorithms. No ROS."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from prediction_core.config import load_config
from prediction_core.geometry_utils import (
    projected_com_on_support_xy,
    support_edge_margins,
    terrain_frame,
)
from prediction_core.models import (
    ExternalWrench,
    GeometryStep,
    RoverState,
    Trajectory,
    TrajectoryStep,
)
from prediction_core.rollover import GRAVITY_WORLD_M_S2, RolloverPredictor

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "outputs" / "rollover_method_comparison"
PLOT_DIR = OUT / "plots"
JSON_PATH = ROOT / "outputs" / "rollover_method_comparison.json"
REPORT_PATH = ROOT / "ROLLOVER_METHOD_COMPARISON.md"
G = abs(float(GRAVITY_WORLD_M_S2[2]))


def normal_for_roll_pitch(roll_deg: float = 0.0, pitch_deg: float = 0.0) -> tuple[float, float, float]:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    nx = -math.tan(pitch)
    ny = -math.tan(roll)
    nz = 1.0
    mag = math.hypot(math.hypot(nx, ny), nz)
    return nx / mag, ny / mag, nz / mag


def limiting_edge(point_xy, length_m: float, width_m: float) -> str | None:
    if point_xy is None:
        return None
    margins = support_edge_margins(point_xy, length_m, width_m)
    values = {
        "front": margins.front_m,
        "rear": margins.rear_m,
        "left": margins.left_m,
        "right": margins.right_m,
    }
    return min(values, key=values.get)


def edge_distance(point_xy, edge: str | None, length_m: float, width_m: float) -> float | None:
    if point_xy is None or edge is None:
        return None
    return float(getattr(support_edge_margins(point_xy, length_m, width_m), f"{edge}_m"))


@dataclass(frozen=True)
class Scenario:
    name: str
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw: float = 0.0
    acceleration_xyz: tuple[float, float, float] | None = (0.0, 0.0, 0.0)
    external_wrenches: list[ExternalWrench] | None = field(default_factory=list)
    notes: str = ""


def build_scenarios(config) -> list[Scenario]:
    tip_lat = math.degrees(math.atan((config.support_width_m / 2.0) / config.com_height_m))
    force = 250.0
    h = config.com_height_m
    return [
        Scenario("flat_static"),
        Scenario("uphill_15deg", pitch_deg=15.0),
        Scenario("downhill_15deg", pitch_deg=-15.0),
        Scenario("side_slope_left_15deg", roll_deg=15.0),
        Scenario("side_slope_right_15deg", roll_deg=-15.0),
        Scenario("near_static_tip", roll_deg=tip_lat - 0.5),
        Scenario("beyond_static_tip", roll_deg=tip_lat + 1.0),
        Scenario("flat_lateral_accel_1", acceleration_xyz=(0.0, 1.0, 0.0)),
        Scenario("flat_lateral_accel_2", acceleration_xyz=(0.0, 2.0, 0.0)),
        Scenario("flat_lateral_accel_4", acceleration_xyz=(0.0, 4.0, 0.0)),
        Scenario(
            "side_slope_plus_lateral_accel",
            roll_deg=10.0,
            acceleration_xyz=(0.0, 2.0, 0.0),
            notes="slope + ay stack; static may look OK while effective/ZMP degrade",
        ),
        Scenario("flat_forward_accel", acceleration_xyz=(2.0, 0.0, 0.0)),
        Scenario("flat_braking", acceleration_xyz=(-2.0, 0.0, 0.0)),
        Scenario(
            "external_lateral_force_at_com_height",
            external_wrenches=[
                ExternalWrench(
                    source="push",
                    force_xyz=(0.0, force, 0.0),
                    torque_xyz=(0.0, 0.0, 0.0),
                    application_point_xyz=(0.0, 0.0, h),
                )
            ],
        ),
        Scenario(
            "external_lateral_force_low",
            external_wrenches=[
                ExternalWrench(
                    source="push_low",
                    force_xyz=(0.0, force, 0.0),
                    torque_xyz=(0.0, 0.0, 0.0),
                    application_point_xyz=(0.0, 0.0, 0.05),
                )
            ],
        ),
        Scenario(
            "external_lateral_force_high",
            external_wrenches=[
                ExternalWrench(
                    source="push_high",
                    force_xyz=(0.0, force, 0.0),
                    torque_xyz=(0.0, 0.0, 0.0),
                    application_point_xyz=(0.0, 0.0, 0.90),
                )
            ],
        ),
        Scenario(
            "external_pure_roll_torque",
            external_wrenches=[
                ExternalWrench(
                    source="couple",
                    force_xyz=(0.0, 0.0, 0.0),
                    torque_xyz=(80.0, 0.0, 0.0),
                )
            ],
        ),
        Scenario(
            "combined_slope_accel_external_force",
            roll_deg=8.0,
            acceleration_xyz=(0.0, 1.5, 0.0),
            external_wrenches=[
                ExternalWrench(
                    source="push",
                    force_xyz=(0.0, 150.0, 0.0),
                    torque_xyz=(0.0, 0.0, 0.0),
                    application_point_xyz=(0.0, 0.0, 0.60),
                )
            ],
        ),
        Scenario("dynamic_beyond_tip", acceleration_xyz=(0.0, 20.0, 0.0)),
        Scenario("acceleration_unavailable", acceleration_xyz=None, external_wrenches=[]),
        Scenario("external_wrench_unavailable", acceleration_xyz=(0.0, 0.0, 0.0), external_wrenches=None),
        Scenario("external_wrench_explicit_empty", acceleration_xyz=(0.0, 0.0, 0.0), external_wrenches=[]),
    ]


def run_scenario(predictor: RolloverPredictor, scenario: Scenario) -> dict[str, Any]:
    config = predictor.config
    normal = normal_for_roll_pitch(scenario.roll_deg, scenario.pitch_deg)
    trajectory = Trajectory(
        timestamp=1.0,
        frame_id="map",
        steps=[TrajectoryStep(0, 0.0, 0.0, scenario.yaw)],
    )
    geometry = [
        GeometryStep(
            timestamp=1.0,
            step_id=0,
            plane_id=scenario.name,
            normal_xyz=normal,
            confidence=1.0,
        )
    ]
    state = (
        RoverState(timestamp=1.0)
        if scenario.acceleration_xyz is None
        else RoverState(timestamp=1.0, acceleration_xyz=scenario.acceleration_xyz)
    )
    step = predictor.predict(
        trajectory,
        geometry,
        state=state,
        external_wrenches=scenario.external_wrenches,
    )[0]
    tip = step.critical_tip
    dyn = step.dynamic_stability
    assert tip is not None and dyn is not None

    static_xy = projected_com_on_support_xy(
        normal, scenario.yaw, config.com_x_m, config.com_y_m, config.com_height_m
    )
    static_edge = limiting_edge(static_xy, config.support_length_m, config.support_width_m)
    eff_xy = dyn.effective_gravity_projection_xy
    eff_edge = limiting_edge(eff_xy, config.support_length_m, config.support_width_m)
    zmp_edge = limiting_edge(dyn.zmp_xy, config.support_length_m, config.support_width_m)
    moment_edge = dyn.critical_edge
    zmp_dist = edge_distance(dyn.zmp_xy, moment_edge, config.support_length_m, config.support_width_m)

    fz_support = m_pred = m_residual = m_actual = None
    if (
        dyn.valid
        and dyn.effective_force_xyz_n is not None
        and moment_edge is not None
        and zmp_dist is not None
        and dyn.edge_stability_moments_nm is not None
    ):
        rotation = terrain_frame(normal, scenario.yaw)
        f_support = rotation.T @ np.asarray(dyn.effective_force_xyz_n, dtype=float)
        fz_support = float(f_support[2])
        m_pred = (-fz_support) * zmp_dist
        m_actual = dyn.edge_stability_moments_nm[moment_edge]
        m_residual = m_actual - m_pred

    normalized_edge_moments = None
    if dyn.edge_stability_moments_nm is not None:
        ref = support_edge_margins(
            (config.com_x_m, config.com_y_m), config.support_length_m, config.support_width_m
        )
        refs = {
            "front": config.mass_kg * G * ref.front_m,
            "rear": config.mass_kg * G * ref.rear_m,
            "left": config.mass_kg * G * ref.left_m,
            "right": config.mass_kg * G * ref.right_m,
        }
        normalized_edge_moments = {
            edge: dyn.edge_stability_moments_nm[edge] / refs[edge] for edge in refs
        }

    return {
        "name": scenario.name,
        "notes": scenario.notes,
        "inputs": {
            "roll_deg_cmd": scenario.roll_deg,
            "pitch_deg_cmd": scenario.pitch_deg,
            "yaw": scenario.yaw,
            "normal_xyz": normal,
            "acceleration_xyz": scenario.acceleration_xyz,
            "external_wrenches": None
            if scenario.external_wrenches is None
            else [asdict(w) for w in scenario.external_wrenches],
        },
        "roll_deg": step.predicted_roll_deg,
        "pitch_deg": step.predicted_pitch_deg,
        "critical_tip_angles": {
            "front_deg": tip.front_deg,
            "rear_deg": tip.rear_deg,
            "left_deg": tip.left_deg,
            "right_deg": tip.right_deg,
            "minimum_deg": tip.minimum_deg,
        },
        "critical_tip_edge": tip.critical_edge,
        "static_projection_xy": list(static_xy),
        "static_ssm_m": step.static_stability_margin_m,
        "normalized_static_ssm": step.normalized_static_stability_margin,
        "static_limiting_edge": static_edge,
        "effective_projection_xy": None if eff_xy is None else list(eff_xy),
        "effective_ssm_m": dyn.effective_ssm_m,
        "normalized_effective_ssm": dyn.normalized_effective_ssm,
        "effective_limiting_edge": eff_edge,
        "zmp_xy": None if dyn.zmp_xy is None else list(dyn.zmp_xy),
        "zmp_margin_m": dyn.zmp_margin_m,
        "normalized_zmp_margin": dyn.normalized_zmp_margin,
        "zmp_limiting_edge": zmp_edge,
        "edge_stability_moments_nm": dyn.edge_stability_moments_nm,
        "normalized_edge_stability_moments": normalized_edge_moments,
        "minimum_stability_moment_nm": dyn.minimum_stability_moment_nm,
        "normalized_minimum_stability_moment": dyn.normalized_minimum_stability_moment,
        "moment_critical_edge": moment_edge,
        "dynamic_valid": dyn.valid,
        "acceleration_available": dyn.acceleration_available,
        "external_wrench_available": dyn.external_wrench_available,
        "external_wrench_included": dyn.external_wrench_included,
        "validity_reason": dyn.validity_reason,
        "assumptions": list(dyn.assumptions),
        "comparisons": {
            "eff_minus_static_ssm_m": None
            if dyn.effective_ssm_m is None
            else dyn.effective_ssm_m - step.static_stability_margin_m,
            "eff_xy_minus_zmp_xy": None
            if eff_xy is None or dyn.zmp_xy is None
            else [eff_xy[0] - dyn.zmp_xy[0], eff_xy[1] - dyn.zmp_xy[1]],
            "eff_ssm_minus_zmp_margin_m": None
            if dyn.effective_ssm_m is None or dyn.zmp_margin_m is None
            else dyn.effective_ssm_m - dyn.zmp_margin_m,
            "moment_vs_minus_Fz_times_zmp_distance": {
                "fz_support": fz_support,
                "zmp_signed_distance_to_moment_edge_m": zmp_dist,
                "predicted_nm": m_pred,
                "actual_nm": m_actual,
                "residual_nm": m_residual,
            },
        },
    }


def sweep_lateral_accel(predictor: RolloverPredictor) -> list[dict[str, Any]]:
    rows = []
    for ay in np.linspace(0.0, 8.0, 17):
        rows.append(
            run_scenario(
                predictor,
                Scenario(
                    name=f"sweep_ay_{ay:.2f}",
                    acceleration_xyz=(0.0, float(ay), 0.0),
                    external_wrenches=[],
                ),
            )
        )
    return rows


def sweep_force_height(predictor: RolloverPredictor) -> list[dict[str, Any]]:
    rows = []
    for height in np.linspace(0.05, 0.95, 10):
        rows.append(
            run_scenario(
                predictor,
                Scenario(
                    name=f"sweep_force_h_{height:.2f}",
                    acceleration_xyz=(0.0, 0.0, 0.0),
                    external_wrenches=[
                        ExternalWrench(
                            source="push",
                            force_xyz=(0.0, 250.0, 0.0),
                            torque_xyz=(0.0, 0.0, 0.0),
                            application_point_xyz=(0.0, 0.0, float(height)),
                        )
                    ],
                ),
            )
        )
    return rows


def make_plots(accel_rows, height_rows, combined, config) -> list[str]:
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    paths: list[str] = []

    ay = [r["inputs"]["acceleration_xyz"][1] for r in accel_rows]
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(ay, [r["static_ssm_m"] for r in accel_rows], label="static SSM", lw=2)
    ax.plot(ay, [r["effective_ssm_m"] for r in accel_rows], label="effective SSM", lw=2)
    ax.plot(ay, [r["zmp_margin_m"] for r in accel_rows], "--", label="ZMP margin", lw=2)
    ax.set_xlabel("lateral acceleration a_y (m/s^2)")
    ax.set_ylabel("margin (m)")
    ax.set_title("Flat ground: static vs effective SSM vs ZMP margin")
    ax.grid(True, alpha=0.3)
    ax.legend()
    p1 = PLOT_DIR / "plot1_lateral_accel_margins.png"
    fig.tight_layout(); fig.savefig(p1, dpi=140); plt.close(fig)
    paths.append(str(p1.relative_to(ROOT)))

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(ay, [r["normalized_minimum_stability_moment"] for r in accel_rows], color="tab:purple", lw=2)
    ax.set_xlabel("lateral acceleration a_y (m/s^2)")
    ax.set_ylabel("normalized minimum stability moment")
    ax.set_title("Flat ground: normalized stability moment vs lateral accel")
    ax.grid(True, alpha=0.3)
    p2 = PLOT_DIR / "plot2_lateral_accel_norm_moment.png"
    fig.tight_layout(); fig.savefig(p2, dpi=140); plt.close(fig)
    paths.append(str(p2.relative_to(ROOT)))

    heights = [r["inputs"]["external_wrenches"][0]["application_point_xyz"][2] for r in height_rows]
    fig, ax1 = plt.subplots(figsize=(8, 5))
    ax1.plot(heights, [r["zmp_margin_m"] for r in height_rows], color="tab:blue")
    ax1.set_xlabel("external force application height (m)")
    ax1.set_ylabel("ZMP margin (m)", color="tab:blue")
    ax2 = ax1.twinx()
    ax2.plot(heights, [r["minimum_stability_moment_nm"] for r in height_rows], color="tab:red")
    ax2.set_ylabel("minimum stability moment (N*m)", color="tab:red")
    ax1.set_title("Same lateral force: height vs ZMP margin / moment")
    ax1.grid(True, alpha=0.3)
    p3 = PLOT_DIR / "plot3_force_height_zmp_moment.png"
    fig.tight_layout(); fig.savefig(p3, dpi=140); plt.close(fig)
    paths.append(str(p3.relative_to(ROOT)))

    fig, ax = plt.subplots(figsize=(6, 6))
    half_l = config.support_length_m / 2
    half_w = config.support_width_m / 2
    ax.plot([-half_l, half_l, half_l, -half_l, -half_l], [-half_w, -half_w, half_w, half_w, -half_w],
            color="tab:blue", label="support polygon")
    ax.scatter([config.com_x_m], [config.com_y_m], c="green", s=60, label="CoM xy")
    ax.scatter(*combined["static_projection_xy"], c="black", marker="x", s=70, label="static proj")
    if combined["effective_projection_xy"] is not None:
        ax.scatter(*combined["effective_projection_xy"], c="orange", marker="+", s=80, label="effective proj")
    if combined["zmp_xy"] is not None:
        ax.scatter(*combined["zmp_xy"], c="purple", marker="*", s=90, label="ZMP")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("support +X forward (m)")
    ax.set_ylabel("support +Y left (m)")
    ax.set_title(f"Support snapshot: {combined['name']}")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    p4 = PLOT_DIR / "plot4_support_snapshot_combined.png"
    fig.tight_layout(); fig.savefig(p4, dpi=140); plt.close(fig)
    paths.append(str(p4.relative_to(ROOT)))
    return paths


def _is_wrench_free(result: dict[str, Any]) -> bool:
    wrenches = result["inputs"]["external_wrenches"]
    return wrenches is None or wrenches == []


def analyze(results: list[dict[str, Any]], accel_rows: list[dict[str, Any]]) -> dict[str, Any]:
    equiv = [
        r for r in results
        if _is_wrench_free(r) and r["inputs"]["acceleration_xyz"] is not None and r["dynamic_valid"]
    ]
    max_xy = max_m = 0.0
    residuals: list[float] = []
    for r in equiv:
        dxy = r["comparisons"]["eff_xy_minus_zmp_xy"]
        dm = r["comparisons"]["eff_ssm_minus_zmp_margin_m"]
        if dxy is not None:
            max_xy = max(max_xy, abs(dxy[0]), abs(dxy[1]))
        if dm is not None:
            max_m = max(max_m, abs(dm))
        residual = r["comparisons"]["moment_vs_minus_Fz_times_zmp_distance"]["residual_nm"]
        if residual is not None:
            residuals.append(abs(residual))

    torque = next(r for r in results if r["name"] == "external_pure_roll_torque")
    torque_residual = torque["comparisons"]["moment_vs_minus_Fz_times_zmp_distance"]["residual_nm"]

    interesting = []
    for r in results:
        if r["normalized_effective_ssm"] is None:
            continue
        if r["normalized_static_ssm"] > 0.7 and r["normalized_effective_ssm"] < r["normalized_static_ssm"] - 0.15:
            interesting.append({
                "name": r["name"],
                "normalized_static_ssm": r["normalized_static_ssm"],
                "normalized_effective_ssm": r["normalized_effective_ssm"],
                "static_ssm_m": r["static_ssm_m"],
                "effective_ssm_m": r["effective_ssm_m"],
            })

    disagreements = []
    for r in results:
        edges = {
            "static": r["static_limiting_edge"],
            "effective": r["effective_limiting_edge"],
            "zmp": r["zmp_limiting_edge"],
            "moment": r["moment_critical_edge"],
        }
        present = {k: v for k, v in edges.items() if v is not None}
        if len(set(present.values())) > 1:
            disagreements.append({"name": r["name"], "edges": present})

    height_group = [
        r for r in results
        if r["name"] in {
            "external_lateral_force_low",
            "external_lateral_force_at_com_height",
            "external_lateral_force_high",
        }
    ]
    none_wrench = next(r for r in results if r["name"] == "external_wrench_unavailable")
    empty_wrench = next(r for r in results if r["name"] == "external_wrench_explicit_empty")
    missing_accel = next(r for r in results if r["name"] == "acceleration_unavailable")
    height_eff_constant = len({r["effective_ssm_m"] for r in height_group}) == 1

    wrench_cases = [r for r in results if not _is_wrench_free(r) and r["dynamic_valid"]]
    wrench_diffs = [
        abs(r["comparisons"]["eff_ssm_minus_zmp_margin_m"])
        for r in wrench_cases
        if r["comparisons"]["eff_ssm_minus_zmp_margin_m"] is not None
    ]

    return {
        "n_scenarios": len(results),
        "eff_vs_zmp_empty_wrench": {
            "n_cases": len(equiv),
            "max_abs_xy_error_m": max_xy,
            "max_abs_margin_error_m": max_m,
            "equivalent_within_1e-9": max_xy < 1e-9 and max_m < 1e-9,
        },
        "eff_vs_zmp_with_wrenches": {
            "n_cases": len(wrench_cases),
            "max_abs_margin_error_m": max(wrench_diffs) if wrench_diffs else None,
            "diverges": bool(wrench_diffs) and max(wrench_diffs) > 1e-6,
        },
        "moment_vs_minus_Fz_times_distance": {
            "n_cases": len(residuals),
            "max_abs_residual_nm": max(residuals) if residuals else None,
            "mean_abs_residual_nm": float(np.mean(residuals)) if residuals else None,
            "holds_within_1e-6": bool(residuals) and max(residuals) < 1e-6,
            "pure_torque_residual_nm": torque_residual,
        },
        "force_height_effective_ssm_constant": height_eff_constant,
        "static_ok_but_effective_worse": interesting,
        "edge_disagreements": disagreements,
        "force_height_group": [
            {
                "name": r["name"],
                "effective_ssm_m": r["effective_ssm_m"],
                "zmp_margin_m": r["zmp_margin_m"],
                "minimum_stability_moment_nm": r["minimum_stability_moment_nm"],
            }
            for r in height_group
        ],
        "pure_torque": {
            "static_ssm_m": torque["static_ssm_m"],
            "effective_ssm_m": torque["effective_ssm_m"],
            "zmp_xy": torque["zmp_xy"],
            "zmp_margin_m": torque["zmp_margin_m"],
            "minimum_stability_moment_nm": torque["minimum_stability_moment_nm"],
        },
        "none_vs_empty_wrench": {
            "none_included": none_wrench["external_wrench_included"],
            "empty_included": empty_wrench["external_wrench_included"],
            "same_numeric_margins": (
                none_wrench["zmp_margin_m"] == empty_wrench["zmp_margin_m"]
                and none_wrench["effective_ssm_m"] == empty_wrench["effective_ssm_m"]
            ),
            "note": "Numeric margins match when unloaded; None means wrench info unavailable.",
        },
        "acceleration_unavailable": {
            "static_ssm_m": missing_accel["static_ssm_m"],
            "critical_tip_available": missing_accel["critical_tip_angles"] is not None,
            "effective_ssm_m": missing_accel["effective_ssm_m"],
            "zmp_margin_m": missing_accel["zmp_margin_m"],
            "dynamic_valid": missing_accel["dynamic_valid"],
            "validity_reason": missing_accel["validity_reason"],
        },
        "accel_sweep_static_constant": all(
            abs(r["static_ssm_m"] - accel_rows[0]["static_ssm_m"]) < 1e-12 for r in accel_rows
        ),
    }


def write_report(analysis, results, plot_paths, pytest_count: str) -> None:
    tip = results[0]["critical_tip_angles"]
    flat = next(r for r in results if r["name"] == "flat_static")
    justify = next(r for r in results if r["name"] == "flat_lateral_accel_4")
    side = next(r for r in results if r["name"] == "side_slope_plus_lateral_accel")
    beyond = next(r for r in results if r["name"] == "dynamic_beyond_tip")
    low = next(r for r in results if r["name"] == "external_lateral_force_low")
    high = next(r for r in results if r["name"] == "external_lateral_force_high")
    equiv = analysis["eff_vs_zmp_empty_wrench"]
    moment_rel = analysis["moment_vs_minus_Fz_times_distance"]
    wrench_div = analysis["eff_vs_zmp_with_wrenches"]

    zmp_note = (
        "Under gravity + translational acceleration with empty wrenches, Effective SSM and "
        "point-mass ZMP encode the same resultant line of action "
        f"(max |dxy|={equiv['max_abs_xy_error_m']:.3e} m, "
        f"max |dmargin|={equiv['max_abs_margin_error_m']:.3e} m; "
        f"equivalent={equiv['equivalent_within_1e-9']}). "
        "They are largely redundant in the accel-only regime."
    )
    wrench_note = (
        f"With external wrenches ({wrench_div['n_cases']} cases), Effective SSM vs ZMP "
        f"max |margin diff|={wrench_div['max_abs_margin_error_m']}; "
        f"diverges={wrench_div['diverges']}."
    )
    moment_note = (
        "Under the point-mass ZMP definition, M_edge ~= (-Fz_support) * signed_ZMP_distance_to_edge "
        f"(max |residual| wrench-free={moment_rel['max_abs_residual_nm']:.3e} N*m; "
        f"pure-torque residual={moment_rel['pure_torque_residual_nm']:.3e} N*m). "
        "Holds by construction once ZMP includes all support-plane moments."
    )
    plots = "\n".join(f"  - `{p}`" for p in plot_paths)

    REPORT_PATH.write_text(f"""# Rollover Method Comparison (Pure Python)

No new algorithms. No ROS work.

Pytest: **{pytest_count}**
Scenarios: **{analysis["n_scenarios"]}**

## 1. Current implemented methods

1. Critical geometric tip angle
2. Static SSM
3. Normalized Static SSM
4. Effective-gravity / inertial SSM
5. Stability Moment / Moment Balance
6. Point-mass / translational ZMP

Not implemented: canonical FASM, full rigid-body ZMP, LTR, multibody, Decision thresholds.

## 2. What each method actually calculates

| Method | Calculates | Answers |
|---|---|---|
| Critical tip angle | atan(reference_margin_i / com_height) on flat configured support | Ideal geometric slope putting CoM ray on an edge for this chassis |
| Static SSM | Gravity projection of CoM onto support; min signed edge margin | Remaining geometric margin for this terrain/heading |
| Normalized Static SSM | Edge margin / flat reference margin; take min | Dimensionless static margin vs flat design margins |
| Effective SSM | Same as static along g_eff = g - a | How translational acceleration moves the support intercept |
| Point-mass ZMP | Support point from non-contact wrench (x=-My/Fz, y=Mx/Fz) | Resultant-force contact location under point-mass assumptions |
| Stability Moment | Restoring moment about each edge from gravity, -ma, wrenches | Moment budget to tip; critical by min normalized moment |

Verified live formulas from `prediction_core/rollover.py`.

## 3. Required inputs

| Method | Terrain+pose | Config | Accel | External wrench |
|---|---|---|---|---|
| Tip angle | no | yes | no | no |
| Static/normalized SSM | yes | yes | no | no |
| Effective SSM | yes | yes | required (None => unavailable) | ignored for projection |
| Point-mass ZMP | yes | yes | required | used when list provided |
| Stability Moment | yes | yes | required for dynamic package | used when list provided |

`acceleration_xyz=None` is not `(0,0,0)`.
`external_wrenches=None` is not `[]`.

## 4. Static cases

Flat tip angles: front/rear ~ **{tip["front_deg"]:.2f} deg**, left/right ~ **{tip["left_deg"]:.2f} deg**,
critical tip edge `{results[0]["critical_tip_edge"]}`.

Tip angles do **not** change with terrain or acceleration (chassis property).

Flat static SSM = **{flat["static_ssm_m"]:.6f} m**, normalized = **{flat["normalized_static_ssm"]:.6f}**.
On slopes with a=(0,0,0) and empty wrenches, effective SSM matches static SSM.

Normalized moment on flat static is ~1. On slopes, normalized moment need not equal normalized SSM
because M_ref = m g m_ref uses flat reference margins while restoring moment depends on support-frame forces.

## 5. Acceleration cases

On flat ground, static SSM stays constant while effective SSM and ZMP margin fall with lateral accel
(`accel_sweep_static_constant={analysis["accel_sweep_static_constant"]}`).

Primary justification (`{justify["name"]}`):
- normalized static SSM = **{justify["normalized_static_ssm"]:.4f}** (unchanged vs flat)
- normalized effective SSM = **{justify["normalized_effective_ssm"]:.4f}**
- static SSM = {justify["static_ssm_m"]:.4f} m, effective SSM = {justify["effective_ssm_m"]:.4f} m

Combined slope+accel (`{side["name"]}`):
- normalized static = **{side["normalized_static_ssm"]:.4f}**
- normalized effective = **{side["normalized_effective_ssm"]:.4f}**
- static SSM = {side["static_ssm_m"]:.4f} m, effective SSM = {side["effective_ssm_m"]:.4f} m

Beyond tip (`{beyond["name"]}`): ZMP margin = {beyond["zmp_margin_m"]:.4f} m,
min moment = {beyond["minimum_stability_moment_nm"]:.2f} N*m.

## 6. External force / torque cases

Same force, low vs high application point:
- low: ZMP={low["zmp_margin_m"]:.4f} m, moment={low["minimum_stability_moment_nm"]:.2f} N*m
- high: ZMP={high["zmp_margin_m"]:.4f} m, moment={high["minimum_stability_moment_nm"]:.2f} N*m
- Effective SSM unchanged at {low["effective_ssm_m"]:.4f} m for both
  (`force_height_effective_ssm_constant={analysis["force_height_effective_ssm_constant"]}`)

Pure torque: static/effective SSM stay flat-static while ZMP/moment move
(`zmp_xy={analysis["pure_torque"]["zmp_xy"]}`).

None vs empty wrench: numeric margins match when unloaded, but
`external_wrench_included` is False for None and True for [].

## 7. Mathematical redundancy / equivalence analysis

### Effective SSM vs point-mass ZMP
{zmp_note}

{wrench_note}

### ZMP vs Stability Moment
{moment_note}

Edge disagreements: **{len(analysis["edge_disagreements"])}** scenarios (see JSON).
Moment critical edge uses min **normalized** moment; SSM/ZMP use raw min margin.

## 8. Advantages and limitations

| Method | Advantage | Limitation |
|---|---|---|
| Tip angle | Design intuition | Not a route-step metric |
| Static SSM | Cheap continuous terrain margin | Blind to accel/loads |
| Effective SSM | Accel effect without wrench plumbing | Ignores external wrenches |
| Point-mass ZMP | Accel + wrench-aware contact point | Not rigid-body ZMP |
| Stability Moment | Force height + pure torque; normalized budget | Needs accel; no rotational inertia |

## 9. Classification and recommendation

| Method | Class |
|---|---|
| Critical geometric tip angle | USEFUL DIAGNOSTIC / REDUNDANT for per-step decisions |
| Static SSM | BASELINE |
| Normalized Static SSM | BASELINE |
| Effective-gravity / inertial SSM | KEEP AS DYNAMIC EXTENSION (accel-only) / diagnostic if ZMP kept |
| Point-mass / translational ZMP | KEEP AS DYNAMIC EXTENSION when wrenches matter; redundant with Effective SSM for accel-only |
| Stability Moment / Moment Balance | KEEP AS DYNAMIC EXTENSION (best wrench-aware scalar) |

### Minimal useful stack today

1. Terrain roll/pitch
2. Static SSM + normalized Static SSM
3. Stability Moment as primary wrench-aware dynamic metric
4. Optional ZMP for operator visualization
5. Tip angles as config/diagnostic only
6. Effective SSM optional if wrench topics may be None and you still want accel-only evidence; otherwise redundant with ZMP when wrenches=`[]`

## 10. Missing FASM / full ZMP / LTR

- **FASM**: not implemented; needs agreed canonical formula; overlaps Stability Moment under current point-mass wrench set. Do not label Stability Moment as FASM.
- **Full ZMP**: needs inertia tensor, angular velocity/acceleration, rotational inertial terms.
- **LTR**: needs left/right vertical contact loads or a validated estimator; not available now.

## 11. Landfill Rover recommendation

Keep static baseline. Add one wrench-aware dynamic metric (Stability Moment recommended; ZMP optional visual).
Treat Effective SSM as accel-only diagnostic. Publish tip angles as vehicle properties, not per-step alarms.
Defer FASM/LTR/full ZMP until required inputs and formulas exist.

## Artifacts

- `outputs/rollover_method_comparison.json`
- Plots:
{plots}
""")
    print("wrote", REPORT_PATH)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    config = load_config(ROOT / "config" / "rover.mock.yaml")
    predictor = RolloverPredictor(config)
    scenarios = build_scenarios(config)
    results = [run_scenario(predictor, s) for s in scenarios]
    accel_rows = sweep_lateral_accel(predictor)
    height_rows = sweep_force_height(predictor)
    combined = next(r for r in results if r["name"] == "combined_slope_accel_external_force")
    plot_paths = make_plots(accel_rows, height_rows, combined, config)
    summary = analyze(results, accel_rows)
    payload = {
        "config": {
            "mass_kg": config.mass_kg,
            "support_length_m": config.support_length_m,
            "support_width_m": config.support_width_m,
            "com_x_m": config.com_x_m,
            "com_y_m": config.com_y_m,
            "com_height_m": config.com_height_m,
            "gravity_world_m_s2": GRAVITY_WORLD_M_S2,
        },
        "scenarios": results,
        "sweeps": {"lateral_acceleration": accel_rows, "force_height": height_rows},
        "analysis": summary,
        "plots": plot_paths,
    }
    JSON_PATH.write_text(json.dumps(payload, indent=2) + "\n")
    print("wrote", JSON_PATH)
    write_report(summary, results, plot_paths, pytest_count="(pending final pytest)")
    print(json.dumps(summary["eff_vs_zmp_empty_wrench"], indent=2))
    print(json.dumps(summary["moment_vs_minus_Fz_times_distance"], indent=2))
    side = next(r for r in results if r["name"] == "side_slope_plus_lateral_accel")
    print("side_slope_plus", side["normalized_static_ssm"], side["normalized_effective_ssm"],
          side["static_ssm_m"], side["effective_ssm_m"])
    print("scenarios", summary["n_scenarios"])


if __name__ == "__main__":
    main()
