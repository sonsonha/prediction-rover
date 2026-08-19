"""Terrain-attitude R0 and quasi-static stability-margin R1 predictors."""

from __future__ import annotations

import logging
import math

from .config import RoverConfig
from .geometry_utils import (
    normalized_static_stability_margin,
    projected_com_on_support_xy,
    support_edge_margins,
    terrain_roll_pitch_rad,
)
from .models import GeometryStep, RolloverStep, Trajectory


LOGGER = logging.getLogger(__name__)


class RolloverPredictor:
    """Predict terrain-following attitude and quasi-static SSM at each step."""

    def __init__(self, config: RoverConfig) -> None:
        self.config = config
        self.last_missing_step_ids: list[int] = []

    def predict(
        self, trajectory: Trajectory, geometry: list[GeometryStep]
    ) -> list[RolloverStep]:
        geometry_by_step: dict[int, GeometryStep] = {}
        for geometry_step in geometry:
            if geometry_step.step_id in geometry_by_step:
                raise ValueError(f"duplicate GeometryStep for step_id {geometry_step.step_id}")
            geometry_by_step[geometry_step.step_id] = geometry_step

        self.last_missing_step_ids = []
        output: list[RolloverStep] = []
        reference_margins = support_edge_margins(
            (self.config.com_x_m, self.config.com_y_m),
            self.config.support_length_m,
            self.config.support_width_m,
        )
        for step in trajectory.steps:
            terrain = geometry_by_step.get(step.step_id)
            if terrain is None:
                self.last_missing_step_ids.append(step.step_id)
                LOGGER.warning("missing geometry for trajectory step_id=%s", step.step_id)
                continue
            roll_rad, pitch_rad = terrain_roll_pitch_rad(terrain.normal_xyz, step.yaw)
            projected_com = projected_com_on_support_xy(
                terrain.normal_xyz,
                step.yaw,
                self.config.com_x_m,
                self.config.com_y_m,
                self.config.com_height_m,
            )
            current_margins = support_edge_margins(
                projected_com,
                self.config.support_length_m,
                self.config.support_width_m,
            )
            margin = current_margins.minimum_m()
            normalized_margin = normalized_static_stability_margin(
                current_margins, reference_margins
            )
            output.append(
                RolloverStep(
                    step_id=step.step_id,
                    predicted_roll_deg=math.degrees(roll_rad),
                    predicted_pitch_deg=math.degrees(pitch_rad),
                    static_stability_margin_m=margin,
                    normalized_static_stability_margin=normalized_margin,
                    terrain_id=terrain.plane_id,
                    confidence_or_validity=terrain.confidence,
                )
            )
        return output

