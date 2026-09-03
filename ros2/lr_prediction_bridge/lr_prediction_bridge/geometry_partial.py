"""Pure helpers for partial GeometryArray construction (no ROS spin).

Missing terrain normals are omitted (UNKNOWN), not treated as flat ground,
unless ``allow_flat_fallback`` is explicitly enabled for smoke tests.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable, Optional, Protocol


class TrajectoryStepLike(Protocol):
    step_id: int
    x: float
    y: float


# (nx, ny, nz, confidence, confidence_valid, plane_id)
NormalLookup = Callable[
    [float, float], Optional[tuple[float, float, float, float, bool, str]]
]


@dataclass(frozen=True)
class BuiltGeometryStep:
    step_id: int
    plane_id: str
    normal_x: float
    normal_y: float
    normal_z: float
    confidence: float
    confidence_valid: bool


@dataclass(frozen=True)
class GeometryBuildResult:
    """Result of building a GeometryArray subset from a trajectory."""

    steps: tuple[BuiltGeometryStep, ...]
    requested_steps: int
    missing_step_ids: tuple[int, ...]
    used_flat_fallback_count: int

    @property
    def valid_geometry_steps(self) -> int:
        return len(self.steps)

    @property
    def missing_geometry_steps(self) -> int:
        return len(self.missing_step_ids)

    @property
    def coverage_ratio(self) -> float:
        if self.requested_steps <= 0:
            return 0.0
        return self.valid_geometry_steps / float(self.requested_steps)

    @property
    def should_publish(self) -> bool:
        """Publish only when at least one valid GeometryStep exists."""
        return self.valid_geometry_steps > 0


def build_geometry_steps(
    trajectory_steps: Iterable[TrajectoryStepLike],
    lookup_normal: NormalLookup,
    *,
    allow_flat_fallback: bool = False,
    flat_fallback_confidence: float = 0.25,
) -> GeometryBuildResult:
    """Build geometry steps, omitting missing normals when fallback is off.

    Preserves original ``step_id`` values. Does not renumber or reindex.
    """
    steps: list[BuiltGeometryStep] = []
    missing: list[int] = []
    flat_count = 0
    requested = 0

    for step in trajectory_steps:
        requested += 1
        step_id = int(step.step_id)
        normal = lookup_normal(float(step.x), float(step.y))
        if normal is None:
            missing.append(step_id)
            if not allow_flat_fallback:
                continue
            flat_count += 1
            steps.append(
                BuiltGeometryStep(
                    step_id=step_id,
                    plane_id=f"flat-fallback-{step_id}",
                    normal_x=0.0,
                    normal_y=0.0,
                    normal_z=1.0,
                    confidence=float(flat_fallback_confidence),
                    confidence_valid=True,
                )
            )
            continue

        nx, ny, nz, conf, conf_valid, plane_id = normal
        steps.append(
            BuiltGeometryStep(
                step_id=step_id,
                plane_id=str(plane_id),
                normal_x=float(nx),
                normal_y=float(ny),
                normal_z=float(nz),
                confidence=float(conf),
                confidence_valid=bool(conf_valid),
            )
        )

    return GeometryBuildResult(
        steps=tuple(steps),
        requested_steps=requested,
        missing_step_ids=tuple(missing),
        used_flat_fallback_count=flat_count,
    )
