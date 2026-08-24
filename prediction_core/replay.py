"""Canonical Pure-Python Prediction V1 event-stream CLI (no ROS).

Run::

    python -m prediction_core.replay \\
      --config config/rover.mock.yaml \\
      --scenario mock/runtime_scenarios/flat_static.json
"""

from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path
from typing import Any, Sequence

from .cache import ExternalWrenchData
from .config import RoverConfig, load_config
from .events import (
    ExternalWrenchEvent,
    GeometryEvent,
    ObjectsEvent,
    StateEvent,
    TrajectoryEvent,
)
from .models import (
    GeometryStep,
    PredictionOutput,
    RoverState,
    TrackedObject,
    Trajectory,
    TrajectoryStep,
)
from .runtime import PredictionRuntime, RuntimeResult
from .serialization import to_jsonable, write_json
from .validation import PredictionProfile
from .version import PREDICTION_PYTHON_LABEL, PREDICTION_PYTHON_VERSION


LOGGER = logging.getLogger(__name__)


def _point2(value: Sequence[float]) -> tuple[float, float]:
    return float(value[0]), float(value[1])


def _vector3(value: Sequence[float]) -> tuple[float, float, float]:
    return float(value[0]), float(value[1]), float(value[2])


def parse_trajectory_event(raw: dict[str, Any]) -> TrajectoryEvent:
    timestamp = float(raw.get("timestamp", 0.0))
    frame_id = str(raw.get("frame_id", "map"))
    steps = [
        TrajectoryStep(
            step_id=int(step["step_id"]),
            x=float(step["x"]),
            y=float(step["y"]),
            yaw=float(step["yaw"]),
        )
        for step in raw["steps"]
    ]
    return TrajectoryEvent(
        trajectory_id=int(raw["trajectory_id"]),
        trajectory=Trajectory(timestamp=timestamp, frame_id=frame_id, steps=steps),
    )


def parse_objects_event(raw: dict[str, Any]) -> ObjectsEvent:
    if "objects" not in raw:
        raise ValueError("objects event requires an objects field (use [] for empty)")
    timestamp = float(raw.get("timestamp", 0.0))
    frame_id = str(raw.get("frame_id", "map"))
    objects = [
        TrackedObject(
            timestamp=float(obj.get("timestamp", timestamp)),
            track_id=obj["track_id"],
            class_name=str(obj["class_name"]),
            footprint_polygon_xy=[_point2(point) for point in obj["footprint_polygon_xy"]],
            height_m=None if obj.get("height_m") is None else float(obj["height_m"]),
            velocity_xy=(
                None if obj.get("velocity_xy") is None else _point2(obj["velocity_xy"])
            ),
            confidence=None if obj.get("confidence") is None else float(obj["confidence"]),
        )
        for obj in raw["objects"]
    ]
    return ObjectsEvent(objects=objects, frame_id=frame_id, timestamp=timestamp)


def parse_geometry_event(raw: dict[str, Any]) -> GeometryEvent:
    timestamp = float(raw.get("timestamp", 0.0))
    frame_id = str(raw.get("frame_id", "map"))
    source_trajectory_id = int(raw["source_trajectory_id"])
    source_stamp = raw.get("source_trajectory_stamp", timestamp)
    steps_raw = raw.get("steps", raw.get("geometry", []))
    geometry = [
        GeometryStep(
            timestamp=float(step.get("timestamp", timestamp)),
            step_id=int(step["step_id"]),
            plane_id=step.get("plane_id", step.get("terrain_id", f"plane-{step['step_id']}")),
            normal_xyz=_vector3(step["normal_xyz"]),
            centroid_xyz=(
                None if step.get("centroid_xyz") is None else _vector3(step["centroid_xyz"])
            ),
            confidence=None if step.get("confidence") is None else float(step["confidence"]),
        )
        for step in steps_raw
    ]
    return GeometryEvent(
        geometry=geometry,
        frame_id=frame_id,
        source_trajectory_id=source_trajectory_id,
        source_trajectory_stamp=None if source_stamp is None else float(source_stamp),
    )


def parse_state_event(raw: dict[str, Any]) -> StateEvent:
    timestamp = float(raw.get("timestamp", 0.0))
    frame_id = str(raw.get("frame_id", "map"))
    # Explicit "acceleration_xyz": null means unavailable; omit key also means None.
    has_accel_xyz = "acceleration_xyz" in raw
    accel_xyz_raw = raw.get("acceleration_xyz")
    state = RoverState(
        timestamp=timestamp,
        x=None if raw.get("x") is None else float(raw["x"]),
        y=None if raw.get("y") is None else float(raw["y"]),
        yaw=None if raw.get("yaw") is None else float(raw["yaw"]),
        roll=None if raw.get("roll") is None else float(raw["roll"]),
        pitch=None if raw.get("pitch") is None else float(raw["pitch"]),
        velocity_xy=(
            None if raw.get("velocity_xy") is None else _point2(raw["velocity_xy"])
        ),
        acceleration_xy=(
            None if raw.get("acceleration_xy") is None else _point2(raw["acceleration_xy"])
        ),
        angular_velocity_xyz=(
            None
            if raw.get("angular_velocity_xyz") is None
            else _vector3(raw["angular_velocity_xyz"])
        ),
        velocity_xyz=(
            None if raw.get("velocity_xyz") is None else _vector3(raw["velocity_xyz"])
        ),
        acceleration_xyz=(
            None
            if (not has_accel_xyz or accel_xyz_raw is None)
            else _vector3(accel_xyz_raw)
        ),
        angular_acceleration_xyz=(
            None
            if raw.get("angular_acceleration_xyz") is None
            else _vector3(raw["angular_acceleration_xyz"])
        ),
    )
    return StateEvent(state=state, frame_id=frame_id)


def parse_external_wrench_event(raw: dict[str, Any]) -> ExternalWrenchEvent:
    if "wrenches" not in raw:
        raise ValueError("external_wrench event requires wrenches field (use [] for empty)")
    frame_id = str(raw.get("frame_id", "map"))
    wrenches = [
        ExternalWrenchData(
            source=str(item.get("source", "unknown")),
            frame_id=str(item.get("frame_id", frame_id)),
            force_xyz=_vector3(item["force_xyz"]),
            torque_xyz=_vector3(item["torque_xyz"]),
            application_point_xyz=(
                None
                if item.get("application_point_xyz") is None
                else _vector3(item["application_point_xyz"])
            ),
            confidence=None if item.get("confidence") is None else float(item["confidence"]),
        )
        for item in raw["wrenches"]
    ]
    return ExternalWrenchEvent(wrenches=wrenches, frame_id=frame_id)


def parse_event(raw: dict[str, Any]):
    event_type = str(raw.get("type", "")).strip().lower()
    if event_type == "trajectory":
        return parse_trajectory_event(raw)
    if event_type in {"objects", "tracked_objects"}:
        return parse_objects_event(raw)
    if event_type == "geometry":
        return parse_geometry_event(raw)
    if event_type in {"state", "rover_state"}:
        return parse_state_event(raw)
    if event_type in {"external_wrench", "external_wrenches"}:
        return parse_external_wrench_event(raw)
    raise ValueError(f"unsupported event type: {event_type!r}")


def load_event_stream(path: Path) -> list[dict[str, Any]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(payload, dict) and "events" in payload:
        events = payload["events"]
    elif isinstance(payload, list):
        events = payload
    else:
        raise ValueError("event file must be a list or an object with an events array")
    if not isinstance(events, list):
        raise ValueError("events must be a list")
    return events


def replay_events(
    runtime: PredictionRuntime,
    raw_events: Sequence[dict[str, Any]],
    *,
    log_result_messages: bool = True,
) -> list[RuntimeResult]:
    results: list[RuntimeResult] = []
    for index, raw in enumerate(raw_events):
        event = parse_event(raw)
        LOGGER.debug("event[%s] type=%s", index, raw.get("type"))
        result = runtime.handle_event(event)
        if log_result_messages:
            for message in result.messages:
                LOGGER.info("%s", message)
        if result.output is not None:
            LOGGER.info(
                "prediction completed collision_steps=%s rollover_steps=%s",
                len(result.output.collision_steps),
                len(result.output.rollover_steps),
            )
        results.append(result)
    return results


def format_prediction_summary(
    output: PredictionOutput | None,
    *,
    cycle_id: int | None = None,
    frame_id: str = "map",
    profile: PredictionProfile | str = PredictionProfile.STATIC,
    readiness_reason: str | None = None,
) -> str:
    """Concise human-readable V1 evidence summary (no Decision semantics)."""
    profile_value = (
        profile.value if isinstance(profile, PredictionProfile) else str(profile)
    )
    lines: list[str] = [
        f"{PREDICTION_PYTHON_LABEL} (v{PREDICTION_PYTHON_VERSION})",
        f"Prediction profile: {profile_value}",
        f"Prediction cycle: {cycle_id if cycle_id is not None else 'n/a'}",
        f"Frame: {frame_id}",
    ]
    if output is None:
        lines.extend(
            [
                "",
                "Prediction: not produced",
                f"Readiness: {readiness_reason or 'not ready'}",
            ]
        )
        return "\n".join(lines) + "\n"

    lines.extend(
        [
            f"Source trajectory stamp: {output.source_trajectory_stamp}",
            "",
            "Collision:",
            f"  steps with candidates: {len(output.collision_steps)}",
            "",
            "Rollover:",
            f"  evaluated steps: {len(output.rollover_steps)}",
        ]
    )
    if not output.rollover_steps:
        return "\n".join(lines) + "\n"

    # Show first and (if different) last step for multi-step demos.
    indices = [0]
    if len(output.rollover_steps) > 1:
        indices.append(len(output.rollover_steps) - 1)
    for index in indices:
        step = output.rollover_steps[index]
        dyn = step.dynamic_stability
        lines.extend(
            [
                "",
                f"Step {step.step_id}",
                f"  roll:                    {step.predicted_roll_deg:8.2f} deg",
                f"  pitch:                   {step.predicted_pitch_deg:8.2f} deg",
                "",
                f"  static SSM:              {step.static_stability_margin_m:8.4f} m",
                f"  normalized static SSM:   {step.normalized_static_stability_margin:8.4f}",
                f"  nearest static edge:     {step.nearest_static_edge}",
            ]
        )
        if dyn is None:
            lines.append("  dynamic:                 unavailable")
            continue
        if not dyn.valid:
            lines.extend(
                [
                    "  dynamic:                 unavailable/invalid",
                    f"  validity:                {dyn.validity_reason}",
                ]
            )
        else:
            lines.extend(
                [
                    "",
                    "  # primary dynamic",
                    f"  stability moment:        {dyn.minimum_stability_moment_nm:8.2f} Nm",
                    f"  normalized moment:       {dyn.normalized_minimum_stability_moment:8.4f}",
                    f"  limiting moment edge:    {dyn.minimum_normalized_moment_edge}",
                    "",
                    "  # optional diagnostic",
                    f"  ZMP margin:              {dyn.zmp_margin_m:8.4f} m",
                    f"  nearest ZMP edge:        {dyn.nearest_zmp_edge}",
                    "  ZMP role:                diagnostic",
                    "",
                    "  # secondary diagnostic",
                    f"  effective SSM:           {dyn.effective_ssm_m:8.4f} m",
                    f"  nearest effective edge:  {dyn.nearest_effective_edge}",
                ]
            )
        lines.extend(
            [
                "",
                "Dynamic assumptions:",
                f"  acceleration:            {'available' if dyn.acceleration_available else 'unavailable'}",
                f"  external wrench:         "
                f"{'included' if dyn.external_wrench_included else ('unavailable' if not dyn.external_wrench_available else 'empty')}",
            ]
        )
        if step.critical_tip is not None:
            tip = step.critical_tip
            lines.extend(
                [
                    "",
                    "Critical tip (config diagnostic):",
                    f"  minimum tip angle:       {tip.minimum_deg:8.2f} deg ({tip.minimum_tip_angle_edge})",
                ]
            )
    return "\n".join(lines) + "\n"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            f"{PREDICTION_PYTHON_LABEL}: replay a pure-Python prediction event stream "
            "(no ROS)."
        )
    )
    parser.add_argument(
        "events",
        type=Path,
        nargs="?",
        default=None,
        help="JSON event stream path (positional; same as --scenario)",
    )
    parser.add_argument(
        "--scenario",
        type=Path,
        default=None,
        help="JSON event stream path (canonical flag)",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/rover.mock.yaml"),
        help="Rover YAML config",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Optional path to write full prediction JSON",
    )
    parser.add_argument(
        "--expected-frame-id",
        default="map",
        help="Expected frame_id for all inputs",
    )
    parser.add_argument(
        "--profile",
        choices=[PredictionProfile.STATIC.value, PredictionProfile.DYNAMIC.value],
        default=PredictionProfile.STATIC.value,
        help="Runtime readiness profile (default: static)",
    )
    parser.add_argument(
        "--json-only",
        action="store_true",
        help="Print JSON payload only (skip human summary)",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Reduce runtime log chatter",
    )
    parser.add_argument(
        "--version",
        action="store_true",
        help="Print Prediction Python V1 version and exit",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.version:
        print(f"{PREDICTION_PYTHON_LABEL} {PREDICTION_PYTHON_VERSION}")
        return 0

    logging.basicConfig(
        level=logging.WARNING if args.quiet else logging.INFO,
        format="%(levelname)s %(message)s",
    )

    scenario_path = args.scenario or args.events
    if scenario_path is None:
        build_parser().error("provide --scenario PATH or positional events path")

    config: RoverConfig = load_config(args.config)
    profile = PredictionProfile(args.profile)
    runtime = PredictionRuntime(
        config,
        profile=profile,
        expected_frame_id=args.expected_frame_id,
        # Replay logs RuntimeResult.messages; keep runtime logger silent here.
        logger=lambda _message: None,
    )
    results = replay_events(
        runtime,
        load_event_stream(scenario_path),
        log_result_messages=not args.quiet,
    )
    outputs = [result.output for result in results if result.output is not None]
    last_result = results[-1] if results else None
    if not outputs:
        LOGGER.warning("replay finished with no prediction output")
        reason = (
            last_result.readiness.reason
            if last_result is not None
            else "no events processed"
        )
        if not args.json_only:
            print(
                format_prediction_summary(
                    None,
                    profile=profile,
                    frame_id=args.expected_frame_id,
                    readiness_reason=reason,
                ),
                end="",
            )
        print(
            json.dumps(
                {
                    "prediction_python_version": PREDICTION_PYTHON_VERSION,
                    "prediction_profile": profile.value,
                    "predictions": 0,
                    "readiness_reason": reason,
                    "outputs": [],
                },
                indent=2,
            )
        )
        return 0

    cycle_id = None
    for result in results:
        if result.cycle_key is not None:
            cycle_id = result.cycle_key.trajectory_id
            break

    last_output = outputs[-1]
    if not args.json_only:
        print(
            format_prediction_summary(
                last_output,
                cycle_id=cycle_id,
                frame_id=args.expected_frame_id,
                profile=profile,
            ),
            end="",
        )

    payload = {
        "prediction_python_version": PREDICTION_PYTHON_VERSION,
        "prediction_python_label": PREDICTION_PYTHON_LABEL,
        "prediction_profile": profile.value,
        "predictions": len(outputs),
        "outputs": [to_jsonable(output) for output in outputs],
    }
    if args.json_only:
        print(json.dumps(payload, indent=2))
    if args.output is not None:
        write_json(args.output, payload)
        LOGGER.info("wrote %s", args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
