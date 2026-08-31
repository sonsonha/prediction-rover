#!/usr/bin/env python3
"""Condition-driven gate monitor: readiness waits + short collection + bag trigger."""

from __future__ import annotations

import argparse
import json
import signal
import subprocess
import sys
import time
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from safety_perception_msgs.msg import (
    GeometryArray,
    PredictionOutput,
    RoverState,
    TrackedObjectArray,
    Trajectory,
)


@dataclass
class GateState:
    saw_trajectory: bool = False
    saw_geometry: bool = False
    saw_rover_state: bool = False
    saw_rover_accel: bool = False
    saw_nonempty_tracked: bool = False
    first_traj_sim_t: float | None = None
    first_geom_sim_t: float | None = None
    first_state_sim_t: float | None = None
    first_nonempty_tracked_sim_t: float | None = None
    trajectory_ids: list[int] = field(default_factory=list)
    prediction_source_ids: list[int] = field(default_factory=list)
    predictions: list[PredictionOutput] = field(default_factory=list)
    nonempty_tracked_count: int = 0
    empty_tracked_count: int = 0
    predict_count: int = 0
    collection_nonempty_count: int = 0
    collection_empty_count: int = 0
    collection_predict_count: int = 0
    tracked_samples: list[dict[str, Any]] = field(default_factory=list)
    bag_started: bool = False
    bag_proc: subprocess.Popen | None = None
    collection_started: bool = False
    collection_start_wall: float | None = None


class ConditionGateMonitor(Node):
    def __init__(
        self,
        state: GateState,
        *,
        target_nonempty: int,
        target_predict: int,
        collection_limit_s: float,
        min_collection_s: float = 15.0,
    ) -> None:
        super().__init__("condition_gate_monitor")
        self.state = state
        self.target_nonempty = target_nonempty
        self.target_predict = target_predict
        self.collection_limit_s = collection_limit_s
        self.min_collection_s = min_collection_s
        qos = 10
        self.create_subscription(Trajectory, "/trajectory", self._on_traj, qos)
        self.create_subscription(GeometryArray, "/geometry", self._on_geom, qos)
        self.create_subscription(RoverState, "/rover/state", self._on_state, qos)
        self.create_subscription(
            TrackedObjectArray, "/tracked_objects", self._on_tracked, qos
        )
        self.create_subscription(
            PredictionOutput, "/predict_output", self._on_predict, qos
        )
        self.create_subscription(Clock, "/clock", self._on_clock, qos)

    @staticmethod
    def _sim_t(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _on_clock(self, _msg: Clock) -> None:
        pass

    def _on_traj(self, msg: Trajectory) -> None:
        st = self.state
        if not st.saw_trajectory:
            st.saw_trajectory = True
            st.first_traj_sim_t = self._sim_t(msg.header.stamp)
        st.trajectory_ids.append(int(msg.trajectory_id))

    def _on_geom(self, msg: GeometryArray) -> None:
        st = self.state
        if not st.saw_geometry and msg.steps:
            st.saw_geometry = True
            st.first_geom_sim_t = self._sim_t(msg.header.stamp)

    def _on_state(self, msg: RoverState) -> None:
        st = self.state
        if not st.saw_rover_state:
            st.saw_rover_state = True
            st.first_state_sim_t = self._sim_t(msg.header.stamp)
        if msg.acceleration_valid and not st.saw_rover_accel:
            st.saw_rover_accel = True

    def _on_tracked(self, msg: TrackedObjectArray) -> None:
        st = self.state
        if msg.objects:
            st.nonempty_tracked_count += 1
            if st.collection_started:
                st.collection_nonempty_count += 1
            if not st.saw_nonempty_tracked:
                st.saw_nonempty_tracked = True
                st.first_nonempty_tracked_sim_t = self._sim_t(msg.header.stamp)
            if len(st.tracked_samples) < 5:
                obj = msg.objects[0]
                st.tracked_samples.append(
                    {
                        "sim_t": self._sim_t(msg.header.stamp),
                        "frame_id": msg.header.frame_id,
                        "track_id": int(obj.track_id),
                        "class_name": obj.class_name,
                        "confidence": float(obj.confidence),
                        "confidence_valid": bool(obj.confidence_valid),
                        "velocity_valid": bool(obj.velocity_valid),
                        "footprint_polygon_xy": [
                            {"x": float(p.x), "y": float(p.y)}
                            for p in obj.footprint_polygon_xy
                        ],
                    }
                )
        else:
            st.empty_tracked_count += 1
            if st.collection_started:
                st.collection_empty_count += 1

    def _on_predict(self, msg: PredictionOutput) -> None:
        st = self.state
        st.predict_count += 1
        if st.collection_started:
            st.collection_predict_count += 1
        st.prediction_source_ids.append(int(msg.source_trajectory_id))
        if len(st.predictions) < 25:
            st.predictions.append(msg)

    def ready_for_collection(self) -> bool:
        st = self.state
        return (
            st.saw_trajectory
            and st.saw_geometry
            and st.saw_rover_state
            and st.saw_nonempty_tracked
        )

    def collection_done(self) -> bool:
        st = self.state
        if not st.collection_started or st.collection_start_wall is None:
            return False
        elapsed = time.time() - st.collection_start_wall
        if elapsed < self.min_collection_s:
            return False
        if st.collection_nonempty_count >= self.target_nonempty:
            return True
        if st.collection_predict_count >= self.target_predict:
            return True
        if elapsed >= self.collection_limit_s:
            return True
        return False


def stop_bag_record(bag_proc: subprocess.Popen, bag_dir: Path, timeout_s: float = 12.0) -> bool:
    bag_proc.send_signal(signal.SIGINT)
    try:
        bag_proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        bag_proc.kill()
        bag_proc.wait(timeout=3)
    deadline = time.time() + 3.0
    while time.time() < deadline:
        if (bag_dir / "metadata.yaml").exists():
            return True
        time.sleep(0.2)
    return bag_dir.exists() and any(bag_dir.iterdir())


def start_bag_record(bag_dir: Path) -> subprocess.Popen:
    bag_dir.parent.mkdir(parents=True, exist_ok=True)
    if bag_dir.exists():
        import shutil

        shutil.rmtree(bag_dir)
    topics = [
        "/trajectory",
        "/tracked_objects",
        "/geometry",
        "/rover/state",
    ]
    cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        str(bag_dir),
        "--include-hidden-topics",
        *topics,
    ]
    return subprocess.Popen(cmd)


def duplicate_ids(values: list[int]) -> list[int]:
    counts = Counter(values)
    return sorted(v for v, c in counts.items() if c > 1)


def run_upstream_phase(args: argparse.Namespace) -> dict[str, Any]:
    gate_start = time.time()
    deadline = gate_start + args.gate_limit_s
    state = GateState()
    rclpy.init()
    node = ConditionGateMonitor(
        state,
        target_nonempty=args.target_nonempty,
        target_predict=args.target_predict,
        collection_limit_s=args.collection_limit_s,
        min_collection_s=args.min_collection_s,
    )
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

    report: dict[str, Any] = {
        "phase": "upstream",
        "gate_limit_s": args.gate_limit_s,
        "svo_start_sim_t": args.svo_start_sim_t,
        "pipe_window_sim_t": [args.pipe_start_sim_t, args.pipe_end_sim_t],
    }
    blocker: str | None = None

    try:
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = time.time() - gate_start

            if not state.saw_trajectory and elapsed > args.wait_trajectory_s:
                blocker = "timeout waiting for /trajectory"
                break
            if state.saw_trajectory and not state.saw_geometry and elapsed > args.wait_geometry_s:
                blocker = "timeout waiting for /geometry"
                break
            if state.saw_trajectory and not state.saw_rover_state and elapsed > args.wait_geometry_s:
                blocker = "timeout waiting for /rover/state"
                break
            if (
                state.saw_trajectory
                and state.saw_geometry
                and state.saw_rover_state
                and not state.saw_nonempty_tracked
                and elapsed > args.wait_nonempty_s
            ):
                blocker = "timeout waiting for non-empty /tracked_objects"
                break

            if node.ready_for_collection() and not state.collection_started:
                state.collection_started = True
                state.collection_start_wall = time.time()
                report["readiness_wall_s"] = round(elapsed, 2)
                report["readiness_sim_t"] = {
                    "trajectory": state.first_traj_sim_t,
                    "geometry": state.first_geom_sim_t,
                    "rover_state": state.first_state_sim_t,
                    "nonempty_tracked": state.first_nonempty_tracked_sim_t,
                }
                if not state.bag_started:
                    state.bag_proc = start_bag_record(args.bag_dir)
                    state.bag_started = True
                    report["bag_record_started"] = True

            if state.collection_started and node.collection_done():
                break
    finally:
        bag_created = False
        if state.bag_proc is not None:
            bag_created = stop_bag_record(state.bag_proc, args.bag_dir)
        node.destroy_node()
        rclpy.shutdown()

    report["elapsed_wall_s"] = round(time.time() - gate_start, 2)
    report["readiness"] = {
        "trajectory": state.saw_trajectory,
        "geometry": state.saw_geometry,
        "rover_state": state.saw_rover_state,
        "rover_acceleration": state.saw_rover_accel,
        "nonempty_tracked": state.saw_nonempty_tracked,
    }
    report["collection"] = {
        "nonempty_tracked_messages": state.collection_nonempty_count,
        "empty_tracked_messages": state.collection_empty_count,
        "predict_output_messages": state.collection_predict_count,
        "tracked_samples": state.tracked_samples,
        "lifetime_nonempty_tracked_messages": state.nonempty_tracked_count,
    }
    report["once_per_trajectory"] = {
        "trajectory_message_count": len(state.trajectory_ids),
        "unique_trajectory_ids": sorted(set(state.trajectory_ids))[:30],
        "prediction_message_count": len(state.prediction_source_ids),
        "unique_prediction_source_ids": sorted(set(state.prediction_source_ids))[:30],
        "duplicate_prediction_source_ids": duplicate_ids(state.prediction_source_ids),
    }
    report["bag_created"] = bag_created if state.bag_proc is not None else (
        args.bag_dir.exists() and (args.bag_dir / "metadata.yaml").exists()
    )
    report["geometry_at_first_tracked"] = (
        state.first_geom_sim_t is not None
        and state.first_nonempty_tracked_sim_t is not None
        and state.first_geom_sim_t <= state.first_nonempty_tracked_sim_t
    )
    report["rover_state_at_first_tracked"] = (
        state.first_state_sim_t is not None
        and state.first_nonempty_tracked_sim_t is not None
        and state.first_state_sim_t <= state.first_nonempty_tracked_sim_t
    )
    report.update(_prediction_metrics(state))
    report["blocker"] = blocker
    if blocker is None and not report["bag_created"]:
        report["blocker"] = "canonical bag not created (bag record too short or failed to finalize)"
    report["success"] = blocker is None and state.saw_nonempty_tracked and report["bag_created"]
    return report


def _prediction_metrics(state: GateState) -> dict[str, Any]:
    collision_steps_total = 0
    rollover_steps_total = 0
    pred_entries: list[dict[str, Any]] = []
    for pred in state.predictions:
        collision_steps_total += len(pred.collision_steps)
        rollover_steps_total += len(pred.rollover_steps)
        entry: dict[str, Any] = {
            "source_trajectory_id": int(pred.source_trajectory_id),
            "collision_steps": len(pred.collision_steps),
            "rollover_steps": len(pred.rollover_steps),
        }
        if pred.rollover_steps:
            rs = pred.rollover_steps[0]
            entry["rollover_valid"] = {
                "stability_moment_valid": bool(rs.stability_moment.valid),
                "zmp_valid": bool(rs.zmp.valid),
            }
        pred_entries.append(entry)
    return {
        "prediction": {
            "samples": pred_entries[:10],
            "collision_steps_total": collision_steps_total,
            "rollover_steps_total": rollover_steps_total,
        },
        "stability_valid_seen": any(
            p.rollover_steps and p.rollover_steps[0].stability_moment.valid
            for p in state.predictions
        ),
        "zmp_valid_seen": any(
            p.rollover_steps and p.rollover_steps[0].zmp.valid for p in state.predictions
        ),
    }


def run_bag_prediction_phase(
    args: argparse.Namespace,
    profile: str,
    duration_s: float,
) -> dict[str, Any]:
    if not args.bag_dir.exists():
        return {"profile": profile, "success": False, "blocker": "canonical bag missing"}

    play = subprocess.Popen(
        [
            "ros2",
            "bag",
            "play",
            str(args.bag_dir),
            "--clock",
            "--rate",
            "1.0",
        ]
    )
    rclpy.init()
    state = GateState()
    node = ConditionGateMonitor(
        state,
        target_nonempty=1,
        target_predict=args.target_predict,
        collection_limit_s=duration_s,
    )
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    start = time.time()
    try:
        while time.time() - start < duration_s and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if play.poll() is not None and state.predict_count >= args.target_predict:
                break
    finally:
        play.send_signal(signal.SIGINT)
        try:
            play.wait(timeout=5)
        except subprocess.TimeoutExpired:
            play.kill()
        node.destroy_node()
        rclpy.shutdown()

    metrics = _prediction_metrics(state)
    success = state.predict_count >= args.target_predict
    if profile == "dynamic":
        success = success and state.saw_rover_accel
    return {
        "profile": profile,
        "success": success,
        "rover_acceleration_seen": state.saw_rover_accel,
        "predict_output_messages": state.predict_count,
        "nonempty_tracked_seen": state.nonempty_tracked_count,
        "once_per_trajectory": {
            "trajectory_message_count": len(state.trajectory_ids),
            "unique_trajectory_ids": sorted(set(state.trajectory_ids))[:30],
            "prediction_message_count": len(state.prediction_source_ids),
            "unique_prediction_source_ids": sorted(set(state.prediction_source_ids))[:30],
            "duplicate_prediction_source_ids": duplicate_ids(state.prediction_source_ids),
        },
        **metrics,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir", type=Path, required=True)
    parser.add_argument("--bag-dir", type=Path, required=True)
    parser.add_argument("--phase", choices=["upstream", "replay-static", "replay-dynamic"], default="upstream")
    parser.add_argument("--gate-limit-s", type=float, default=180.0)
    parser.add_argument("--wait-trajectory-s", type=float, default=60.0)
    parser.add_argument("--wait-geometry-s", type=float, default=120.0)
    parser.add_argument("--wait-nonempty-s", type=float, default=120.0)
    parser.add_argument("--collection-limit-s", type=float, default=45.0)
    parser.add_argument("--min-collection-s", type=float, default=15.0)
    parser.add_argument("--target-nonempty", type=int, default=20)
    parser.add_argument("--target-predict", type=int, default=10)
    parser.add_argument("--svo-start-sim-t", type=float, default=1783700660.428)
    parser.add_argument("--pipe-start-sim-t", type=float, default=1783700698.0)
    parser.add_argument("--pipe-end-sim-t", type=float, default=1783700725.0)
    parser.add_argument("--replay-duration-s", type=float, default=35.0)
    args = parser.parse_args()
    args.log_dir.mkdir(parents=True, exist_ok=True)

    if args.phase == "upstream":
        report = run_upstream_phase(args)
    else:
        profile = "static" if args.phase == "replay-static" else "dynamic"
        report = run_bag_prediction_phase(args, profile, args.replay_duration_s)

    out = args.log_dir / f"report_{args.phase}.json"
    out.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0 if report.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
