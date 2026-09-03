#!/usr/bin/env python3
"""Build /geometry from /trajectory + terrain GridMap normal layers."""

from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from safety_perception_msgs.msg import GeometryArray, GeometryStep, Trajectory

from lr_prediction_bridge.geometry_partial import build_geometry_steps


class GeometryAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("geometry_adapter_node")
        self.declare_parameter("trajectory_topic", "/trajectory")
        self.declare_parameter("grid_map_topic", "/terrain_geometry/grid_map")
        self.declare_parameter("output_topic", "/geometry")
        self.declare_parameter("expected_frame_id", "map")
        self.declare_parameter("force_frame_id_map", True)
        self.declare_parameter("allow_flat_fallback", False)
        self.declare_parameter("flat_fallback_confidence", 0.25)
        self.declare_parameter("coverage_log_period_s", 2.0)

        self._expected_frame = str(self.get_parameter("expected_frame_id").value)
        self._force_map = bool(self.get_parameter("force_frame_id_map").value)
        self._allow_flat = bool(self.get_parameter("allow_flat_fallback").value)
        self._flat_conf = float(self.get_parameter("flat_fallback_confidence").value)
        self._coverage_log_period_s = float(
            self.get_parameter("coverage_log_period_s").value
        )
        self._last_coverage_log_wall = 0.0

        self._grid: Optional[GridMap] = None
        self._layers: dict[str, np.ndarray] = {}

        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        grid_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(
            GeometryArray, self.get_parameter("output_topic").value, reliable
        )
        self.create_subscription(
            GridMap,
            self.get_parameter("grid_map_topic").value,
            self._on_grid,
            grid_qos,
        )
        self.create_subscription(
            Trajectory,
            self.get_parameter("trajectory_topic").value,
            self._on_trajectory,
            reliable,
        )
        self.get_logger().info(
            "geometry adapter: /trajectory + GridMap normals → /geometry "
            f"(allow_flat_fallback={self._allow_flat}, partial_coverage=True)"
        )

    def _on_grid(self, msg: GridMap) -> None:
        self._grid = msg
        self._layers = {}
        if not msg.data or len(msg.data[0].layout.dim) < 2:
            return
        columns = int(msg.data[0].layout.dim[0].size)
        rows = int(msg.data[0].layout.dim[1].size)
        for name, data in zip(msg.layers, msg.data):
            arr = np.asarray(data.data, dtype=np.float32)
            if arr.size != rows * columns:
                continue
            self._layers[name] = arr.reshape((rows, columns), order="F")

    def _on_trajectory(self, traj: Trajectory) -> None:
        if not traj.steps:
            self.get_logger().warn("ignoring empty trajectory")
            return

        frame_id = traj.header.frame_id.strip() or self._expected_frame
        if self._force_map:
            frame_id = "map"

        built = build_geometry_steps(
            traj.steps,
            self._lookup_normal,
            allow_flat_fallback=self._allow_flat,
            flat_fallback_confidence=self._flat_conf,
        )
        self._log_coverage(
            trajectory_id=int(traj.trajectory_id),
            requested_steps=built.requested_steps,
            valid_geometry_steps=built.valid_geometry_steps,
            missing_geometry_steps=built.missing_geometry_steps,
            coverage_ratio=built.coverage_ratio,
        )

        if not built.should_publish:
            if built.missing_geometry_steps:
                self.get_logger().warn(
                    "no valid terrain normals for any trajectory step; "
                    "skipping GeometryArray publish "
                    f"(trajectory_id={int(traj.trajectory_id)}, "
                    f"requested={built.requested_steps})"
                )
            return

        if built.missing_step_ids and not self._allow_flat:
            # Debug detail for omitted steps; coverage summary is throttled above.
            sample = list(built.missing_step_ids[:5])
            self.get_logger().debug(
                f"omitting {built.missing_geometry_steps} steps without terrain "
                f"normals (sample step_ids={sample})"
            )

        out = GeometryArray()
        out.header.stamp = traj.header.stamp
        out.header.frame_id = frame_id
        out.source_trajectory_id = int(traj.trajectory_id)
        out.source_trajectory_stamp = traj.header.stamp
        steps: list[GeometryStep] = []
        for item in built.steps:
            geom = GeometryStep()
            geom.step_id = int(item.step_id)
            geom.plane_id = item.plane_id
            geom.normal.x = item.normal_x
            geom.normal.y = item.normal_y
            geom.normal.z = item.normal_z
            geom.confidence = item.confidence
            geom.confidence_valid = item.confidence_valid
            steps.append(geom)
        out.steps = steps

        if built.used_flat_fallback_count:
            self.get_logger().warn(
                f"geometry used flat fallback for "
                f"{built.used_flat_fallback_count}/{built.requested_steps} steps"
            )
        self._pub.publish(out)

    def _log_coverage(
        self,
        *,
        trajectory_id: int,
        requested_steps: int,
        valid_geometry_steps: int,
        missing_geometry_steps: int,
        coverage_ratio: float,
    ) -> None:
        now = time.monotonic()
        if (now - self._last_coverage_log_wall) < self._coverage_log_period_s:
            return
        self._last_coverage_log_wall = now
        self.get_logger().info(
            "geometry coverage "
            f"trajectory_id={trajectory_id} "
            f"requested_steps={requested_steps} "
            f"valid_geometry_steps={valid_geometry_steps} "
            f"missing_geometry_steps={missing_geometry_steps} "
            f"coverage_ratio={coverage_ratio:.3f}"
        )

    def _lookup_normal(
        self, x: float, y: float
    ) -> Optional[tuple[float, float, float, float, bool, str]]:
        if self._grid is None:
            return None
        for layer in ("normal_x", "normal_y", "normal_z"):
            if layer not in self._layers:
                return None

        info = self._grid.info
        resolution = float(info.resolution)
        if resolution <= 0.0:
            return None
        rows, cols = self._layers["normal_x"].shape
        # Inverse of terrain_result_to_grid_map indexing.
        center_key_x = float(info.pose.position.x) / resolution - 0.5
        center_key_y = float(info.pose.position.y) / resolution - 0.5
        half_cells = (rows - 1) / 2.0
        max_key_x = center_key_x + half_cells
        max_key_y = center_key_y + half_cells
        key_x = math.floor(x / resolution)
        key_y = math.floor(y / resolution)
        row = int(round(max_key_x - key_x))
        col = int(round(max_key_y - key_y))
        if not (0 <= row < rows and 0 <= col < cols):
            return None

        nx = float(self._layers["normal_x"][row, col])
        ny = float(self._layers["normal_y"][row, col])
        nz = float(self._layers["normal_z"][row, col])
        if not all(math.isfinite(v) for v in (nx, ny, nz)):
            return None
        norm = math.sqrt(nx * nx + ny * ny + nz * nz)
        if norm <= 1e-12:
            return None
        nx, ny, nz = nx / norm, ny / norm, nz / norm

        conf = 1.0
        conf_valid = True
        if "inlier_ratio" in self._layers:
            ratio = float(self._layers["inlier_ratio"][row, col])
            if math.isfinite(ratio):
                conf = max(0.0, min(1.0, ratio))
        if "state" in self._layers:
            state = float(self._layers["state"][row, col])
            if math.isfinite(state) and state < 0.5:
                conf_valid = False

        return nx, ny, nz, conf, conf_valid, f"terrain-r{row}-c{col}"


def main(argv: list[str] | None = None) -> None:
    import rclpy

    rclpy.init(args=argv)
    node = GeometryAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
