"""ROS 2 Prediction visualization node (visualization only)."""

from __future__ import annotations

from typing import Any

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

from safety_perception_msgs.msg import (
    DecisionEvidence,
    DecisionOutput,
    GeometryArray,
    PredictionOutput,
    RoverState,
    TrackedObjectArray,
    Trajectory,
)

from prediction_visualization.marker_builders import (
    build_collision_markers,
    build_object_markers,
    build_rollover_markers,
    build_rover_markers,
    build_status_markers,
    build_terrain_normal_markers,
    build_trajectory_path,
    build_trajectory_step_markers,
    build_zmp_markers,
    clear_namespaces,
    trajectory_steps_by_id,
)


class PredictionVisualizationNode(Node):
    def __init__(self) -> None:
        super().__init__("prediction_visualization")
        self.declare_parameter("fixed_frame", "map")
        self.declare_parameter("terrain_normal_stride", 2)
        self.declare_parameter("normal_arrow_scale", 0.45)
        self.declare_parameter("object_label_enabled", True)
        self.declare_parameter("status_text_enabled", True)
        self.declare_parameter("trajectory_step_markers_enabled", False)
        self.declare_parameter("body_length_m", 1.05)
        self.declare_parameter("body_width_m", 0.90)
        self.declare_parameter("body_height_m", 0.50)
        self.declare_parameter("heading_arrow_length_m", 0.6)
        self.declare_parameter("z_lift_m", 0.05)

        self._traj: Trajectory | None = None
        self._objects: TrackedObjectArray | None = None
        self._geometry: GeometryArray | None = None
        self._rover: RoverState | None = None
        self._predict: PredictionOutput | None = None
        self._decision_evidence: DecisionEvidence | None = None
        self._decision_output: DecisionOutput | None = None

        qos = 10
        self.create_subscription(Trajectory, "/trajectory", self._on_traj, qos)
        self.create_subscription(
            TrackedObjectArray, "/tracked_objects", self._on_objects, qos
        )
        self.create_subscription(GeometryArray, "/geometry", self._on_geometry, qos)
        self.create_subscription(RoverState, "/rover/state", self._on_rover, qos)
        self.create_subscription(
            PredictionOutput, "/predict_output", self._on_predict, qos
        )
        self.create_subscription(
            DecisionEvidence, "/decision/evidence", self._on_decision_evidence, qos
        )
        self.create_subscription(
            DecisionOutput, "/decision", self._on_decision_output, qos
        )

        self._pub_path = self.create_publisher(Path, "/prediction_viz/trajectory", qos)
        self._pub_objects = self.create_publisher(
            MarkerArray, "/prediction_viz/objects", qos
        )
        self._pub_terrain = self.create_publisher(
            MarkerArray, "/prediction_viz/terrain", qos
        )
        self._pub_rover = self.create_publisher(MarkerArray, "/prediction_viz/rover", qos)
        self._pub_collision = self.create_publisher(
            MarkerArray, "/prediction_viz/collision", qos
        )
        self._pub_rollover = self.create_publisher(
            MarkerArray, "/prediction_viz/rollover", qos
        )
        self._pub_zmp = self.create_publisher(MarkerArray, "/prediction_viz/zmp", qos)
        self._pub_status = self.create_publisher(
            MarkerArray, "/prediction_viz/status", qos
        )

        self.get_logger().info("prediction_visualization ready (viz only)")

    def _frame(self) -> str:
        return str(self.get_parameter("fixed_frame").value)

    def _param(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _matching_prediction(self) -> PredictionOutput | None:
        if self._traj is None or self._predict is None:
            return None
        if int(self._predict.source_trajectory_id) != int(self._traj.trajectory_id):
            return None
        return self._predict

    def _clear_prediction_pubs(self, stamp: Any | None = None) -> None:
        frame = self._frame()
        self._pub_collision.publish(
            clear_namespaces(frame, ("collision", "collision_text"), stamp)
        )
        self._pub_rollover.publish(
            clear_namespaces(frame, ("rollover", "rollover_text"), stamp)
        )
        self._pub_zmp.publish(clear_namespaces(frame, ("zmp",), stamp))

    def _publish_trajectory_layer(self) -> None:
        assert self._traj is not None
        frame = self._frame()
        stamp = self._traj.header.stamp
        self._pub_path.publish(build_trajectory_path(self._traj, frame, stamp))
        if bool(self._param("trajectory_step_markers_enabled")):
            self._pub_status.publish(
                build_trajectory_step_markers(
                    self._traj,
                    frame,
                    enabled=True,
                    z_lift_m=float(self._param("z_lift_m")),
                    stamp=stamp,
                )
            )

    def _publish_objects_layer(self) -> None:
        if self._objects is None:
            return
        self._pub_objects.publish(
            build_object_markers(
                self._objects,
                self._frame(),
                label_enabled=bool(self._param("object_label_enabled")),
                z_lift_m=float(self._param("z_lift_m")),
            )
        )

    def _publish_terrain_layer(self) -> None:
        if self._geometry is None or self._traj is None:
            return
        if int(self._geometry.source_trajectory_id) != int(self._traj.trajectory_id):
            return
        by_id = trajectory_steps_by_id(self._traj.steps)
        self._pub_terrain.publish(
            build_terrain_normal_markers(
                self._geometry,
                by_id,
                self._frame(),
                stride=int(self._param("terrain_normal_stride")),
                arrow_scale=float(self._param("normal_arrow_scale")),
            )
        )

    def _publish_rover_layer(self) -> None:
        if self._rover is None:
            return
        self._pub_rover.publish(
            build_rover_markers(
                self._rover,
                self._frame(),
                body_length_m=float(self._param("body_length_m")),
                body_width_m=float(self._param("body_width_m")),
                body_height_m=float(self._param("body_height_m")),
                heading_arrow_length_m=float(self._param("heading_arrow_length_m")),
                z_lift_m=float(self._param("z_lift_m")),
            )
        )

    def _publish_status_layer(self, stamp: Any | None = None) -> None:
        if self._traj is None:
            return
        pred = self._matching_prediction()
        if stamp is None:
            stamp = (
                pred.header.stamp
                if pred is not None
                else self._traj.header.stamp
            )
        anchor = None
        if self._traj.steps:
            s0 = self._traj.steps[0]
            anchor = (float(s0.x), float(s0.y))
        self._pub_status.publish(
            build_status_markers(
                pred,
                int(self._traj.trajectory_id),
                self._frame(),
                enabled=bool(self._param("status_text_enabled")),
                anchor_xy=anchor,
                stamp=stamp,
                decision_evidence=self._decision_evidence,
                decision_output=self._decision_output,
            )
        )

    def _publish_prediction_layers(self) -> None:
        if self._traj is None:
            return
        pred = self._matching_prediction()
        stamp = pred.header.stamp if pred is not None else self._traj.header.stamp
        if pred is None:
            self._pub_collision.publish(
                clear_namespaces(self._frame(), ("collision", "collision_text"), stamp)
            )
            self._pub_rollover.publish(
                clear_namespaces(self._frame(), ("rollover", "rollover_text"), stamp)
            )
            self._pub_zmp.publish(clear_namespaces(self._frame(), ("zmp",), stamp))
            self._publish_status_layer(stamp)
            return
        by_id = trajectory_steps_by_id(self._traj.steps)
        frame = self._frame()
        z = float(self._param("z_lift_m"))
        self._pub_collision.publish(
            build_collision_markers(pred, by_id, frame, z_lift_m=z + 0.08, stamp=stamp)
        )
        self._pub_rollover.publish(
            build_rollover_markers(pred, by_id, frame, z_lift_m=z + 0.05, stamp=stamp)
        )
        self._pub_zmp.publish(
            build_zmp_markers(pred, by_id, frame, z_lift_m=z + 0.04, stamp=stamp)
        )
        self._publish_status_layer(stamp)

    def _on_traj(self, msg: Trajectory) -> None:
        self._traj = msg
        new_id = int(msg.trajectory_id)
        if self._predict is not None and int(self._predict.source_trajectory_id) != new_id:
            self._predict = None
            self._clear_prediction_pubs(msg.header.stamp)
        self._publish_trajectory_layer()
        self._publish_terrain_layer()
        self._publish_prediction_layers()

    def _on_objects(self, msg: TrackedObjectArray) -> None:
        self._objects = msg
        self._publish_objects_layer()

    def _on_geometry(self, msg: GeometryArray) -> None:
        self._geometry = msg
        self._publish_terrain_layer()

    def _on_rover(self, msg: RoverState) -> None:
        self._rover = msg
        self._publish_rover_layer()

    def _on_predict(self, msg: PredictionOutput) -> None:
        if self._traj is not None and int(msg.source_trajectory_id) != int(
            self._traj.trajectory_id
        ):
            # Do not attach PredictionOutput from another trajectory to the active one.
            self.get_logger().debug(
                f"skip predict source={msg.source_trajectory_id} "
                f"active_traj={self._traj.trajectory_id}"
            )
            return
        self._predict = msg
        self._publish_prediction_layers()

    def _on_decision_evidence(self, msg: DecisionEvidence) -> None:
        self._decision_evidence = msg
        self._publish_status_layer(msg.header.stamp)

    def _on_decision_output(self, msg: DecisionOutput) -> None:
        self._decision_output = msg
        self._publish_status_layer(msg.header.stamp)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PredictionVisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
