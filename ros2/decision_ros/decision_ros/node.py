"""ROS 2 Decision evidence node (V0: evidence state only)."""

from __future__ import annotations

from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from safety_perception_msgs.msg import DecisionEvidence, PredictionOutput, Trajectory

from decision_ros.evidence import build_decision_evidence


def _apply_fields(msg: DecisionEvidence, fields: Any) -> None:
    msg.evidence_state = int(fields.evidence_state)
    msg.source_trajectory_id = str(fields.source_trajectory_id)
    msg.collision_candidates_present = bool(fields.collision_candidates_present)
    msg.rollover_baseline_present = bool(fields.rollover_baseline_present)
    msg.dynamic_stability_moment_valid = bool(fields.dynamic_stability_moment_valid)
    msg.zmp_valid = bool(fields.zmp_valid)
    msg.nearest_collision_distance_valid = bool(fields.nearest_collision_distance_valid)
    msg.nearest_collision_distance_m = float(fields.nearest_collision_distance_m)
    msg.minimum_normalized_ssm_valid = bool(fields.minimum_normalized_ssm_valid)
    msg.minimum_normalized_static_stability_margin = float(
        fields.minimum_normalized_static_stability_margin
    )
    msg.minimum_stability_moment_valid = bool(fields.minimum_stability_moment_valid)
    msg.minimum_stability_moment_nm = float(fields.minimum_stability_moment_nm)
    msg.minimum_zmp_margin_valid = bool(fields.minimum_zmp_margin_valid)
    msg.minimum_zmp_margin_m = float(fields.minimum_zmp_margin_m)


class DecisionEvidenceNode(Node):
    def __init__(self) -> None:
        super().__init__("decision_evidence")
        self._active_trajectory_id: int | None = None
        self._latest_prediction: PredictionOutput | None = None

        qos = 10
        self.create_subscription(Trajectory, "/trajectory", self._on_trajectory, qos)
        self.create_subscription(
            PredictionOutput, "/predict_output", self._on_predict_output, qos
        )
        self._pub = self.create_publisher(DecisionEvidence, "/decision/evidence", qos)

    def _publish(self, header: Header | None = None) -> None:
        fields = build_decision_evidence(
            self._active_trajectory_id, self._latest_prediction
        )
        msg = DecisionEvidence()
        if header is not None:
            msg.header = header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = ""
        _apply_fields(msg, fields)
        self._pub.publish(msg)

    def _on_trajectory(self, msg: Trajectory) -> None:
        self._active_trajectory_id = int(msg.trajectory_id)
        self._publish(msg.header)

    def _on_predict_output(self, msg: PredictionOutput) -> None:
        self._latest_prediction = msg
        self._publish(msg.header)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DecisionEvidenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
