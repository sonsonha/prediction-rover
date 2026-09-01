"""ROS 2 Decision prototype policy node (V1: STOP/GO from DecisionEvidence)."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from safety_perception_msgs.msg import DecisionEvidence, DecisionOutput

from decision_ros.policy import (
    METRIC_STABILITY_MOMENT,
    PolicyConfig,
    PolicyResult,
    RolloverPolicyConfig,
    evaluate_policy,
)


def _load_policy_config(node: Node) -> PolicyConfig:
    prototype_only = bool(node.get_parameter("decision_policy.prototype_only").value)
    stop_on_collision = bool(
        node.get_parameter("decision_policy.stop_on_collision_candidate").value
    )
    stop_on_missing = bool(
        node.get_parameter("decision_policy.stop_on_missing_current_prediction").value
    )
    rollover_enabled = bool(
        node.get_parameter("decision_policy.rollover_policy.enabled").value
    )
    rollover_metric = str(
        node.get_parameter("decision_policy.rollover_policy.metric").value
    )
    threshold_param = node.get_parameter("decision_policy.rollover_policy.threshold")
    threshold: float | None
    if threshold_param.type_ == Parameter.Type.NOT_SET:
        threshold = None
    else:
        raw = float(threshold_param.value)
        threshold = raw if math.isfinite(raw) else None
    return PolicyConfig(
        prototype_only=prototype_only,
        stop_on_collision_candidate=stop_on_collision,
        stop_on_missing_current_prediction=stop_on_missing,
        rollover_policy=RolloverPolicyConfig(
            enabled=rollover_enabled,
            metric=rollover_metric,
            threshold=threshold,
        ),
    )


def _apply_policy(msg: DecisionOutput, result: PolicyResult) -> None:
    msg.source_trajectory_id = str(result.source_trajectory_id)
    msg.decision = int(result.decision)
    msg.reason = int(result.reason)
    msg.prototype_policy = bool(result.prototype_policy)


class DecisionPolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("decision_policy")
        self.declare_parameter("decision_policy.prototype_only", True)
        self.declare_parameter("decision_policy.stop_on_collision_candidate", True)
        self.declare_parameter(
            "decision_policy.stop_on_missing_current_prediction", True
        )
        self.declare_parameter("decision_policy.rollover_policy.enabled", False)
        self.declare_parameter(
            "decision_policy.rollover_policy.metric", METRIC_STABILITY_MOMENT
        )
        # NaN means unset; rollover policy remains disabled until explicitly configured.
        self.declare_parameter("decision_policy.rollover_policy.threshold", float("nan"))

        self._config = _load_policy_config(self)
        qos = 10
        self.create_subscription(
            DecisionEvidence, "/decision/evidence", self._on_evidence, qos
        )
        self._pub = self.create_publisher(DecisionOutput, "/decision", qos)
        self.get_logger().info(
            "decision_policy ready (prototype STOP/GO; rollover_policy.enabled="
            f"{self._config.rollover_policy.enabled})"
        )

    def _on_evidence(self, msg: DecisionEvidence) -> None:
        result = evaluate_policy(msg, self._config)
        out = DecisionOutput()
        out.header = msg.header
        _apply_policy(out, result)
        self._pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DecisionPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
