#!/usr/bin/env python3
"""Build /rover/state from MAVLink position + attitude with finite-difference accel."""

from __future__ import annotations

import math
from collections import deque

from geometry_msgs.msg import Accel, PointStamped, Pose, QuaternionStamped, Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from safety_perception_msgs.msg import RoverState

from lr_prediction_bridge.helpers import finite_difference


class RoverStateAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("rover_state_adapter_node")
        self.declare_parameter("position_topic", "/lr/mavlink/position_enu")
        self.declare_parameter("attitude_topic", "/lr/mavlink/attitude_enu")
        self.declare_parameter("output_topic", "/rover/state")
        self.declare_parameter("expected_frame_id", "map")
        self.declare_parameter("force_frame_id_map", True)
        self.declare_parameter("max_dt_sec", 1.0)
        self.declare_parameter("min_dt_sec", 1e-3)

        self._expected_frame = str(self.get_parameter("expected_frame_id").value)
        self._force_map = bool(self.get_parameter("force_frame_id_map").value)
        self._max_dt = float(self.get_parameter("max_dt_sec").value)
        self._min_dt = float(self.get_parameter("min_dt_sec").value)

        # (t, x, y, z, qx, qy, qz, qw)
        self._history: deque[tuple[float, float, float, float, float, float, float, float]] = deque(
            maxlen=3
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self._pub = self.create_publisher(
            RoverState, self.get_parameter("output_topic").value, qos
        )

        # ApproximateTimeSynchronizer needs message_filters Subscriber wrappers.
        pos_sub = Subscriber(
            self, PointStamped, self.get_parameter("position_topic").value, qos_profile=qos
        )
        att_sub = Subscriber(
            self,
            QuaternionStamped,
            self.get_parameter("attitude_topic").value,
            qos_profile=qos,
        )
        self._sync = ApproximateTimeSynchronizer(
            [pos_sub, att_sub], queue_size=20, slop=0.05
        )
        self._sync.registerCallback(self._on_pose)
        self.get_logger().info(
            "rover_state adapter: position+attitude → /rover/state "
            "(accel via finite difference; gravity excluded by construction)"
        )

    def _on_pose(self, position: PointStamped, attitude: QuaternionStamped) -> None:
        stamp = position.header.stamp
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        frame_id = position.header.frame_id.strip() or self._expected_frame
        if self._force_map:
            frame_id = "map"

        x = float(position.point.x)
        y = float(position.point.y)
        z = float(position.point.z)
        qx = float(attitude.quaternion.x)
        qy = float(attitude.quaternion.y)
        qz = float(attitude.quaternion.z)
        qw = float(attitude.quaternion.w)
        if not all(math.isfinite(v) for v in (t, x, y, z, qx, qy, qz, qw)):
            self.get_logger().warn("dropping non-finite pose sample")
            return

        self._history.append((t, x, y, z, qx, qy, qz, qw))

        state = RoverState()
        state.header.stamp = stamp
        state.header.frame_id = frame_id
        state.pose = Pose()
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw
        state.pose_valid = True

        state.twist = Twist()
        state.twist_valid = False
        state.acceleration = Accel()
        state.acceleration_valid = False

        if len(self._history) >= 2:
            t0, x0, y0, z0, *_ = self._history[-2]
            t1, x1, y1, z1, *_ = self._history[-1]
            dt = t1 - t0
            if self._min_dt <= dt <= self._max_dt:
                vx, vy, vz = finite_difference((x0, y0, z0), (x1, y1, z1), dt)
                state.twist.linear.x = vx
                state.twist.linear.y = vy
                state.twist.linear.z = vz
                state.twist_valid = True

        if len(self._history) >= 3 and state.twist_valid:
            t0, x0, y0, z0, *_ = self._history[-3]
            t1, x1, y1, z1, *_ = self._history[-2]
            t2, x2, y2, z2, *_ = self._history[-1]
            dt01 = t1 - t0
            dt12 = t2 - t1
            if (
                self._min_dt <= dt01 <= self._max_dt
                and self._min_dt <= dt12 <= self._max_dt
            ):
                v0 = finite_difference((x0, y0, z0), (x1, y1, z1), dt01)
                v1 = finite_difference((x1, y1, z1), (x2, y2, z2), dt12)
                # Acceleration at the latest interval (kinematic, no gravity term).
                ax, ay, az = finite_difference(v0, v1, dt12)
                state.acceleration.linear.x = ax
                state.acceleration.linear.y = ay
                state.acceleration.linear.z = az
                state.acceleration_valid = True

        self._pub.publish(state)


def main(argv: list[str] | None = None) -> None:
    import rclpy

    rclpy.init(args=argv)
    node = RoverStateAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
