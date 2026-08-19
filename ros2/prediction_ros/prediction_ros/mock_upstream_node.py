"""Minimal typed upstream publisher for ROS end-to-end development."""

from __future__ import annotations


def main() -> None:  # pragma: no cover - requires ROS-generated messages
    import rclpy
    from rclpy.node import Node
    from safety_perception_msgs.msg import GeometryArray, GeometryStep, TrackedObjectArray, Trajectory

    class MockUpstreamNode(Node):
        def __init__(self) -> None:
            super().__init__("mock_upstream_node")
            self.trajectory_pub = self.create_publisher(Trajectory, "/trajectory", 10)
            self.objects_pub = self.create_publisher(TrackedObjectArray, "/tracked_objects", 10)
            self.geometry_pub = self.create_publisher(GeometryArray, "/geometry", 10)
            self.timer = self.create_timer(0.5, self.publish_once)
            self.sent = False

        def publish_once(self) -> None:
            if self.sent:
                return
            now = self.get_clock().now().to_msg()
            trajectory = Trajectory()
            trajectory.header.stamp = now
            trajectory.header.frame_id = "map"
            trajectory.trajectory_id = 1
            from safety_perception_msgs.msg import TrajectoryStep
            for step_id in range(3):
                step = TrajectoryStep()
                step.step_id, step.x, step.y, step.yaw = step_id, float(step_id), 0.0, 0.0
                trajectory.steps.append(step)

            objects = TrackedObjectArray()
            objects.header.stamp, objects.header.frame_id = now, "map"

            geometry = GeometryArray()
            geometry.header.stamp, geometry.header.frame_id = now, "map"
            geometry.source_trajectory_id = 1
            geometry.source_trajectory_stamp = now
            for step_id in range(3):
                step = GeometryStep()
                step.step_id, step.plane_id = step_id, f"flat-{step_id}"
                step.normal.z = 1.0
                geometry.steps.append(step)

            self.trajectory_pub.publish(trajectory)
            self.objects_pub.publish(objects)
            self.geometry_pub.publish(geometry)
            self.sent = True
            self.get_logger().info("Published trajectory=1, empty objects, matching flat geometry")

    rclpy.init()
    node = MockUpstreamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
