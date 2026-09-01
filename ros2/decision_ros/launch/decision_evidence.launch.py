from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock from /clock",
            ),
            Node(
                package="decision_ros",
                executable="decision_evidence_node",
                name="decision_evidence",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
