#!/usr/bin/env python3
"""Launch Prediction visualization node (use_sim_time)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("prediction_visualization")
    default_params = os.path.join(pkg, "config", "prediction_visualization.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            Node(
                package="prediction_visualization",
                executable="prediction_visualization_node",
                name="prediction_visualization",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
