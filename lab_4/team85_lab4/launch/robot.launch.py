"""
Nodes that run on the robot.

Usage:
    ros2 launch team85_lab4 robot.launch.py
    ros2 launch team85_lab4 robot.launch.py params_file:=/path/to/robot.yaml
    ros2 launch team85_lab4 robot.launch.py log:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    # Applied after params_file, so log:= overrides it for every node
    log_params = {"log_result": ParameterValue(LaunchConfiguration("log"), value_type=bool)}

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                get_package_share_directory("team85_lab4"), "config", "robot.yaml"
            ),
            description="Parameter file for the robot nodes",
        ),
        DeclareLaunchArgument(
            "log",
            default_value="true",
            description="Print node results (e.g. /object_position) to the terminal",
        ),
        Node(
            package="team85_lab4",
            executable="get_object_range",
            name="get_object_range",
            parameters=[params_file, log_params],
            output="screen",
        ),
    ])
