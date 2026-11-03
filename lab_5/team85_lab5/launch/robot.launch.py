"""
Waypoint navigator. Nav2 and the recovery server are started separately; see INSTRUCTION.md.

Usage:
    ros2 launch team85_lab5 robot.launch.py
    ros2 launch team85_lab5 robot.launch.py params_file:=/path/to/robot.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                get_package_share_directory("team85_lab5"), "config", "robot.yaml"
            ),
            description="Parameter file for waypoint_navigator",
        ),
        Node(
            package="team85_lab5",
            executable="waypoint_navigator",
            name="waypoint_navigator",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
