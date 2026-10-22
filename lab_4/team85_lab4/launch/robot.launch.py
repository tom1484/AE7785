"""
Nodes that run on the robot.

    scan_processor -> obstacle_state --+
                                       +--> go_to_goal -> low_level_controller -> /cmd_vel
    goal ------------------------------+

Usage:
    ros2 launch team85_lab4 robot.launch.py
    ros2 launch team85_lab4 robot.launch.py params_file:=/path/to/robot.yaml
    ros2 launch team85_lab4 robot.launch.py verbose:=1

verbose: 0 = no logging, 1 = one-off messages, 2 = also continuous results.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


NODES = ["scan_processor", "obstacle_state", "goal", "go_to_goal", "low_level_controller"]


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    # Applied after params_file, so verbose:= overrides it for every node
    verbose_params = {"verbose": ParameterValue(LaunchConfiguration("verbose"), value_type=int)}

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                get_package_share_directory("team85_lab4"), "config", "robot.yaml"
            ),
            description="Parameter file for the robot nodes",
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value="2",
            description="Logging level: 0 none, 1 one-off messages, 2 also continuous results",
        ),
        *[
            Node(
                package="team85_lab4",
                executable=name,
                name=name,
                parameters=[params_file, verbose_params],
                output="screen",
            )
            for name in NODES
        ],
    ])
