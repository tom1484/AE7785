"""
Debug / monitoring nodes that run on the host computer, never on the robot.

view_scan draws the outputs of scan_processor, goal_monitor the progress of
the goal node, and avoidance_monitor the decisions of obstacle_state (all
running on the robot).

Usage:
    ros2 launch team85_lab4 debug.launch.py
    ros2 launch team85_lab4 debug.launch.py params_file:=/path/to/debug.yaml
    ros2 launch team85_lab4 debug.launch.py robot_params_file:=/path/to/robot.yaml
    ros2 launch team85_lab4 debug.launch.py verbose:=0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


NODES = ["view_scan", "goal_monitor", "avoidance_monitor"]


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    # Only its shared (/**) section applies to the debug nodes
    robot_params_file = LaunchConfiguration("robot_params_file")
    verbose_params = {"verbose": ParameterValue(LaunchConfiguration("verbose"), value_type=int)}

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                get_package_share_directory("team85_lab4"), "config", "debug.yaml"
            ),
            description="Parameter file for the debug nodes",
        ),
        DeclareLaunchArgument(
            "robot_params_file",
            default_value=os.path.join(
                get_package_share_directory("team85_lab4"), "config", "robot.yaml"
            ),
            description="Robot parameter file, for parameters shared with the robot nodes",
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value="1",
            description="Logging level: 0 none, 1 one-off messages, 2 also continuous results",
        ),
        *[
            Node(
                package="team85_lab4",
                executable=name,
                name=name,
                parameters=[robot_params_file, params_file, verbose_params],
                output="screen",
            )
            for name in NODES
        ],
    ])
