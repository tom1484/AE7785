"""
Waypoint navigator. Nav2 and the recovery server are started separately; see INSTRUCTION.md.

Usage:
    ros2 launch team85_lab5 robot.launch.py
    ros2 launch team85_lab5 robot.launch.py waypoints_file:=/path/to/wayPoints.txt
    ros2 launch team85_lab5 robot.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("team85_lab5")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share, "config", "robot.yaml"),
            description="Parameter file for waypoint_navigator",
        ),
        DeclareLaunchArgument(
            "waypoints_file",
            default_value=os.path.join(share, "wayPoints.txt"),
            description="Waypoints, one 'x y' or 'x y yaw' per line (map frame)",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="true in Gazebo",
        ),
        Node(
            package="team85_lab5",
            executable="waypoint_navigator",
            name="waypoint_navigator",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "waypoints_file": LaunchConfiguration("waypoints_file"),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                },
            ],
        ),
    ])
