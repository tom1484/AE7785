"""
Debug / monitoring nodes that run on the host computer, never on the robot.

view_scan gets the segmentation and detection parameters of get_object_range
from the robot parameter file, so it shows what the robot computes. Values in
the debug parameter file override them.

Usage:
    ros2 launch team85_lab4 debug.launch.py
    ros2 launch team85_lab4 debug.launch.py params_file:=/path/to/debug.yaml \
        robot_params_file:=/path/to/robot.yaml
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Parameters of get_object_range that view_scan also uses
SHARED_PARAMS = (
    "min_points", "base_threshold", "k",
    "field_of_view", "normal_radius", "min_normal_points",
)


def load_shared_params(robot_params_file):
    with open(robot_params_file) as f:
        params = yaml.safe_load(f) or {}
    node_params = params.get("get_object_range", {}).get("ros__parameters", {})
    return {key: node_params[key] for key in SHARED_PARAMS if key in node_params}


def launch_nodes(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    robot_params_file = LaunchConfiguration("robot_params_file").perform(context)

    return [
        Node(
            package="team85_lab4",
            executable="view_scan",
            name="view_scan",
            # Later entries override earlier ones
            parameters=[load_shared_params(robot_params_file), params_file],
            output="screen",
        ),
    ]


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("team85_lab4"), "config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(config_dir, "debug.yaml"),
            description="Parameter file for the debug nodes",
        ),
        DeclareLaunchArgument(
            "robot_params_file",
            default_value=os.path.join(config_dir, "robot.yaml"),
            description="Robot parameter file to share segmentation/detection parameters from",
        ),
        OpaqueFunction(function=launch_nodes),
    ])
