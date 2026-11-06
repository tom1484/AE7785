"""
Mapping with Cartographer (+ RViz), for the real robot by default.

Start the robot's bringup first. Drive with teleop in another terminal
(scripts/teleop.zsh), then save with scripts/save.zsh while this is running.

Usage:
    ros2 launch team85_lab5 mapping.launch.py
    ros2 launch team85_lab5 mapping.launch.py use_sim_time:=true   # or mapping_sim.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    cartographer = os.path.join(
        get_package_share_directory("turtlebot3_cartographer"),
        "launch", "cartographer.launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="true in Gazebo"
        ),
        # Uses the workspace's cartographer_ros build (fixed for macOS)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cartographer),
            launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
        ),
    ])
