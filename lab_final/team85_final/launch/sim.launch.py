"""
Gazebo with the maze (gzserver, optional gzclient, robot_state_publisher).

Included by the other launch files with sim:=true; runs alone too. Use the sim
ROS domain (scripts/*.zsh --sim). If Gazebo hangs on exit: pkill -9 gzserver gzclient

Default world: sim/7785_maze_signs.world from the course's Lab6_Gazebo repo (robot with
camera + signs built in; see sim/README.md). The sign models in sim/models are put on
GAZEBO_MODEL_PATH here. Camera: /simulated_camera/image_raw (raw Image; verified 2026-11-19).

Usage:
    ros2 launch team85_final sim.launch.py
    ros2 launch team85_final sim.launch.py gui:=false world:=/path/to/world
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_ros = get_package_share_directory("gazebo_ros")
    tb3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    final = get_package_share_directory("team85_final")
    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value=os.path.join(final, "sim", "7785_maze_signs.world"),
            description="Lab 5's maze without signs: <turtlebot3_gazebo>/worlds/7785_maze.world",
        ),
        AppendEnvironmentVariable(
            "GAZEBO_MODEL_PATH", os.path.join(final, "sim", "models"), prepend=True
        ),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Show the Gazebo window (false: lighter on the CPU)",
        ),
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gzserver.launch.py")),
            launch_arguments={"world": LaunchConfiguration("world")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gzclient.launch.py")),
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo, "launch", "robot_state_publisher.launch.py")
            ),
            launch_arguments={"use_sim_time": "true"}.items(),
        ),
    ])
