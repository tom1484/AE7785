"""
Mapping in simulation: Gazebo with the 7785 maze + Cartographer (+ RViz).

Run it through scripts/launch/mapping.zsh --sim, which sets the sim ROS domain first.
Drive with scripts/teleop.zsh --sim, then save with scripts/save.zsh --sim
(maps/map_sim.*) while this is running.

Usage:
    ros2 launch team85_lab5 mapping_sim.launch.py
    ros2 launch team85_lab5 mapping_sim.launch.py gui:=false   # no Gazebo window
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mapping = os.path.join(
        get_package_share_directory("team85_lab5"), "launch", "mapping.launch.py"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Show the Gazebo window (false: lighter on the CPU)",
        ),
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
        *maze(LaunchConfiguration("gui")),
        # Give Gazebo time to load the maze and start publishing /clock
        TimerAction(period=10.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping),
                launch_arguments={"use_sim_time": "true"}.items(),
            ),
        ]),
    ])


def maze(gui):
    """Gazebo with the 7785 maze, as turtlebot3_gazebo's 7785_maze.launch.py,
    but with the Gazebo window (gzclient) optional: it's the heaviest part,
    and an overloaded Mac can make Nav2 miss heartbeats and reset itself."""
    gazebo_ros = get_package_share_directory("gazebo_ros")
    tb3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, "launch", "gzserver.launch.py")
            ),
            launch_arguments={
                "world": os.path.join(tb3_gazebo, "worlds", "7785_maze.world")
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, "launch", "gzclient.launch.py")
            ),
            condition=IfCondition(gui),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo, "launch", "robot_state_publisher.launch.py")
            ),
            launch_arguments={"use_sim_time": "true"}.items(),
        ),
    ]
