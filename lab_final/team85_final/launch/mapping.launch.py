"""
Mapping the final maze with Cartographer (+ RViz). Real robot by default.

The final records its own map (the Lab 5 maze is a different layout). Drive
with scripts/teleop.zsh, save with scripts/save.zsh (maps/map.* or, with --sim,
maps/map_sim.*), then rebuild so the map is installed.

Cartographer runs under nodes/mapping_supervisor.py, which can reset the map without
quitting: click the red RESET MAP box above the robot in RViz (Interact tool), or
`ros2 service call /reset_map std_srvs/srv/Trigger`. The new map starts at the robot.

Config: config/cartographer/maze_lds_2d.lua (loop closure limited to 0.3 m / 10 deg:
false loop closures between look-alike cells teleported the pose on the real robot).
config:=stock uses turtlebot3's turtlebot3_lds_2d.lua (7 m / 30 deg).

Usage:
    ros2 launch team85_final mapping.launch.py             # robot bringup first
    ros2 launch team85_final mapping.launch.py sim:=true   # Gazebo maze too
    ... config:=stock  use_rviz:=false  gui:=false
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FINAL = get_package_share_directory("team85_final")
CONFIGS = {
    "maze": (os.path.join(FINAL, "config", "cartographer"), "maze_lds_2d.lua"),
    "stock": (os.path.join(get_package_share_directory("turtlebot3_cartographer"), "config"),
              "turtlebot3_lds_2d.lua"),
}


def launch_setup(context):
    sim = LaunchConfiguration("sim").perform(context).lower() in ("true", "1", "yes")
    name = LaunchConfiguration("config").perform(context)
    if name not in CONFIGS:
        raise ValueError(f"config:={name}; choose one of {sorted(CONFIGS)}")
    directory, basename = CONFIGS[name]
    nodes = [
        # `python` prefix: macOS SIP drops DYLD_LIBRARY_PATH through a #!/usr/bin/env shebang.
        Node(package="team85_final", executable="mapping_supervisor", prefix=sys.executable,
             output="screen",
             parameters=[{"use_sim_time": sim, "configuration_directory": directory,
                          "configuration_basename": basename}]),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", os.path.join(FINAL, "rviz", "mapping.rviz")],
             parameters=[{"use_sim_time": sim}],
             condition=IfCondition(LaunchConfiguration("use_rviz"))),
    ]
    # In sim, give Gazebo time to load the maze and start publishing /clock.
    return [TimerAction(period=10.0, actions=nodes)] if sim else nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo window (sim)"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("config", default_value="maze",
                              description="Cartographer config: maze (near loop closure) | stock"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(FINAL, "launch", "sim.launch.py")),
            launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
            condition=IfCondition(LaunchConfiguration("sim")),
        ),
        OpaqueFunction(function=launch_setup),
    ])
