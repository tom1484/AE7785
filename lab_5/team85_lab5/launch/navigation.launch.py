"""
Navigation with the lab's recovery: recovery node, Nav2, RViz, and
optionally the initial pose and the waypoint navigator. Real robot by default.

Start the robot's bringup first. The recovery node starts first and Nav2 a few
seconds later, since bt_navigator crashes if /divert_robot is missing.

Usage:
    ros2 launch team85_lab5 navigation.launch.py
    ros2 launch team85_lab5 navigation.launch.py waypoints:=true
    ros2 launch team85_lab5 navigation.launch.py set_initial_pose:=true \
        initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0
    ros2 launch team85_lab5 navigation.launch.py map:=/path/to/map.yaml rviz:=false
    ros2 launch team85_lab5 navigation.launch.py nav2_params_file:=/path/to/nav2_params.yaml

Without set_initial_pose, set it with 2D Pose Estimate in RViz. The waypoint
navigator waits for it before sending goals.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LAB5 = get_package_share_directory("team85_lab5")
RECOVERY = get_package_share_directory("turtlebot3_recovery_lab")
BRINGUP = os.path.join(
    get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py"
)
# turtlebot3_navigation2's layout minus the Nav2 plugins that crash RViz on exit
RVIZ_CONFIG = os.path.join(LAB5, "rviz", "navigation.rviz")


def generate_launch_description():
    lab5, recovery = LAB5, RECOVERY

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "map", default_value=os.path.join(lab5, "maps", "map.yaml"),
            description="Map yaml (rebuild after saving a new map so it's installed)",
        ),
        # Not "params_file": robot.launch.py (waypoints) uses that name for its
        # own file, and launch arguments are shared between included files.
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=os.path.join(recovery, "config", "nav2_params.yaml"),
            description="Nav2 parameters (the lab's recovery behavior tree)",
        ),
        DeclareLaunchArgument(
            "candidates_file",
            default_value=os.path.join(recovery, "config", "recovery_candidates.yaml"),
            description="Recovery waypoint candidates",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "set_initial_pose", default_value="false",
            description="Set the initial pose from initial_x/y/yaw instead of RViz",
        ),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0", description="(rad)"),
        DeclareLaunchArgument(
            "waypoints", default_value="false",
            description="Also run waypoint_navigator",
        ),
        DeclareLaunchArgument(
            "waypoints_file", default_value=os.path.join(lab5, "wayPoints.txt"),
        ),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):
    """Build the actions from the argument values as plain strings.

    Nav2 starts later, from an event handler. Reading the arguments here, once,
    keeps it from seeing different values by then (launch arguments are shared
    between all included launch files).
    """
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return arg(name).lower() in ("true", "1", "yes")

    use_sim_time = flag("use_sim_time")

    # Started with `python` rather than ros2 run: its #!/usr/bin/env shebang
    # goes through a protected macOS binary, which drops DYLD_LIBRARY_PATH.
    recovery_node = Node(
        package="turtlebot3_recovery_lab",
        executable="recovery_student.py",
        prefix=sys.executable,
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "candidates_file": arg("candidates_file"),
        }],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(BRINGUP),
        launch_arguments={
            "map": arg("map"),
            "params_file": arg("nav2_params_file"),
            "use_sim_time": str(use_sim_time),
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )
    after_recovery = [nav2]

    if flag("set_initial_pose"):
        after_recovery.append(Node(
            package="team85_lab5",
            executable="set_initial_pose",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "x": float(arg("initial_x")),
                "y": float(arg("initial_y")),
                "yaw": float(arg("initial_yaw")),
            }],
        ))

    if flag("waypoints"):
        after_recovery.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(LAB5, "launch", "robot.launch.py")),
            launch_arguments={
                "params_file": os.path.join(LAB5, "config", "robot.yaml"),
                "use_sim_time": str(use_sim_time).lower(),
                "waypoints_file": arg("waypoints_file"),
            }.items(),
        ))

    actions = [
        recovery_node,
        # Nav2 (and what needs it) only after /divert_robot can be up
        RegisterEventHandler(OnProcessStart(
            target_action=recovery_node,
            on_start=[TimerAction(period=3.0, actions=after_recovery)],
        )),
    ]

    if flag("rviz"):
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", RVIZ_CONFIG],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ))

    return actions
