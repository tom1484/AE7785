"""
The whole final run: navigation.launch.py + sign_detector + mission.

Usage:
    ros2 launch team85_final final.launch.py                       # robot bringup first
    ros2 launch team85_final final.launch.py sim:=true
    ros2 launch team85_final final.launch.py planner:=graph
    ros2 launch team85_final final.launch.py sim:=true planner:=explore \
        planner_params:="{steps: 15, seed: 1}" graph_dump:=/tmp/graph.yaml
All navigation.launch.py arguments pass through (map, rviz, set_initial_pose, ...).
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FINAL = get_package_share_directory("team85_final")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("planner", default_value="",
                              description="Override the planner in config/final.yaml (grid|graph)"),
        DeclareLaunchArgument("planner_params", default_value="",
                              description='Override planner_params, e.g. "{steps: 20, seed: 3}"'),
        DeclareLaunchArgument("graph_dump", default_value="",
                              description="Write the planner's graph (yaml) here at the end"),
        DeclareLaunchArgument("onboard", default_value="false",
                              description="Running on the robot's Pi: detector 2 Hz / 2 threads; Nav2 composed + lower rates"),
        DeclareLaunchArgument("detector_rate", default_value="",
                              description="Override sign_detector max_rate (Hz)"),
        DeclareLaunchArgument("detector_model", default_value="",
                              description="Override sign_detector model (path; relative = package share)"),
        DeclareLaunchArgument("detector_imgsz", default_value="",
                              description="Override sign_detector imgsz (must match the model)"),
        DeclareLaunchArgument("final_params_file",  # not "params_file": gzserver and nav2 bringup use that name and launch configs are global
                              default_value=os.path.join(FINAL, "config", "final.yaml")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(FINAL, "launch", "navigation.launch.py")),
        ),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    sim = LaunchConfiguration("sim").perform(context).lower() in ("true", "1", "yes")
    params = [LaunchConfiguration("final_params_file").perform(context), {"use_sim_time": sim}]
    planner = LaunchConfiguration("planner").perform(context)
    mission_params = params + ([{"planner": planner}] if planner else [])
    # Same rule as navigation.launch.py: is set_initial_pose running? Then the mission
    # waits for its /localization_ready (ICP + spin) before the first goal.
    start = LaunchConfiguration("start", default="").perform(context)
    sip = LaunchConfiguration("set_initial_pose", default="").perform(context).lower()
    mission_params.append({"wait_localized": (sim or bool(start)) if sip == ""
                           else sip in ("true", "1", "yes")})
    for name in ("planner_params", "graph_dump"):
        value = LaunchConfiguration(name).perform(context)
        if value:
            mission_params.append({name: value})
    if sim:
        # Gazebo's camera_rgb_frame is 0.069 m ahead of base_link; final.yaml holds the robot's.
        mission_params.append({"maze_file": "config/maze_sim.yaml", "camera_offset_x": 0.069,
                               "camera_offset_y": 0.0})
    # Gazebo's camera publishes raw Image, the robot's CompressedImage (sim/README.md).
    detector_params = params + ([{"image_topic": "/simulated_camera/image_raw", "compressed": False,
                                  "camera_info_topic": "/simulated_camera/camera_info",
                                  "publish_debug_image": True}]
                                if sim else [])
    model = LaunchConfiguration("detector_model").perform(context)
    imgsz = LaunchConfiguration("detector_imgsz").perform(context)
    if model:
        detector_params.append({"model": model})
    if imgsz:
        detector_params.append({"imgsz": int(imgsz)})
    if LaunchConfiguration("onboard").perform(context).lower() in ("true", "1", "yes"):
        # Pi 4: one 416 inference takes ~0.4 s; uncapped, OpenCV's threads starve Nav2.
        detector_params.append({"max_rate": 2.0, "threads": 2})
    rate = LaunchConfiguration("detector_rate").perform(context)
    if rate:
        detector_params.append({"max_rate": float(rate)})
    nodes = [
        # `python` prefix: a #!/usr/bin/env shebang drops DYLD_LIBRARY_PATH on macOS (SIP),
        # and the workspace message types then fail to load.
        Node(package="team85_final", executable="sign_detector", prefix=sys.executable,
             output="screen", parameters=detector_params),
        Node(package="team85_final", executable="mission", prefix=sys.executable,
             output="screen", parameters=mission_params),
    ]
    # In sim, wait for Gazebo + Nav2 (navigation.launch.py delays Nav2 by 10 s).
    return [TimerAction(period=15.0 if sim else 3.0, actions=nodes)]
