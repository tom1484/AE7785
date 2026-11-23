"""
Recovery server, then Nav2 (map server, AMCL, planners) + RViz on the final's own
map and params.

The recovery server (team85_final_recovery) must be up before Nav2: bt_navigator
crashes if /divert_robot is missing. Its candidates are the maze cell centers
(config/maze.yaml, or maze_sim.yaml in sim).

Usage:
    ros2 launch team85_final navigation.launch.py                 # robot bringup first
    ros2 launch team85_final navigation.launch.py sim:=true       # + Gazebo, sim map, pose at spawn
    ros2 launch team85_final navigation.launch.py start:=0,0,S      # robot at cell (0,0) facing S
        (set_initial_pose: ICP of the LIDAR against the map corrects the hand placement,
         then a 360 deg spin lets AMCL converge; refine_pose:=false / spin:=false to skip)
    ros2 launch team85_final navigation.launch.py set_initial_pose:=true \
        initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0
"""

import math
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

FINAL = get_package_share_directory("team85_final")
BRINGUP = os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false",
                              description="Also start Gazebo; sim map and sim time"),
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo window (sim)"),
        DeclareLaunchArgument("map", default_value="",
                              description="Map yaml (default: maps/map.yaml, or map_sim.yaml in sim)"),
        DeclareLaunchArgument("nav2_params_file", default_value="",
                              description="Nav2 params (default: config/nav2_params.yaml, "
                                          "or nav2_params_fast.yaml with fast:=true)"),
        DeclareLaunchArgument("fast", default_value="false",
                              description="Faster Nav2 profile (config/nav2_params_fast.yaml)"),
        DeclareLaunchArgument("candidates_file", default_value="",
                              description="Recovery candidates (default: config/maze.yaml, "
                                          "or maze_sim.yaml in sim)"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("start", default_value="",
                              description="row,col,heading (E/N/W/S): AMCL pose = that cell's center "
                                          "facing that way, from the maze yaml (e.g. 0,0,S)"),
        DeclareLaunchArgument("set_initial_pose", default_value="",
                              description="Set AMCL's pose from initial_* (default: true in sim)"),
        # Empty = in sim, the spawn cell from maze_sim.yaml (`spawn`); on the robot, 0.0.
        DeclareLaunchArgument("initial_x", default_value=""),
        DeclareLaunchArgument("initial_y", default_value=""),
        DeclareLaunchArgument("initial_yaw", default_value="", description="(rad)"),
        DeclareLaunchArgument("refine_pose", default_value="true",
                              description="ICP the LIDAR against the map to correct the hand placement"),
        DeclareLaunchArgument("spin", default_value="true",
                              description="After the initial pose: turn 360 deg in place so AMCL converges"),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name, default=False):
        v = arg(name).lower()
        return default if v == "" else v in ("true", "1", "yes")

    sim = flag("sim")
    map_yaml = arg("map") or os.path.join(FINAL, "maps", "map_sim.yaml" if sim else "map.yaml")

    nav = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(BRINGUP),
        launch_arguments={
            "map": map_yaml,
            "params_file": arg("nav2_params_file") or os.path.join(
                FINAL, "config", "nav2_params_fast.yaml" if flag("fast") else "nav2_params.yaml"),
            "use_sim_time": str(sim),
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )]
    # Sim: the robot spawns at the center of the maze's `spawn` cell. Not (0, 0, 0): map_sim's
    # origin is wherever Cartographer put it, which was not the spawn pose (config/maze_sim.yaml).
    init = [arg("initial_x"), arg("initial_y"), arg("initial_yaw")]
    maze_yaml = os.path.join(FINAL, "config", "maze_sim.yaml" if sim else "maze.yaml")
    start = arg("start")
    if start:                                   # demo day: "start in (0,0) facing south"
        r, c, h = (v.strip() for v in start.split(","))
        init = [str(v) for v in cell_pose(maze_yaml, [r, c, h])]
    elif sim and "" in init:
        sx, sy, syaw = spawn_pose(maze_yaml)
        init = [v or str(d) for v, d in zip(init, (sx, sy, syaw))]
    init = [float(v or 0.0) for v in init]
    if flag("set_initial_pose", default=sim or bool(start)):
        nav.append(Node(
            package="team85_final", executable="set_initial_pose", output="screen",
            prefix=sys.executable,
            parameters=[{
                "use_sim_time": sim,
                "x": init[0],
                "y": init[1],
                "yaw": init[2],
                "refine": flag("refine_pose", default=True),
                "map_yaml": map_yaml,
                "spin": 2 * math.pi if flag("spin", default=True) else 0.0,
            }],
        ))
    # `python` prefix: a #!/usr/bin/env shebang drops DYLD_LIBRARY_PATH on macOS (SIP).
    recovery = Node(
        package="team85_final_recovery", executable="recovery_server.py",
        prefix=sys.executable, output="screen",
        parameters=[{
            "use_sim_time": sim,
            "candidates_file": arg("candidates_file") or os.path.join(
                FINAL, "config", "maze_sim.yaml" if sim else "maze.yaml"),
        }],
    )
    actions = [
        recovery,
        # Nav2 (and what needs it) only after /divert_robot can be up
        RegisterEventHandler(OnProcessStart(
            target_action=recovery, on_start=[TimerAction(period=3.0, actions=nav)],
        )),
    ]
    if flag("rviz"):
        actions.append(Node(
            package="rviz2", executable="rviz2", output="screen",
            arguments=["-d", os.path.join(FINAL, "rviz", "navigation.rviz")],
            parameters=[{"use_sim_time": sim}],
        ))

    if not sim:
        return actions
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(FINAL, "launch", "sim.launch.py")),
            launch_arguments={"gui": arg("gui")}.items(),
        ),
        # Give Gazebo time to load the maze and start publishing /clock
        TimerAction(period=10.0, actions=actions),
    ]


def spawn_pose(maze_yaml):
    """Map-frame (x, y, yaw) of the center of the maze yaml's `spawn: [row, col, heading]`."""
    import yaml

    with open(maze_yaml) as f:
        return cell_pose(maze_yaml, yaml.safe_load(f).get("spawn", [0, 0, "E"]))


def cell_pose(maze_yaml, rch):
    """Map-frame (x, y, yaw) of cell (row, col)'s center facing heading, in maze_yaml's grid."""
    from team85_final.maze.geometry import Cell, CellPose, Heading
    from team85_final.maze.grid import MazeGrid

    r, c, h = rch
    return MazeGrid.from_yaml(maze_yaml).pose_to_map(
        CellPose(Cell(int(r), int(c)), Heading.parse(str(h))))
