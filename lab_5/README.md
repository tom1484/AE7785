# team85_lab5

AE7785 Lab 5 (team 85): a TurtleBot3 Burger drives through global waypoints using the ROS 2 Humble Nav2 stack, with a custom recovery routine.

To run it on macOS with RoboStack, see [INSTRUCTION.md](INSTRUCTION.md).

## Packages

| Package | Build type | Contents |
|---|---|---|
| `team85_lab5` | ament_python | Waypoint node, launch file, waypoints, maps |
| `turtlebot3_recovery_lab` | ament_cmake | Instructor's recovery package: behavior tree, BT plugin, `nav2_params.yaml`, `recovery_student.py` |

Both go in the `src/` directory of a colcon workspace.

## Layout

| Path | Purpose |
|---|---|
| `team85_lab5/scripts/waypoint_navigator.py` | Sends the waypoints in `wayPoints.txt` to Nav2 one by one (NavigateToPose action) |
| `team85_lab5/scripts/set_initial_pose.py` | Sets AMCL's initial pose from parameters |
| `launch/navigation.launch.py` | Recovery node, Nav2, RViz; `waypoints:=true` also runs the waypoints |
| `launch/navigation_sim.launch.py` | Same in Gazebo (7785 maze), with the sim map and candidates |
| `launch/mapping.launch.py`, `mapping_sim.launch.py` | Cartographer mapping, real robot or Gazebo |
| `launch/robot.launch.py` | Starts `waypoint_navigator` |
| `config/robot.yaml` | Parameters for `waypoint_navigator` |
| `wayPoints.txt` | Goal points for grading day: `x y` per line, in the map frame |
| `wayPoints_sim.txt` | Test waypoints for the Gazebo maze |
| `maps/` | Saved maps: `map.*` (real), `map_sim.*` (Gazebo) |
| `scripts/launch/` | Scripts that run this package's launch files (`mapping`, `navigation`, `waypoints`) |
| `scripts/` | Other scripts: Nav2/Cartographer/Gazebo alone, recovery node, RViz, teleop, map saving, `print_initial_pose` |
| `copy_to_repo.sh` | Copies both packages to `lab_5/` in the course repo |
| `test/` | Lint tests |

The scripts run on the real robot by default; add `--sim` for the Gazebo
simulator (ROS domain 77, localhost only).

## Grading day

1. Start the robot bringup, then `scripts/launch/navigation.zsh`.
2. Set the initial pose with 2D Pose Estimate in RViz and check the scan
   matches the map.
3. Put the 3 goal points in `wayPoints.txt` (`ros2 topic echo /clicked_point`
   with Publish Point gives the coordinates).
4. `scripts/launch/waypoints.zsh`.
