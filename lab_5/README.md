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
| `team85_lab5/scripts/waypoint_navigator.py` | Sends the waypoints to Nav2 (stub, not implemented yet) |
| `launch/robot.launch.py` | Starts `waypoint_navigator` |
| `config/robot.yaml` | Parameters for `waypoint_navigator` |
| `wayPoints.txt` | Goal points for grading day, in the map frame |
| `maps/` | Saved maps (`.yaml` and `.pgm`); rebuild after adding one so it gets installed |
| `copy_to_repo.sh` | Copies both packages to `lab_5/` in the course repo |
| `test/` | Lint tests |
