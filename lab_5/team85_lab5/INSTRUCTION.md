# Running Lab 5 on my Mac

How to run Lab 5 on macOS (arm64) with the RoboStack `humble` conda env. For creating the env itself, see [env_setup.md](../../env_setup.md).

## Reminders

1. **Don't use Cartographer.** `cartographer_node` crashes at startup on this install. Map with `slam_toolbox` instead.
2. **Don't run `mamba update --all`.** It moves setuptools from 79.0.1 to 84.2.1, which breaks the pin in `env_setup.md`, and it offers no ROS updates. Install specific packages only.
3. **Start the recovery node with `python`, not `ros2 run`,** and start it **before** Nav2.
4. **Keep the three recovery package fixes below.** Unzipping `turtlebot3_recovery_lab.zip` again overwrites them.
5. **The TODOs in `recovery_student.py` are still unfinished.** Until they're filled in, the node starts but fails as soon as a real recovery is triggered.
6. **Check the domain before testing in sim.** The default `ROS_DOMAIN_ID=85` can reach a real robot on the network, and a Nav2 goal would drive it.

## Fixes applied to `src/turtlebot3_recovery_lab`

These change the instructor's package, so they'll be in `TeamName_Lab5.zip` when you submit it.

| # | File | Change | What goes wrong without it |
|---|---|---|---|
| 1 | `CMakeLists.txt` | `install(PROGRAMS ...)` points to `recovery_student.py` instead of `recovery.py` | `colcon build` fails because `recovery.py` isn't in the zip. **This affects every platform, so tell the TAs.** |
| 2 | `CMakeLists.txt` | Added the `if(APPLE)` link option below, just above `ament_package()` | `bt_navigator` fails with `symbol not found in flat namespace '_PyBool_FromLong'` |
| 3 | `recovery_student.py` | `chmod +x turtlebot3_recovery_nodes/recovery_student.py` | The zip ships the script without the executable bit. It's installed as a program, so it should be executable. |

```cmake
if(APPLE)
  target_link_options(beep_and_divert_bridge_plugin PRIVATE -Wl,-dead_strip_dylibs)
endif()
```

Fix 2 only applies on macOS, so it does nothing on the Lab PCs.

## Every new terminal

```zsh
zsh                        # my login shell is fish
conda activate humble
cd ~/Workspace/turtlebot3_ws
source install/setup.zsh
```

Rebuild after changing code:

```zsh
colcon build
source install/setup.zsh
```

**For simulation only,** also run this in every terminal so nothing reaches a real robot:

```zsh
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
```

Use the same values in all terminals, or they won't see each other.

## Simulation

### 1. Start Gazebo

```zsh
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

For the maze, clone `Lab5_Gazebo` (needs a GT login) and follow its README: https://github.gatech.edu/7785-Intro-to-Robotics-Research/Lab5_Gazebo

### 2. Make a map

Use this instead of the emanual's Cartographer step. Drive with teleop, then save:

```zsh
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
ros2 run turtlebot3_teleop teleop_keyboard
ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args -p use_sim_time:=true
```

Stop `slam_toolbox` once the map is saved, before starting Nav2.

### 3. Navigate with recovery

Start the recovery node first:

```zsh
python $(ros2 pkg prefix turtlebot3_recovery_lab)/lib/turtlebot3_recovery_lab/recovery_student.py --ros-args -p use_sim_time:=true
```

Then, in another terminal, start Nav2. Change `map:=` to your map:

```zsh
ros2 launch nav2_bringup bringup_launch.py \
  map:=$HOME/map.yaml \
  params_file:=$HOME/Workspace/turtlebot3_ws/src/turtlebot3_recovery_lab/config/nav2_params.yaml \
  use_sim_time:=True autostart:=True
```

Set the initial pose. These values match the spawn point in `turtlebot3_world`, so change `x` and `y` for other worlds:

```zsh
ros2 topic pub -t 3 -r 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"
```

Send a goal:

```zsh
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: -1.5, y: -0.5}, orientation: {w: 1.0}}}}"
```

To force a recovery, send a goal outside the map, such as `x: 50.0, y: 50.0`. The planner fails, recovery runs up to 3 times, and the goal ends as `ABORTED`. That's the expected result for an unreachable goal.

Gazebo has no `/sound` service, so `Cannot beep: /sound service unavailable` is normal in sim. Beeps can only be tested on the robot.

## Real robot

Skip the two simulation exports so you stay on domain 85, skip Gazebo, and bring up the robot as in the earlier labs. Then run the same commands with sim time off:

- **Mapping:** `use_sim_time:=False`, and leave out `--ros-args -p use_sim_time:=true`
- **Recovery node:** leave out `--ros-args -p use_sim_time:=true`
- **Nav2:** `use_sim_time:=False`

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `flag 'collect_metrics' was defined more than once` | Bug in the RoboStack Cartographer build | Use `slam_toolbox` |
| `Could not load library libturtlebot3_msgs__rosidl_typesupport_fastrtps_c.dylib` from the recovery node | The `#!/usr/bin/env python3` shebang goes through a protected macOS binary, which strips `DYLD_LIBRARY_PATH` | Start it with `python <path>` as shown above, not `ros2 run` |
| `"/divert_robot" service server not available`, then `Magick: abort due to signal 11` | The recovery node wasn't running when Nav2 started | Start the recovery node first, then restart Nav2 |
| `class nav2_navfn_planner::NavfnPlanner ... does not exist` | Used `turtlebot3_navigation2/param/burger.yaml`, which is for newer ROS distros | Use the lab's `nav2_params.yaml`, or `param/humble/burger.yaml` |
| Gazebo hangs on exit, or the next launch misbehaves | `gzserver` can ignore SIGTERM and keep running | `pkill -9 gzserver; pkill -9 gzclient` |
| `pytest` crashes with `Plugin 'launch_testing' ... PluginValidationError` | The `launch_testing` pytest plugin in this env is incompatible with the installed pytest | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test` |
| `ros2 topic` or `ros2 lifecycle` shows a `!rclpy.ok()` fault | The ROS CLI daemon misbehaved. Nav2 itself was fine when this happened. | Try `ros2 daemon stop`, then rerun. Seen once; this fix isn't confirmed. |

## What has been tested

Tested in simulation on `turtlebot3_world`, 2026-11-01 and 2026-11-02:

- Gazebo launches, the burger spawns, and `/clock`, `/scan`, `/odom` and `/imu` publish.
- `slam_toolbox` publishes `/map` and `map_saver_cli` saves it, with the robot stationary.
- Nav2 reaches a goal with the stock Humble params and with the lab's `nav2_params.yaml`, using the stock TurtleBot3 map.
- `colcon build` passes for the whole workspace (21 packages).
- The recovery sequence ran on a scratch copy of the package, using a temporary test waypoint.

Not tested yet:

- Anything on the real robot, including the beeps
- The `Lab5_Gazebo` maze
- Driving with teleop, or a full map built while moving
- Nav2 on a map made with `slam_toolbox`
- How the Gazebo window looks (only checked that the process stays up), and RViz (never launched)
- Recovery using your finished `recovery_student.py`
