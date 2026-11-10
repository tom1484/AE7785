# Running Lab 5 on my Mac

How to run Lab 5 on macOS (arm64) with the RoboStack `humble` conda env, on the
real robot and in the Gazebo maze. For creating the env itself, see
[env_setup.md](../../env_setup.md).

## Reminders

1. **Use the scripts in `scripts/`.** They source the workspace setup
   themselves, which macOS needs (see Troubleshooting). Add `--sim` to any of
   them for the simulator.
2. **The recovery node must be up before Nav2,** or `bt_navigator` crashes.
   `scripts/launch/navigation.zsh` does this for you.
3. **Don't run `mamba update --all`.** It moves setuptools from 79.0.1 to
   84.2.1, which breaks the pin in `env_setup.md`, and offers no ROS updates.
   Install specific packages only.
4. **Keep the fixes to the instructor's package below.** Unzipping
   `turtlebot3_recovery_lab.zip` again overwrites them.
5. **Cartographer is a patched source build** in `src/cartographer_ros` of the
   workspace (not in this repo). The RoboStack binary crashes at startup.
6. **Real robot terminals need `ROS_DOMAIN_ID=85` and
   `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`** (set in `~/.zshrc`). With another
   middleware, topics still work but service calls such as the buzzer
   (`/sound`) get no answer.

## Every new terminal

```zsh
zsh                        # my login shell is fish
conda activate humble
cd ~/Workspace/turtlebot3_ws
```

The scripts source `install/setup.zsh` themselves. Rebuild after changing C++
code, `CMakeLists.txt`, `setup.py`, or adding a map:

```zsh
colcon build --packages-select team85_lab5 turtlebot3_recovery_lab
```

Python code, launch files, configs and `wayPoints.txt` are installed as links
(`--symlink-install`), so editing them needs no rebuild.

## Scripts

All in `src/team85_lab5/scripts/`. Real robot by default; `--sim` runs on the
simulator's own ROS domain (77, localhost only), so it can't reach a robot.

| Script | What it starts |
|---|---|
| `launch/mapping.zsh` | Cartographer + RViz (`--sim`: plus the Gazebo maze) |
| `launch/navigation.zsh` | Recovery node, then Nav2, plus RViz (`--sim`: plus the Gazebo maze, sim map and candidates, initial pose at the spawn point) |
| `launch/waypoints.zsh` | Waypoint navigator (`wayPoints.txt`; `--sim`: `wayPoints_sim.txt`) |
| `teleop.zsh` | Keyboard teleop |
| `save.zsh` | Saves the map being built to `maps/map.*` (`--sim`: `maps/map_sim.*`) |
| `print_initial_pose.zsh` | Prints poses set with 2D Pose Estimate, as launch arguments |
| `rviz.zsh`, `recovery.zsh`, `nav_recovery.zsh`, `slam.zsh`, `gazebo_sim.zsh`, `nav.zsh` | Single pieces, for debugging |

Launch arguments go after the script, for example:

```zsh
scripts/launch/navigation.zsh waypoints:=true
scripts/launch/navigation.zsh set_initial_pose:=true initial_x:=0.5 initial_y:=0.2 initial_yaw:=1.57
scripts/launch/navigation.zsh --sim gui:=false rviz:=false waypoints:=true
```

`gui:=false` hides the Gazebo window. Use it when the Mac is loaded: an
overloaded CPU makes Nav2 miss its lifecycle heartbeats and reset mid-run.

## Real robot

Bring up the robot as in the earlier labs, then:

### Map

```zsh
scripts/launch/mapping.zsh        # terminal 1
scripts/teleop.zsh                # terminal 2: drive the whole maze
scripts/save.zsh                  # terminal 3, while mapping still runs
colcon build --packages-select team85_lab5   # install the new map
```

The map frame's origin is where the robot was when mapping started, facing +x.

### Navigate (grading day)

1. `scripts/launch/navigation.zsh`
2. In RViz, set the robot's pose with **2D Pose Estimate**, and check that the
   laser scan lines up with the map walls.
3. Get the 3 goal points with **Publish Point** and
   `ros2 topic echo /clicked_point`, and write them into
   `src/team85_lab5/wayPoints.txt`, one `x y` per line (map frame, meters).
4. `scripts/launch/waypoints.zsh`

The navigator drives to each point in order, pauses 2 s at each, and prints a
summary with the distance it ended from each one. If Nav2 aborts a waypoint,
it sends it once more, then skips it. Ctrl-C cancels the goal and stops the
robot.

### Recovery candidates

`turtlebot3_recovery_lab/config/recovery_candidates.yaml` lists safe points in
the map frame, picked with Publish Point. Use open spots spread over the maze.

## Simulation

The 7785 maze from
[Lab5_Gazebo](https://github.gatech.edu/7785-Intro-to-Robotics-Research/Lab5_Gazebo)
is installed in `turtlebot3_gazebo` (`7785_maze.launch.py`).

```zsh
scripts/launch/mapping.zsh --sim          # map it: teleop.zsh --sim, save.zsh --sim
scripts/launch/navigation.zsh --sim waypoints:=true
```

Navigation in sim uses `maps/map_sim.yaml`, `recovery_candidates_sim.yaml` and
`wayPoints_sim.txt`. To force recoveries, send a goal off the map:

```zsh
source src/team85_lab5/scripts/sim_env.zsh
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 50.0, y: 50.0}, orientation: {w: 1.0}}}}"
```

The planner fails, recovery runs 3 times, and the goal ends `ABORTED`, as
expected. Gazebo has no buzzer, so `Cannot beep: /sound service unavailable`
is normal there.

## How recovery works

When Nav2 fails, the behavior tree calls `/divert_robot` on
`recovery_student.py`, which:

1. Plays the activation sound (ERROR).
2. Plans a path to the nearby candidates with Nav2's planner and picks the
   shortest real path. It skips candidates closer than `min_distance` (0.5 m)
   and any candidate that already failed for the current goal: if the robot
   gets stuck again after diverting to a candidate, that one is excluded until
   the next goal.
3. Returns it. The behavior tree drives there, then retries the original goal,
   up to 3 recoveries per goal.
4. Plays the success sound (ON) when the robot reaches the candidate. If the
   recovery is abandoned first (another recovery, the goal ends, or 60 s),
   there's no success sound.

The buzzer plays one sound at a time, and a new request cuts off the sound
playing. Each lasts almost 2 s, so the node spaces sounds 2.5 s apart
(`min_beep_gap`).

For grading, the block must be removed after the robot reaches the recovery
point. If it stays, the goal aborts after the third recovery.

## Changes to the instructor's package

`turtlebot3_recovery_lab`, so they're in the submission zip.

| File | Change | Why |
|---|---|---|
| `CMakeLists.txt` | `install(PROGRAMS ...)` installs `recovery_student.py`, not `recovery.py` | `recovery.py` isn't in the zip, so the build fails. **Affects every platform.** |
| `CMakeLists.txt` | `-Wl,-dead_strip_dylibs` under `if(APPLE)` | macOS only: otherwise `bt_navigator` can't load the BT plugin (`symbol not found in flat namespace '_PyBool_FromLong'`) |
| `recovery_student.py` | The recovery logic above; executable bit set | The lab's TODOs |
| `behavior_trees/navigate_w_recovery_divert.xml` | `server_timeout="5000"` on `BeepAndDivertBridge` | Planning to the candidates takes longer than the default 200 ms |
| `config/nav2_params.yaml` | Tuning, below | |
| `config/recovery_candidates*.yaml` | New | Candidate points, real map and sim |
| `package.xml` | Run dependencies of `recovery_student.py` | |

### Nav2 tuning (`nav2_params.yaml`)

- **Controller: Regulated Pure Pursuit** instead of DWB, which got stuck in
  the maze's small rooms. It turns in place toward the path when it's more than
  about 26° off (`rotate_to_heading_min_angle`), and slows down in tight turns.
  The old DWB block is kept as `FollowPathDWB`, unused.
- **Goal checker: position only** (`PositionGoalChecker`, 0.15 m). The
  waypoints have no heading, and turning in place at the goal tripped the
  progress checker.
- **Progress checker: `PoseProgressChecker`,** so turning in place counts as
  progress.
- **Planner tolerance 0.15 m.** If a goal is in an obstacle, NavFn plans to the
  nearest free cell within this distance, so a larger value lets Nav2 report
  "reached" far from the goal.
- **`default_server_timeout` 200 ms** (was 20), so the planner isn't timed out
  when the Mac is busy.
- **Inflation** is small (`inflation_radius` 0.15–0.2 m) for the narrow maze.
  Paths can run close to walls; raise it if the robot scrapes corners.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Could not load library ...typesupport...dylib`, or `bt_navigator` can't load the BT plugin | macOS (SIP) drops `DYLD_LIBRARY_PATH` when a program starts through `/usr/bin/env` or `/bin/zsh` | Use the scripts, which source the setup inside, and start Python nodes with `python <path>` |
| `"/divert_robot" service server not available`, then signal 11 | Recovery node not running when Nav2 started | Use `launch/navigation.zsh`, or start `recovery.zsh` first |
| Nav2 resets mid-run: `CRITICAL FAILURE` from the lifecycle manager (heartbeat) | Mac overloaded, usually by the Gazebo window | `gui:=false`, close other heavy apps |
| No beep, and `ros2 service call /sound ...` gets no response | Different DDS middleware from the robot | `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |
| Only one of two sounds plays | The buzzer cuts off a sound when a new one starts | Keep `min_beep_gap` ≥ 2.5 s |
| A recovery picks a point the robot is standing on | `min_distance` too small | Keep it well above `arrival_tolerance` (0.25 m); the node warns at startup |
| Robot swings left and right when the path is behind it | Turn-in-place and pure pursuit fighting | Lower `rotate_to_heading_min_angle`, raise `min_lookahead_dist` |
| Gazebo hangs on exit, or the next launch fails (`gzserver` exit 255) | `gzserver` ignores SIGTERM, or is still shutting down | `pkill -9 gzserver gzclient`, then launch again |
| `pytest` crashes with `Plugin 'launch_testing' ...` | The `launch_testing` pytest plugin doesn't work with this pytest | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test` |

## What has been tested

On the real robot:

- Navigation with Regulated Pure Pursuit on the saved map
- Recovery with a physical block: activation sound, diversion to a candidate,
  success sound on arrival, retry of the goal
- Both sounds audible with the 2.5 s spacing

In the Gazebo maze:

- Mapping, navigation, and the waypoint navigator (3/3 waypoints from one
  command, ending within 0.15 m)
- Recovery: path-aware candidate choice, exclusion of a failed candidate,
  success only on arrival, no success after a canceled goal or a newer recovery
- Ctrl-C on the waypoint navigator stops the robot

Not tested: the full grading run (3 waypoints and a recovery in one go) on the
real robot.
