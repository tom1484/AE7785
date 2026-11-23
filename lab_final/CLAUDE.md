# turtlebot3_ws — current project: AE7785 final (follow signs through a maze)

This workspace holds every lab (team85_lab3..5 etc.); **active work is the final**:
`src/team85_final`, `src/team85_final_interfaces`, `src/team85_final_recovery`.
Handout: `~/Documents/lab6_final.pdf` (also `~/Workspace/Courses/AE7785/lab_6/provided/`).
Demo **2026-12-07**, one 20-minute slot, no late days, no maze time on demo day.
Env traps (RoboStack/macOS, SIP, Cartographer, DDS): the user's memory notes and
`src/team85_lab5/README.md` — they all apply here.

## Machines
- **Primary: this laptop** (user `tom1484`, `~/Workspace/turtlebot3_ws`) - it drives the real
  robot. Moved back from the Mac mini `home` on 2026-11-20; the mini copy (user `tomchen`,
  same layout, own `humble` env + zsh config) is now frozen: edit only here. Remote-only
  path rewrites there (`colcon_defaults.yaml`, `env_setup.md`, `.vscode`, yolo `data.yaml`)
  must never be copied back. To move again: dry-run `rsync -anic` both ways first (exclude
  build/ install/ log/ .venv/), copy only real changes.
- Env `humble` matches the lockfile `humble.explicit.osx-arm64.txt` (894 pkgs) on both machines.
- ROS always runs in zsh (`~/.zshrc`: ROS_DOMAIN_ID=85, burger, `conda activate humble`,
  `install/local_setup.zsh`, `~/.config/ros/tailscale-env.bash` = FastDDS over Tailscale,
  whitelisting this laptop's Tailscale IP 100.66.9.62, robot peer 100.74.118.59).
- Mini access if needed: login shell fish and `ssh home` forces tmux+fish; scripted:
  `ssh -o RemoteCommand=none -o RequestTTY=no home bash -s`, ROS inside `/bin/zsh -i -c`.

## The task in one paragraph
3x6 grid maze, fixed walls (mapped once, a priori). On demo day only start, goal and signs
change. At each wall the sign says: left/right = turn 90° and drive to the next wall,
stop/do-not-enter = turn around and drive to the next wall, goal (star) = done.
Grade (Manhattan distances in cells): reached → `1 − |r − n|·0.01 − 0.1·c`, where n = sign
path length, r = driven length, c = 1 if a collision or the robot never *signals* the goal.
**|r − n| penalizes shortcuts exactly like detours.**

## Packages
| Package | Build | Contents |
|---|---|---|
| `team85_final` | ament_python | maze model, detector, planners, mission executive, launch, config, sim world |
| `team85_final_interfaces` | ament_cmake | `SignDetection.msg`, `SignDetectionArray.msg` (no vision_msgs in the env) |
| `team85_final_recovery` | ament_cmake | Nav2 recovery: renamed copy of Lab 5's instructor `turtlebot3_recovery_lab` (BT plugin `team85_final_divert_bridge_plugin`, `DivertRobot.srv`, `recovery_server.py`, divert BT). Do not use the original package. |

## Layout (`src/team85_final/`)
Core modules are **rclpy-free** and unit-tested; nodes are thin wrappers.
| Path | What |
|---|---|
| `team85_final/maze/geometry.py` | `Heading` (E/N/W/S = yaw quadrants, left = +1), `Cell(row, col)`, `CellPose` = robot in a cell facing a heading = **a wall face** when blocked |
| `team85_final/maze/grid.py` | `MazeGrid`: walls, `drive_to_wall`, `path_length` (BFS), maze↔map frame (`origin`, `yaw`), `locate`, ASCII `render` |
| `team85_final/maze/extract.py` + `nodes/extract_maze.py` | walls from a saved SLAM map, given the grid geometry |
| `team85_final/maze/scan_match.py` | ICP of LIDAR scans onto the saved map (distance-transform nearest wall, multi-start); used by `set_initial_pose` |
| `team85_final/maze/sign_locator.py` | camera pose (robot + offset) + bearing → `raycast` to the first wall face of the known maze; `lidar_agrees` check |
| `team85_final/vision/signs.py` | `SignClass` (ids = Lab 6 labels) and the turn each sign commands |
| `team85_final/vision/detector.py` | YOLO-detect ONNX via cv2.dnn (letterbox, NMS, bearing) |
| `team85_final/planning/base.py` | **Planner interface**: `reset / observe(SignObservation) / next_action(pose) → GoTo / Inspect / Finish / GiveUp` |
| `team85_final/planning/registry.py` | `@register('name')`, `make_planner(name, grid)`; picked by the `planner` param |
| `team85_final/planning/grid_planner.py` | 3.1: drive to wall, inspect, obey; EMPTY → inspect the other walls of the cell in place |
| `team85_final/planning/sign_graph.py` | `SignGraph`: nodes = wall faces, edges derived from map + sign beliefs (votes; inspection > passive) |
| `team85_final/planning/graph_planner.py` | 3.2: sign following + graph for reliability (strategies 1-5 in its docstring) + optional shortcuts |
| `team85_final/planning/explore_planner.py` | `explore`: random wall-face targets, builds a SignGraph from passive sightings only (graph test tool) |
| `team85_final/planning/simulate.py` | offline run of any planner on a sign layout, handout grading incl. partial credit; `fan_sightings` = passive model |
| `team85_final/planning/scenarios.py` + `montecarlo.py` | random valid sign layouts + sensing-error model; paired Monte Carlo planner comparison (how every planner change is judged) |
| `team85_final/nodes/mission_node.py` | executive: TF pose → planner → Nav2 → ALIGN (square up) → inspection (ray must hit the faced wall) / passive sightings / goal beeps on `/sound` / RViz markers / graph dump |
| `team85_final/nodes/sign_detector_node.py` | camera → `/sign_detections` (+ debug image) |
| `team85_final/nodes/plot_graph.py` | graph dump → PNG on the saved map (sign + sightings per face, turn arrows) for checking by eye |
| `team85_final/nodes/mapping_supervisor.py` | runs Cartographer; RESET MAP button (RViz interactive marker) / `/reset_map` restarts it |
| `config/final.yaml` | node params · `config/nav2_params.yaml` Nav2 (from Lab 5, see below) |
| `config/maze.yaml`, `maze_sim.yaml` | grid geometry + walls; `maze_sim.yaml` done (matches the Gazebo mesh), `maze.yaml` = the real maze (fitted from `maps/map`); also the recovery candidates |
| `launch/` | `sim` (Gazebo only), `mapping`, `navigation` (recovery server → 3 s → Nav2), `final` (all take `sim:=true`) |
| `sim/` | Lab 6 Gazebo world with signs + sign models (see `sim/README.md`); `gazebo_truth.py` (walls), `sign_truth.py` (36 signs as faces; `--compare graph.yaml`) |
| `scripts/` | zsh wrappers as in Lab 5 (`--sim` = domain 77, localhost); `save.zsh` writes `maps/` here |
| `models/` | `signs.onnx` goes here (from lab_final_vision) |
| `docs/graph_planner.md` | graph planner explained: every strategy, the Monte Carlo setup, results + per-strategy ablations, how to reproduce |
| `docs/mission_node.md` | mission node walkthrough: state machine, detections (inspect votes + passive), Nav2 seq ids, ALIGN, with snippets |
| `runs/` (workspace root) | real-robot logs, graph dumps and plots per session |
| `copy_to_repo.sh` | mirror the 3 packages + this file to `~/Git/AE7785/lab_final` (no commit) |

## Decisions (and why)
- **Own map.** Lab 5's real maze is a different layout; the final records `maps/map.yaml`
  (real) and `maps/map_sim.yaml` (Gazebo) with `scripts/launch/mapping.zsh` + `save.zsh`.
  The Gazebo walls are byte-identical to Lab 5's sim maze (only signs/robot differ).
- **Nav2 params copied from Lab 5** (tuned there), pointed at `team85_final_recovery`'s BT and
  plugin. `/divert_robot` must be up before bt_navigator, so navigation.launch.py starts
  the recovery server first.
- **Recovery candidates = maze cell centers.** `recovery_server.py` accepts a team85_final
  maze yaml as `candidates_file` (min_distance = half a cell); Lab 5 style lists still work.
- **Maze = grid of wall faces.** A known map makes a sign's edge deterministic
  (`drive_to_wall(turned pose)`), so the graph stores beliefs per face, not edges.
- **Planner decides, executive acts.** Inspection is an action the planner asks for, so
  the graph planner can skip it where it already knows the sign.
- **Detector sees all signs** (YOLO detect, not cls) so the graph planner can learn signs
  in passing. The grid planner only uses `Inspect` readings.
- Runtime inference = cv2.dnn (cv2 4.13 in the humble env, no onnxruntime there).
- **Sim world vendored** into `team85_final/sim` (sign models via GAZEBO_MODEL_PATH in
  sim.launch.py) instead of copying files into turtlebot3_gazebo / ~/.gazebo.
- **Graph planner is for reliability, not speed.** The grade has no time term and |r − n|
  punishes shortcuts like detours; the points are lost to misreads and getting lost. Every
  rule in `graph_planner.py` was kept only if the Monte Carlo improved (rejected: finishing
  at a wall the signs lead to twice that still reads EMPTY - it turned give-ups into
  wrong-wall finishes). Mean grade, 1000 trials, seed 1:
  | scenario | grid | graph | graph no-shortcuts |
  |---|---|---|---|
  | nominal errors | 0.979 | 0.996 | 0.995 |
  | harsh errors | 0.942 | 0.984 | 0.978 |
  | 10% signs missing | 0.984 | 0.999 | 0.997 |
  | no passive sightings | 0.979 | 0.976 | 0.975 |
  The graph only helps if passive sightings work (in Gazebo they were 36/36 correct).
  Ablations (one strategy off each, `docs/graph_planner.md` §6.2): the `max_visits` cap matters
  most (off: harsh −0.021, runs wander to the action limit); `distrust_goal` halves wrong-wall
  finishes; `passive_cap` and re-reads add a few tenths each; `reinspect_conf` alone shows no
  measurable effect (re-reads are driven by passive disagreement) - unproven, harmless.
- **Sighting-only mode** `planner_params:={inspect: false}` (graph planner, for show): no
  inspections - the sign in front is the firm passive belief (one Inspect only if nothing is
  known, and `confirm_goal` reads a sighted GOAL once before finishing); shortcuts take any
  known chain and the mission **retargets mid-drive** (`maybe_retarget` -> `planner.retarget`)
  when sightings extend the chain; no ALIGN on arrival. Monte Carlo real maze, 500, nominal:
  0.993 vs graph 0.996, 1.4 vs 10 inspections; harsh 0.975 vs 0.983; without confirm_goal
  0.985 / 0.960 (wrong-wall finishes). Default stays inspect: true.
  Why shortcuts rarely fired in inspect mode: the camera sees walls ahead, so the wall after a
  turn is unknown until arrival and known chains are short; mid-drive retarget fixes that.
- Mission Nav2 goals carry a sequence id: results of replaced goals (aborted on preempt) are
  ignored.
- **Shortcuts ON** (`allow_shortcuts`) by the user's choice - for fun, not score; the user
  will tell the TA about the formula. Don't turn them off as an "optimization".
- **Passive evidence is capped at one inspection** when deciding the faced wall: hundreds of
  far frames would otherwise outvote close-up reads (far stop vs goal is the detector's
  weak spot). Inspected EMPTY + firm passive sign → obey passive (a miss is likelier).
- **Passive placement by ray through the known maze** (camera pose + bearing → first wall),
  LIDAR only as a check; replaced "LIDAR range at the bearing" (wrong origin, no walls known).
- **ALIGN in the mission node, not Nav2 tolerance alone.** Nav2 reports success while its
  velocity smoother is still ramping a turn down (~15 deg overshoot after "arrival"), and
  can "succeed" in-place turns without turning. Goal checker yaw 0.05 rad + ALIGN to 2 deg.
- **"In front" = the detection's camera ray hits the inspected face**, not a bearing window:
  up close and 13 cm off-center the front sign is 23 deg off-axis.
- **Global costmap = static + inflation (0.45 m, scaling 2.5).** LIDAR marks smeared by
  rotation closed corridors; narrow inflation made plans hug corners and RPP stop
  ("collision ahead"). Walls are fixed and mapped; the local costmap still marks obstacles.
- **Mapping config `config/cartographer/maze_lds_2d.lua`**: loop-closure search limited to
  0.3 m / 10 deg (stock 7 m / 30 deg). On the real robot, stock accepted false matches
  ~1.1 m away (look-alike cells, score 0.65-0.75) -> pose "teleports", smeared map.
  Loop closure fully off was tried in sim and drifted. `mapping.launch.py config:=stock`
  for the original. Cartographer runs under `nodes/mapping_supervisor.py`: RESET MAP
  button in RViz (interactive marker) / `ros2 service call /reset_map std_srvs/srv/Trigger`
  restarts it (Cartographer has no clear-map service); the new map starts at the robot.
- **Detector debug image on by default** (`publish_debug_image: true`): RViz navigation.rviz
  "SignDetections" panel shows the camera with boxes on the robot too (some laptop CPU).
- **Bearings from `/camera_info`** (sign_detector `camera_info_topic`), not `hfov`: the robot's
  Pi camera is calibrated (fx 250.8, cx 166.3, k1 0.26 → 65 deg HFOV, 1-2.4 deg off the old
  pinhole guess); `hfov` is only the fallback. Sim uses `/simulated_camera/camera_info`.
- **`start:=row,col,heading`** on navigation/final launch sets AMCL's pose at that cell's
  center from the maze yaml - how the TA states the demo start. RViz 2D Pose Estimate still works.
- **Initial pose = given cell + ICP + 360 deg spin** (Nav2 guide's advice). `set_initial_pose`
  ICPs 5 scans onto `maps/map` from `start:=r,c,h`, accepts only >= 80% points on walls and
  <= 35 cm / 35 deg correction (real data: right cell 100%, wrong cell/heading <= 59%),
  publishes /initialpose, spins 360 deg on odometry, then latches `/localization_ready`;
  the mission (`wait_localized`, set by final.launch) waits for it. Real test 2026-11-23 at
  (0,5)N: ICP moved 10 cm, after spin AMCL = ICP within 3.6 cm / 0.4 deg.
  `refine_pose:=false` / `spin:=false` to skip.
- Launch arg named `final_params_file`, not `params_file` (gzserver and Nav2 bringup use that
  name and launch configs are global).

## Current state (2026-11-20)
- Sim: everything works end to end in Gazebo (laptop and mini) - grid and graph planners reach
  the goal reading every wall 5/5; graph building from passive sightings 36/36 correct;
  Cartographer maps. Gates green (unit tests `src/team85_final/test`).
- Vision done: `models/signs.onnx` (YOLO11n @416, all 591 hand-labeled images; held-out
  P 0.938 R 0.923, 0 left/right swaps; far stop vs goal is the weak spot). Retraining:
  `~/Workspace/Courses/AE7785/lab_final_vision/.ai/PROJECT.md`.
- Real robot (2026-11-20): grid run from (0,0)E reached the goal (4 cells = sign path, every
  wall 5/5, heading within ±1.8 deg, beeps heard).
- Real full traversal (2026-11-23, explore 12 targets from (0,5)N, `runs/2026-11-23_real/`):
  passive sightings gave a firm sign on **all 34 wall faces** (near-unanimous votes); every
  sign chain leads to the goal (2,0)W with no loop; agrees with all earlier inspections and the
  2026-11-20 partial run; the user checked the plot against the real signs: all correct
  -> passive sightings pass the demo planner gate. One Nav2
  stall (laptop overloaded: planner 5.9 Hz, compute_path ack timeout) -> recovery to the
  nearest cell center worked. Arrivals 12-15 cm from center, heading within ±1.5 deg.
- **Real graph run** (2026-11-23, from (2,5)E, `graph_run1.log`): goal reached following the
  10-sign path exactly (r = n), all 11 inspections correct at conf 1.00, ~3 min, no recovery,
  no shortcut (none shorter by >= min_shortcut on this layout from there). First attempt
  crashed: passive sightings during the init spin reached GraphPlanner.observe before
  reset() -> mission now buffers them (`early_obs`) and replays after reset (99 replayed).
- **Real sighting-only run** (2026-11-23, `inspect: false`, from (0,5)N, `graph_sighting1.log`):
  goal reached in ~100 s (start -> DONE) vs ~172 s for the inspecting graph run on a similar
  10-sign route; 8 mid-drive retargets, zero stops before the goal, one confirming read
  (GOAL 1.00), no recovery. First retarget skipped the (2,5)N DNE bounce (shortcut, r < n).
- Real maze mapped (2026-11-20): `maps/map.*` clean, `config/maze.yaml` fitted from it (8
  interior walls, cell 0.925 m, every edge scored 1.00/0.00). Graph beats grid on this layout
  too (Monte Carlo 500, nominal 0.996 vs 0.979, harsh 0.983 vs 0.930). Final code not yet
  run on the robot.
- Graph planner write-up done (`src/team85_final/docs/graph_planner.md`, numbers re-run
  2026-11-20 on current code). `final.yaml` defaults to `planner: grid`; graph is chosen at launch.
- `src/cartographer_ros` carries 5 local patches (uncommitted in its upstream clone, by
  design); the 5th (absl log libs) was needed because a clean build failed to link.
- Versioning: the workspace isn't git. `copy_to_repo.sh` mirrors to `~/Git/AE7785/lab_final`,
  branch `final` (off main, one branch per lab, pushed to origin; first commit 2026-11-23).
  To publish: run it, review `git status`, commit + push there.

## Next actions
1. User checks `runs/2026-11-23_real/graph_explore.png` against the real signs (plot:
   `python -m team85_final.nodes.plot_graph --map maps/map.yaml --maze config/maze.yaml
   --graph ... --out ...`). If it is right, passive sightings pass the demo planner gate.
   Laptop load (detector + Nav2 + RViz) caused one Nav2 stall: measure CPU if it recurs.
2. Then `final.zsh planner:=graph start:=0,0,E graph_dump:=...`. Demo planner gate: if passive
   sightings on the real robot are as good as in sim, demo with graph; else `planner:=grid`
   (graph < grid without passive). Camera mount angle: estimate from inspection logs if
   bearings show a constant bias. camera_offset_x 0.025 is an eyeball estimate.
3. (optional) Sim start-pose option to watch shortcuts/recovery fire in Gazebo; sim
   fine-tune of the detector. Submission zip `LastName1_LastName2_Final.zip` at the end.

## Commands
```zsh
conda activate humble; cd ~/Workspace/turtlebot3_ws
colcon build --packages-select team85_final_interfaces team85_final_recovery team85_final
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest src/team85_final/test -q   # colcon test is broken here
src/team85_final/scripts/launch/final.zsh --sim planner:=grid
src/team85_final/copy_to_repo.sh -n      # then without -n, commit in ~/Git/AE7785
cd src/team85_final && python -m team85_final.planning.montecarlo --maze config/maze_sim.yaml \
    -n 1000 --seed 1 --planners grid graph "graph:allow_shortcuts=false" --errors nominal
python -I src/team85_final/sim/sign_truth.py --compare /tmp/graph.yaml  # graph_dump vs Gazebo truth
```

## Traps
- Nodes run with `prefix=sys.executable` (SIP drops DYLD_LIBRARY_PATH via /usr/bin/env).
- `ros2 topic echo` of workspace msg types (SignDetectionArray) prints nothing on macOS
  (CLI shebang loses DYLD paths); read them with a `python` rclpy subscriber.
- Agent shells need `ROS_DOMAIN_ID=85` + `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` for the
  real robot's services (`/sound`). Agent shells: source `install/setup.zsh` (not .bash), or use `/bin/zsh -i` (zshrc does it).
- Cartographer's map origin is not necessarily the robot's start pose (map_sim: start at
  ~(-0.37, -0.03)). Sim initial pose comes from maze_sim.yaml `spawn`, not (0, 0, 0).
- Driving for mapping: turn slowly (<= 0.4 rad/s); 1 rad/s at 5 Hz scans smeared walls.
- A map that "teleports" while recording = false loop closure in the look-alike maze; check
  `~/.ros/log/cartographer_node_*.log` for `constraint_builder_2d.cc` matches ~1 m off.
- Speed tests (2026-11-23, `runs/2026-11-23_real/speed/`, nav2_L1-3.yaml): RPP flips between
  rotate-in-place and driving at speed (no hysteresis; rotate_to_heading_min_angle 0.45) ->
  raise it to 0.9 rad + longer lookahead. Fast in-place turns at the goal oscillate around a
  tight yaw tolerance ("Failed to make progress") -> yaw_goal_tolerance ~0.25 when the mission
  ALIGNs anyway. Inflation 0.33 + fast/long lookahead -> "collision ahead" stops at corners and
  run 5 (L3) ended pinned ~5 cm from a wall (RPP can't reverse, recovery loops): keep 0.38.
- **Fast profile** `fast:=true` -> `config/nav2_params_fast.yaml` (v2: level-2 rotation 1.8 rad/s,
  rotate_to_heading_min_angle 0.9, yaw tol 0.25, base lookahead, cost-regulated slow-down near
  walls). v2 real runs from (0,5)N, 0 stalls: graph 104 s (run 1, base-ish L1: 167 s),
  sighting-only 61 s (L1: 75 s). v1 (lookahead 0.45-0.55) ended against walls twice, but the
  motor link was flaky then, so a misbehaving wheel may share the blame.
  Equal-length route ties are decided by inflation cost: (0,5)/(0,4) centers are only 0.37 m
  from walls (inside the 0.38 inflation) vs >= 0.42 on the northern route, so a shortcut
  (2,5)->(1,4) goes north via (2,3),(1,3) - same 4 cells, not a longer path. Fixes (user's
  design): inflation 0.35 (both nav2 files) and rolling shortcuts `max_skip: 1` +
  `retarget_within: 1.0` cell (docs/graph_planner.md 4.10). Real run 11 (graph, fast, from
  (0,5)N): 128 s, 0 stalls, every shortcut 1 wall and on the sign route (south via (0,4)W).
  Real run 12 (sighting-only, fast, rolling, from (0,5)N, `run12_sight_roll.log`): 67 s
  start -> DONE, 0 stalls, 10 one-wall retargets all on the sign route (DNE bounce at (2,5)N
  kept, unlike graph_sighting1), one confirming read GOAL 1.00. (An earlier try,
  `run12_sight_roll_motorfail.*`, died to a turtlebot3_node crash, not the planner.)
  Multi-start sighting-only fast runs (`runs/2026-11-23_real/starts/`, scratchpad
  `chain_starts.sh`): s1 from (2,3)W 73 s, 0 stalls, 12 one-wall retargets, GOAL 1.00.
  s2 (2,2)S 73 s, s3 (2,5)E 57 s, s4 (0,4)S 49 s; all 0 stalls, every retarget one wall on
  the sign route, GOAL 1.00 each; drives between starts all succeeded. Robot left at (0,5)N.
  Only blemish: arrival at the goal (2,0)W is ~14 deg off (no align) -> mission aligns 2-5 s
  before the confirming read.
- Robot hardware: after a battery swap the LIDAR stopped publishing mid-run and turtlebot3_node
  kept dying until a full robot restart. Later the wheels stopped obeying /cmd_vel while /odom
  and /scan still published (init spin "timed out after 0 deg", buzzer "Failed transmit
  instruction packet" = OpenCR -> Dynamixel link). Symptom: TF "extrapolation into the future", buzzer
  "no status packet". Check `/scan` and `/odom` publish (not just node list) before a run.
  **Torque off after a restart** (2026-11-23, after the run-12 crash + battery swap): bringup
  clean, /odom + /scan fine, but wheels don't move; `/sensor_state` shows `torque: false`.
  Fix: `ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"`. Robot shell:
  `ssh turtlebot` (bringup in tmux session 0, `bash -ic` for ROS env).
- Mission/planners: nothing may reach `planner.observe` before `planner.reset` (graph planner
  state is created in reset); the mission buffers pre-START sightings.
- Killing a launch mid init-spin: set_initial_pose now stops the robot (own SIGINT handling);
  the robot may be left off-heading -> check with ICP before relaunching (> 35 deg = rejected).
- Nav2 rejects NavigateToPose for a few s after startup (lifecycle): the mission now waits for
  `/bt_navigator/get_state` = active before its first goal.
- Real robot: if Nav2 stalls with "Transform data too old ... map to odom", check `ros2 node list`
  for `/turtlebot3_node` + `/diff_drive_controller` - the motor driver died (2026-11-20, mid
  explore run; camera/lidar kept running). Restart bringup, check battery. Recovery can't help.
- Recovery needs no waypoint list: candidates = maze.yaml cell centers, nearest by planned path.
- Nav2 "succeeded" != robot stopped/square: check heading after the robot is still (ALIGN).
- Don't send Nav2 goals from a debug script while the mission runs: it preempts the
  mission's goal (status 6). Debug inspection with `--log-level mission:=debug`.
- gzserver ignores SIGTERM: `pkill -9 -f gzserver` after sim runs.
- Conventions in `maze/geometry.py` (row = +y, col = +x, left = +1) are shared by every
  module, the maze yaml, and `maze_cell_centers` in recovery_server.py.
- `fliplr` must stay 0 when training the detector.
- Remote shells are fish: `$?`, `&&` chains with bash syntax and globs fail there; pipe
  scripts to `bash -s`, run ROS in `/bin/zsh -i -c`.
- A clean rebuild is the real test: an old build dir hid a Cartographer link error.
