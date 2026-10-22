# team85_lab4

AE7785 Lab 4 (team 85): a TurtleBot3 Burger drives through a list of waypoints while avoiding obstacles, using ROS 2 Humble.

The robot uses only odometry and the 2D LiDAR. Each scan is segmented to find the closest obstacle on the way to the current goal. A state machine decides when to avoid it, and a virtual target planner blends go-to-goal, follow-wall and avoid-obstacle behaviors into a heading and distance reference. A separate P controller node turns that reference into velocity commands.

- [Packages](#packages)
- [Build and run](#build-and-run)
- [Architecture](#architecture)
- [Algorithm](#algorithm)
  - [1. Scan processing](#1-scan-processing)
  - [2. Goals](#2-goals)
  - [3. Avoidance state](#3-avoidance-state)
  - [4. Virtual target planner](#4-virtual-target-planner)
  - [5. Low-level controller](#5-low-level-controller)
- [Parameters](#parameters)
- [Debug tools](#debug-tools)
- [Tests](#tests)
- [Notes](#notes)

---

## Packages

| Package | Build type | Contents |
|---|---|---|
| `team85_lab4` | ament_python | Nodes, launch files, parameter files, waypoints |
| `team85_lab4_interfaces` | ament_cmake | Custom messages (a Python package cannot define messages) |

Both go in the `src/` directory of a colcon workspace.

```
lab_4/
├── README.md
├── team85_lab4/
│   ├── config/
│   │   ├── robot.yaml            # Robot node parameters (and shared ones)
│   │   └── debug.yaml            # Debug node parameters
│   ├── launch/
│   │   ├── robot.launch.py       # Nodes that run with the robot
│   │   └── debug.launch.py       # Visualization windows (host only)
│   ├── team85_lab4/
│   │   ├── scan_segmentation.py  # ScanSegmenter
│   │   ├── obstacle_detector.py  # ObstacleDetector
│   │   ├── path_zone.py          # Path zone geometry, /goal_vector listener
│   │   ├── verbose_logger.py     # Logging gated by the verbose parameter
│   │   └── scripts/              # One file per node
│   ├── test/                     # Unit tests
│   └── wayPoints.txt             # Waypoints, "x y" per line, in the world frame
└── team85_lab4_interfaces/
    └── msg/                      # ScanSegments, Segment, AvoidanceState,
                                  # GoalStatus, VirtualTarget
```

---

## Build and run

From the workspace root, in the `humble` environment:

```zsh
colcon build --packages-select team85_lab4_interfaces team85_lab4
source install/setup.zsh
```

Rebuild `team85_lab4_interfaces` after changing a `.msg` file, and `team85_lab4` after changing `setup.py` or adding files. With the symlink install, the installed Python code, configs, and launch files link to the sources, so editing them only needs a node restart.

```zsh
# Robot nodes
ros2 launch team85_lab4 robot.launch.py
ros2 launch team85_lab4 robot.launch.py verbose:=1 params_file:=/path/to/robot.yaml

# Visualization (host)
ros2 launch team85_lab4 debug.launch.py
```

| Launch argument | Default | Meaning |
|---|---|---|
| `verbose` | `2` (robot), `1` (debug) | `0`: no logging, `1`: one-off messages (startup, state changes), `2`: also continuous results |
| `params_file` | `config/robot.yaml` / `config/debug.yaml` | Parameter file of the launched nodes |
| `robot_params_file` | `config/robot.yaml` | (debug only) Source of the shared parameters, e.g. `path_radius` |

`verbose` can also be changed on a running node: `ros2 param set /goal verbose 0`.

---

## Architecture

```text
                 /scan                                    /odom
                   │                                        │
                   ▼                                        ▼
          ┌────────────────┐   /goal_vector    ┌────────────────┐
          │ scan_processor │◄──────────────────│      goal      │
          └────────────────┘         │         └────────────────┘
   /obstacle_position │              │           │ /goal  │ /goal_status
   /obstacle_normal   ▼              ▼           │        │
          ┌────────────────┐                     │        │
          │ obstacle_state │                     │        │
          └────────────────┘                     │        │
          /avoidance_state │                     ▼        ▼
                           │              ┌────────────────┐
                           └─────────────►│   go_to_goal   │◄── /odom
                                          └────────────────┘◄── /pause
                                         /virtual_target │     (avoidance_monitor)
                                                         ▼
                                             ┌──────────────────────┐
                                             │ low_level_controller │
                                             └──────────────────────┘
                                                /cmd_vel │
                                                         ▼
                                                     TurtleBot3
```

### Nodes

| Node | Subscribes | Publishes | Role |
|---|---|---|---|
| `scan_processor` | `/scan`, `/goal_vector` | `/scan_segments`, `/obstacle_position`, `/obstacle_normal` | Segments each scan; finds the closest point in the path zone and its surface normal |
| `obstacle_state` | `/obstacle_position`, `/obstacle_normal`, `/goal_vector` | `/avoidance_state` | Avoidance state machine and wall-following vector (20 Hz) |
| `goal` | `/odom` | `/goal`, `/goal_vector`, `/goal_status` | Defines the world frame, tracks and switches goals; serves `/reset_goals` |
| `go_to_goal` | `/odom`, `/goal`, `/goal_status`, `/avoidance_state`, `/pause` | `/virtual_target` | Virtual target planner (20 Hz) |
| `low_level_controller` | `/virtual_target` | `/cmd_vel` | P controller, run on each virtual target |
| `view_scan` | `/scan`, `/scan_segments`, `/obstacle_*`, `/goal_vector` | | Debug: scan, segments, obstacle |
| `goal_monitor` | `/odom`, `/goal_status` | | Debug: map, trajectory, goal progress; `c` calls `/reset_goals` |
| `avoidance_monitor` | `/avoidance_state`, `/virtual_target`, `/scan_segments`, `/pause` | `/pause` | Debug: avoidance criteria and virtual target; space pauses / resumes |

`scan_processor` publishes nothing on the obstacle topics when no obstacle is in the path zone. `obstacle_state` and `go_to_goal` treat inputs older than their timeouts as missing, and `low_level_controller` stops the robot when no virtual target arrives within `target_timeout`.

### Topics and messages

| Topic | Type | Frame | Content |
|---|---|---|---|
| `/scan_segments` | `ScanSegments` | Robot | Segments (scan indices and points); unsegmented points are noise |
| `/obstacle_position` | `geometry_msgs/PointStamped` | Robot | Closest point in the path zone |
| `/obstacle_normal` | `geometry_msgs/Vector3Stamped` | Robot | Its fitted surface normal, toward the robot |
| `/goal` | `geometry_msgs/PointStamped` | Odom | Current goal |
| `/goal_vector` | `geometry_msgs/Vector3Stamped` | Robot | Vector from the robot to the current goal |
| `/goal_status` | `GoalStatus` | Odom / world | World origin, waypoints, goal index, finished flag, distance, hold progress |
| `/avoidance_state` | `AvoidanceState` | Robot | Avoiding flag, wall vector and side, obstacle and normal used, path zone, thresholds |
| `/virtual_target` | `VirtualTarget` | Robot | $\theta_{\mathrm{VT}}$, $d_{\mathrm{VT}}$, status, planner diagnostics |
| `/reset_goals` | `std_srvs/Trigger` (service) | | Back to goal 1, with the world origin set again from the next odometry |
| `/pause` | `std_msgs/Bool` | | `true` pauses `go_to_goal`, `false` resumes it. Transient local, so a restarted `go_to_goal` gets the last value |
| `/cmd_vel` | `geometry_msgs/Twist` | Robot | Velocity command |

The robot frame and the LiDAR frame are identical: $x$ forward, $y$ left, angles counterclockwise from $+x$.

---

## Algorithm

### 1. Scan processing

**Segmentation** (`ScanSegmenter`). Two neighboring scan points belong to the same segment if both are valid and their distance is within an adaptive threshold that grows with range:

$$
\|\mathbf q_i-\mathbf q_{i-1}\|\le b+k\,\min(r_i,r_{i-1})\,|\Delta\theta|
$$

where $b$ is `base_threshold` and $k$ is `k`. Segments with fewer than `min_points` points are dropped as noise. For a full 360° scan, the last and first points are joined, so an object crossing angle 0 stays one segment.

**Path zone.** The obstacle that matters is the one on the way to the goal. With the goal vector $\mathbf g$ in the robot frame (from `/goal_vector`), the path direction is $\hat{\mathbf u}=\mathbf g/\|\mathbf g\|$ and the path length $L=\|\mathbf g\|$. A point $\mathbf q$ has path coordinates

$$
s=\mathbf q^\top\hat{\mathbf u}\ \ (\text{along}),\qquad
\ell=\hat u_x q_y-\hat u_y q_x\ \ (\text{lateral, positive left})
$$

and lies in the path zone if

$$
0<s<L\quad\text{and}\quad|\ell|<r_{\mathrm{path}}
$$

with $r_{\mathrm{path}}$ = `path_radius`. The zone is the strip from the robot to the goal, so it doesn't swing away from an obstacle as the robot turns, and a wall beyond the goal is ignored. Without a goal vector newer than `goal_vector_timeout` (before the first goal, after the last, or at the goal), the zone falls back to the robot heading with $L=\infty$.

**Closest obstacle** (`ObstacleDetector`). The closest segment point in the path zone is the obstacle point $\mathbf p_o$. Its surface normal is fitted by PCA, as the eigenvector of the smallest eigenvalue of the covariance of its neighbors on the same segment, oriented toward the robot. The neighbors are the contiguous run of points within `normal_radius` along the segment, extended to at least `min_normal_points`. They may lie outside the path zone, since they describe the same surface. At a segment end the neighbors come from one side only, which still gives the surface direction.

### 2. Goals

`goal` reads `wayPoints.txt` (world frame). The first odometry pose $(\mathbf p_0,\psi_0)$ defines the world frame: $x$ forward and $y$ left of the robot at start. Calling `/reset_goals` (`c` in `goal_monitor`) returns to the first goal and takes the next odometry pose as the new $(\mathbf p_0,\psi_0)$, so the waypoints are laid out again from where the robot stands. Each waypoint $\mathbf w_i$ is converted once into the odom frame:

$$
\mathbf w_i^{\mathrm{odom}}=R(\psi_0)\,\mathbf w_i+\mathbf p_0
$$

On every odometry reading $(\mathbf p,\psi)$, the goal vector in the robot frame is

$$
\mathbf g=R(\psi)^\top\left(\mathbf w_{\mathrm{current}}^{\mathrm{odom}}-\mathbf p\right)
$$

A goal is reached when the robot stays within `goal_tolerance` of it for `hold_time` seconds; leaving the tolerance resets the timer. `goal` then switches to the next waypoint. After the last one it sets `finished` in `/goal_status` and stops publishing `/goal` and `/goal_vector`.

### 3. Avoidance state

`obstacle_state` has two states, **avoiding** and **goal following**. It avoids while the obstacle point is both

- closer than `avoid_distance`: $\|\mathbf p_o\|<d_{\mathrm{avoid}}$, and
- in the path zone (section 1),

and returns to goal following as soon as either condition fails. Readings older than `obstacle_timeout` count as no obstacle. Since $|\ell|\le\|\mathbf p_o\|$, the lateral bound only matters while `path_radius` < `avoid_distance`.

**Wall normal.** The wall-following geometry uses one normal $\mathbf n$, selected by `wall_normal`:

- `fitted`: the PCA surface normal from `scan_processor`.
- `radial`: the direction from the obstacle point to the robot, $-\mathbf p_o/\|\mathbf p_o\|$.

**Wall vector and side.** The wall vector is the normal rotated by 90°:

$$
\mathbf t=\pm R(90^\circ)\,\mathbf n,\qquad R(90^\circ)\,\mathbf n=(-n_y,\ n_x)
$$

The side is chosen when avoidance starts, as the rotation pointing toward the goal: counterclockwise (`ccw`) if $\left(R(90^\circ)\mathbf n\right)^\top\hat{\mathbf u}\ge0$, clockwise otherwise. It stays fixed until avoidance ends, while $\mathbf t$ follows the current normal.

`/avoidance_state` carries the flag, $\mathbf t$, `ccw`, and the obstacle point, normal, path zone and thresholds the decision used.

### 4. Virtual target planner

`go_to_goal` combines three behaviors into a robot-relative **virtual heading** $\theta_{\mathrm{VT}}$ (positive left) and a signed **virtual distance** $d_{\mathrm{VT}}$:

- **Go-to-Goal (GTG):** steers and drives toward the goal.
- **Follow-Wall (FW):** turns the robot away when it points toward the wall, and drives it along the wall.
- **Avoid-Obstacle (AO):** reduces forward motion near obstacles.

FW and AO are both enabled while `obstacle_state` is avoiding.

All quantities are in the robot frame, so the robot is at the origin with heading $\mathbf h=(1,0)$. The goal is $\mathbf p_g=\mathbf g$, the obstacle $\mathbf p_o$, the wall tangent $\mathbf t$ and the wall normal $\mathbf n$ come from `/avoidance_state`. Angle differences are wrapped to $[-\pi,\pi)$.

#### Geometry

$$
d_{\mathrm{GTG}}=\|\mathbf p_g\|,\qquad
\theta_{\mathrm{GTG}}=\operatorname{wrap}\left(0-\operatorname{atan2}(p_{g,y},p_{g,x})\right)
$$

A goal on the left gives $\theta_{\mathrm{GTG}}<0$.

$$
d_{\mathrm{AO}}=\|\mathbf p_o\|,\qquad
\hat{\mathbf a}=-\mathbf p_o/d_{\mathrm{AO}}\ \ (\text{from the obstacle toward the robot})
$$

$$
\theta_{\mathrm{FW}}=\operatorname{wrap}\left(0-\operatorname{atan2}(t_y,t_x)\right),\qquad
\sigma=\operatorname{sgn}\left(t_xn_y-t_yn_x\right),\qquad
z=\sigma\,\theta_{\mathrm{FW}}
$$

$\sigma=+1$ means a counterclockwise correction (wall on the right, `ccw` false); $\sigma=-1$ a clockwise one (wall on the left, `ccw` true). $z<0$ exactly when the robot points toward the wall, for either side. Because $\mathbf t$ is built from $\mathbf n$, $\left|t_xn_y-t_yn_x\right|=1$.

#### Heading: GTG + FW

GTG steering:

$$
g=-k_{\mathrm G}^{\theta}\,\theta_{\mathrm{GTG}}
$$

FW steering, scaled by a proximity weight that saturates at the danger distance $l_d$:

$$
w_{\mathrm{FW}}=e^{-s_{\mathrm{FW}}^{d}\max(0,\ d_{\mathrm{AO}}-l_d)},\qquad
u_{\mathrm{FW}}^{\theta}=\sigma\,k_{\mathrm{FW}}\,w_{\mathrm{FW}}\left[e^{s_{\mathrm{FW}}\max(0,-z)}-1\right]
$$

$u_{\mathrm{FW}}^{\theta}$ is zero for $z\ge0$, has sign $\sigma$ for $z<0$, and is continuous at $z=0$.

GTG steering that opposes FW is suppressed, and restored over $\delta$ once the robot has turned past the wall direction:

$$
g_\sigma=\sigma g,\qquad
\gamma=\operatorname{clip}\left(\frac{z}{\delta},0,1\right),\qquad
u_{\mathrm G}^{\theta}=\sigma\left[\max(g_\sigma,0)+\gamma\min(g_\sigma,0)\right]
$$

| Condition | FW steering | Opposing GTG steering |
|---|---|---|
| $z<0$ (toward the wall) | Away from the wall | Suppressed |
| $0\le z<\delta$ | Zero | Restored gradually |
| $z\ge\delta$ | Zero | Fully restored |

GTG steering that helps FW is always kept. Without FW, $u_{\mathrm G}^{\theta}=g$ and $u_{\mathrm{FW}}^{\theta}=0$. The virtual heading is

$$
\theta_{\mathrm{VT}}=\tan^{-1}\left(u_{\mathrm G}^{\theta}+u_{\mathrm{FW}}^{\theta}\right)\in\left(-\tfrac{\pi}{2},\tfrac{\pi}{2}\right)
$$

For any $z<0$, the total steering has sign $\sigma$: GTG cannot steer the robot back into the wall, at any distance (a far obstacle only makes the turn slower).

#### Distance: GTG + AO + FW

Each contribution has a magnitude and is projected onto the heading.

$$
\tilde u_{\mathrm G}^{d}=\operatorname{clip}\left(k_{\mathrm G}^{d}d_{\mathrm{GTG}},\ u_{\mathrm G}^{\min},\ u_{\mathrm G}^{\max}\right)\cdot\frac{\mathbf p_g^\top\mathbf h}{d_{\mathrm{GTG}}}
$$

$$
\tilde u_{\mathrm{AO}}^{d}=u_{\mathrm{AO}}^{\max}\,e^{-s_{\mathrm{AO}}\max(0,\ d_{\mathrm{AO}}-l_d)}\cdot\hat{\mathbf a}^\top\mathbf h
$$

$$
\tilde u_{\mathrm{FW}}^{d}=u_{\mathrm{FW}}^{\mathrm{const}}\cdot\mathbf t^\top\mathbf h
$$

- **GTG:** the lower bound keeps the robot moving near the goal. The projection is zero with the goal to the side and negative with the goal behind.
- **AO:** saturates within $l_d$. It is fully backward for an obstacle straight ahead, zero beside the robot, and forward for one behind.
- **FW:** keeps the robot moving along the wall when the goal is nearly perpendicular to the heading. It is not scaled by $w_{\mathrm{FW}}$.

AO and FW terms are zero while disabled. The sum is saturated smoothly and clamped:

$$
d_{\mathrm{VT,raw}}=L_d\tan^{-1}\left(\tilde u_{\mathrm G}^{d}+\tilde u_{\mathrm{AO}}^{d}+\tilde u_{\mathrm{FW}}^{d}\right),\qquad
d_{\mathrm{VT}}=\operatorname{clip}\left(d_{\mathrm{VT,raw}},\ d_{\min},\ d_{\max}\right)
$$

with $d_{\min}=-d_{\max}$ in signed mode (`forward_only: false`, so the robot can back up) and $d_{\min}=0$ in forward-only mode. $d_{\mathrm{VT}}$ is a longitudinal reference along the current heading, not a point to drive to.

#### Status

The output is zeroed with a status explaining why:

| Status | Condition |
|---|---|
| `OK` | Valid target |
| `PAUSED` | Paused on `/pause` (space in `avoidance_monitor`) |
| `GOAL_REACHED` | $d_{\mathrm{GTG}}\le$ `goal_tolerance` (the hold timer runs while stopped) |
| `FINISHED` | All goals reached (`/goal_status`) |
| `STALE_INPUT` | Odometry, goal, or avoidance state older than `input_timeout` |
| `NONFINITE_INPUT` | An input is not finite |
| `MISSING_OBSTACLE`, `INVALID_OBSTACLE_DIRECTION` | Avoiding without a usable obstacle point |
| `MISSING_TANGENT`, `INVALID_TANGENT`, `INVALID_WALL_NORMAL`, `DEGENERATE_WALL_GEOMETRY` | Avoiding without usable wall geometry |

`/virtual_target` also carries the diagnostics: each contribution, the cosines, $\sigma$, $z$, $\gamma$, $w_{\mathrm{FW}}$, and $d_{\mathrm{VT,raw}}$.

### 5. Low-level controller

The `low_level_controller` node is a P controller, after Lab 3's `chase_object`. It maps each virtual target to velocities as it arrives (20 Hz):

$$
v=\operatorname{clip}\left(k_v\,d_{\mathrm{VT}},\ \pm v_{\max}\right),\qquad
\omega=\operatorname{clip}\left(k_\omega\,\theta_{\mathrm{VT}},\ \pm\omega_{\max}\right)
$$

Each term is zero inside its deadband (`distance_deadband`, `angle_deadband`). $\theta_{\mathrm{VT}}$ is positive to the left like ROS `angular.z`, and a negative $d_{\mathrm{VT}}$ backs the robot up. The heading is controlled even when $d_{\mathrm{VT}}=0$ (turning in place). Any status other than `OK` stops the robot. A zero command is also sent once when no virtual target has arrived for `target_timeout` (so `teleop_keyboard` is not overridden while `go_to_goal` is down), and on shutdown.

---

## Parameters

Robot parameters are in `config/robot.yaml`, with units and comments; debug parameters are in `config/debug.yaml`. Floats need a decimal point (`2.0`, not `2`).

| Node | Parameters |
|---|---|
| All (`/**`) | `path_radius`, `goal_vector_timeout`, `goal_tolerance` |
| `scan_processor` | `min_points`, `base_threshold`, `k`, `normal_radius`, `min_normal_points` |
| `obstacle_state` | `obstacle_timeout`, `avoid_distance`, `wall_normal` |
| `goal` | `hold_time`, `waypoints_file` |
| `go_to_goal`, planner | `k_gtg_theta`, `k_fw`, `s_fw`, `s_fw_distance`, `delta`, `k_gtg_distance`, `u_gtg_min`, `u_gtg_max`, `u_ao_max`, `s_ao`, `l_d`, `u_fw_distance`, `distance_scale`, `max_reference_distance`, `forward_only`, `geometry_epsilon`, `min_tangent_cross`, `max_fw_exponent` |
| `go_to_goal`, inputs | `input_timeout` |
| `low_level_controller` | `target_timeout`, `kp_linear`, `kp_angular`, `distance_deadband`, `angle_deadband`, `max_linear_speed`, `max_angular_speed` |

Planner symbols:

| Parameter | Symbol | Meaning | Constraint |
|---|---|---|---|
| `k_gtg_theta` | $k_{\mathrm G}^{\theta}$ | GTG heading gain | $>0$ |
| `k_fw` | $k_{\mathrm{FW}}$ | FW steering gain | $>0$ |
| `s_fw` | $s_{\mathrm{FW}}$ | FW exponential sensitivity | $>0$ |
| `s_fw_distance` | $s_{\mathrm{FW}}^{d}$ | FW proximity decay (1/m) | $\ge0$ ($0$: no scaling) |
| `delta` | $\delta$ | GTG recovery width (rad) | $(0,\pi)$ |
| `k_gtg_distance` | $k_{\mathrm G}^{d}$ | GTG distance gain (1/m) | $>0$ |
| `u_gtg_min`, `u_gtg_max` | $u_{\mathrm G}^{\min},u_{\mathrm G}^{\max}$ | GTG attraction bounds | $0\le u_{\mathrm G}^{\min}\le u_{\mathrm G}^{\max}$ |
| `u_ao_max` | $u_{\mathrm{AO}}^{\max}$ | Maximum AO repulsion | $>0$ |
| `s_ao` | $s_{\mathrm{AO}}$ | AO decay (1/m) | $>0$ |
| `l_d` | $l_d$ | Danger distance, robot center to surface (m) | $>0$ |
| `u_fw_distance` | $u_{\mathrm{FW}}^{\mathrm{const}}$ | FW drive along the wall | $\ge0$ |
| `distance_scale` | $L_d$ | Arctangent distance scale (m) | $>0$ |
| `max_reference_distance` | $d_{\max}$ | Maximum absolute virtual distance (m) | $>0$ |
| `min_tangent_cross` | | Minimum $\left|t_xn_y-t_yn_x\right|$ | $(0,1)$ |

**Tuning notes:**
- **How strongly FW turns:** $\theta_{\mathrm{VT}}$ saturates through the arctangent, and $\omega$ at `max_angular_speed`. Keep $k_{\mathrm{FW}}$ small enough that FW steering stays near or below 1 where avoidance starts, or every FW command looks the same. `s_fw_distance` makes the turn gentler while the obstacle is still far.
- **Moving along a wall:** if the robot barely moves while following a wall, raise `u_fw_distance` relative to `u_ao_max`.
- **Deadband vs. goal tolerance:** keep `distance_deadband` below $d_{\mathrm{VT}}$ at the goal tolerance, or the robot stops short of the goal.

---

## Debug tools

`debug.launch.py` opens three OpenCV windows. Each closes with `q`, Esc, the close button, or Ctrl+C.

| Window | Shows |
|---|---|
| `view_scan` | LiDAR scan around the robot (up: $+x$). Shows segments in color while `/scan_segments` arrives, otherwise the raw scan in gray. Also draws the path zone, the obstacle point, and its normal. |
| `goal_monitor` | World-frame map: waypoints (green: reached, yellow: current, gray: upcoming) with tolerance circles, the trajectory, the robot, and the goal vector. The panel shows the goal index, distance, hold timer, robot pose, and goal vector. `c` clears the trajectory and calls `/reset_goals`: back to goal 1, with the world origin moved to the robot's current pose (e.g. after carrying the robot back to the start). |
| `avoidance_monitor` | Robot-frame view: path zone and avoid distance, obstacle and normal (white), wall direction (magenta), and virtual target (cyan: heading ray, plus the signed distance as a point on the robot's $x$ axis). The panel shows the state, each criterion against its threshold, and the planner diagnostics. Space toggles `/pause`: the robot stops (status `PAUSED`) until space is pressed again. |

### Offline tuning: `tune_planner`

```zsh
ros2 run team85_lab4 tune_planner                         # starts from the installed robot.yaml
ros2 run team85_lab4 tune_planner --params /path/to/robot.yaml
```

A matplotlib window, not a ROS node, so no robot is needed. It plots the planner and the low-level controller, with a slider for every `go_to_goal` and `low_level_controller` parameter. The curves come from the same `VirtualTargetPlanner` and `LowLevelController` classes the robot runs, so what you see is what the robot computes.

**Controls** (right side):
- **Planner and controller sliders:** start from `robot.yaml`.
- **Scene sliders:** obstacle distance, goal angle, goal distance, and the $z$ used by plot 4.
- **Toggles:** wall side (right: $\sigma=+1$, left: $\sigma=-1$) and `forward_only`.
- **Reset** returns to the `robot.yaml` values.
- **Print YAML** prints the current values, ready to paste under `go_to_goal` and `low_level_controller` in `robot.yaml` (`goal_tolerance` goes in the shared `/**` section).
- **Invalid combinations** (e.g. `u_gtg_min` > `u_gtg_max`) show the error in the title, and the plots keep their last valid state.

**Scene.** Plots 1 to 4 describe the robot during avoidance, with FW and AO enabled:

- The robot is at the origin, heading $+x$.
- A straight wall runs at the obstacle distance $d_{\mathrm{AO}}$, on the robot's right or left.
- The wall tangent points in the wall-following direction.
- The goal is at the goal distance, at the goal angle from the tangent toward the wall: $0^\circ$ means along the wall, $90^\circ$ straight behind it, and negative angles away from the wall.

The horizontal axis of plots 1 to 3 is the FW error $z$: the robot's heading relative to the tangent, signed so that

- $z<0$: the robot points into the wall,
- $z=0$: it is parallel to the wall,
- $z>0$: it is turned away from the wall.

A robot entering avoidance head-on starts near $z=-\pi/2\approx-1.6$, and wall following should bring it to $z\approx0$.

| Plot | Horizontal axis | Curves |
|---|---|---|
| 1. Heading terms | $z$ | $u_{\mathrm G}^{\theta}$ (after suppression), $u_{\mathrm{FW}}^{\theta}$, and their sum $u^\theta$ |
| 2. Virtual heading and $\omega$ | $z$ | $\theta_{\mathrm{VT}}$ (deg, left axis); $\omega$ (rad/s, right axis), dashed at $\pm$`max_angular_speed` |
| 3. Distance terms and $v$ | $z$ | Projected GTG, AO and FW distance terms and their sum $u^d$ (left); $v$ (m/s, right), dashed at $\pm$`max_linear_speed` |
| 4. Proximity terms | $d_{\mathrm{AO}}$ | $w_{\mathrm{FW}}$ and $u_{\mathrm{AO}}^d/u_{\mathrm{AO}}^{\max}$ (left); $\omega$ and $v$ (right), at the $z$ slider |
| 5. GTG only, goal ahead | Goal distance | $u_{\mathrm G}^{d}$, $d_{\mathrm{VT}}$ (left); $v$ (right) |
| 6. GTG only, goal around | Goal bearing | $\theta_{\mathrm{VT}}$ (deg, left); $\omega$ and $v$ (right) |

**1. Heading terms vs $z$.** This shows how FW and GTG share the steering.
- **$u_{\mathrm{FW}}^{\theta}$** is zero for $z\ge0$ and grows exponentially as the robot turns further into the wall. `k_fw` sets its size, and `s_fw` how fast it grows. The proximity weight scales it, so moving the obstacle distance slider beyond `l_d` shrinks it.
- **$u_{\mathrm G}^{\theta}$** is flat at zero wherever GTG would steer into the wall (suppression). Past $z=0$ it ramps back over `delta`.
- **$u^\theta$** must have the sign of the FW correction for all $z<0$. That is the priority guarantee.
- **Tuning:** a steep $u^\theta$ near $z=0$ makes the robot snap parallel and overshoot. A wider `delta` softens how GTG pulls the robot back toward the wall after it has turned away. The vertical axis is clipped at $\pm10$.

**2. Virtual heading and $\omega$ vs $z$.** This shows what the robot actually does with the heading.
- **$\theta_{\mathrm{VT}}=\tan^{-1}(u^\theta)$** saturates toward $\pm90^\circ$.
- **$\omega$** is $\theta_{\mathrm{VT}}$ through `kp_angular`, zero inside `angle_deadband`, and clipped at `max_angular_speed`.
- **Flat parts of $\omega$** are where the robot turns at its limit. Parameter changes there have no effect, which is why lowering a large `k_fw` may not visibly change anything.
- **The slope of $\omega$ around $z=0$** decides how smoothly the robot settles parallel to the wall. Too steep, and it oscillates in and out of alignment.

**3. Distance terms and $v$ vs $z$.** This shows whether the robot moves forward along the wall. Each term is its magnitude times its cosine with the heading:
- **GTG** is largest when the robot faces the goal.
- **AO** is negative while the robot faces the wall, and zero when parallel to it.
- **FW** is largest when aligned with the tangent, and negative when pointing against it.

The robot backs up where $v<0$ and stalls where the sum crosses zero. Around $z\approx0$, $v$ should be clearly positive; if it isn't, the robot barely moves while following the wall. Raise `u_fw_distance`, or lower `u_ao_max`. `distance_scale`, `max_reference_distance` and `kp_linear` map the sum to $v$; `forward_only` cuts $v$ at 0.

**4. Proximity terms vs $d_{\mathrm{AO}}$.** This shows how behavior changes as the robot approaches the wall, at the $z$ chosen by its slider.
- **$w_{\mathrm{FW}}$** (FW steering strength) and **$u_{\mathrm{AO}}^d/u_{\mathrm{AO}}^{\max}$** (AO repulsion strength) both equal 1 inside `l_d` (dashed line) and decay beyond it, at rates `s_fw_distance` and `s_ao`.
- **$\omega$ and $v$** show the resulting turn rate and speed. The dotted line marks the scene's obstacle distance.
- **Where avoidance starts:** `obstacle_state` begins avoiding at `avoid_distance`. Read the curves there to see how gently the robot turns when avoidance starts.

**5. GTG only: vs goal distance (goal straight ahead).** This shows the approach to a goal without obstacles.
- **$u_{\mathrm G}^{d}$** is $k_{\mathrm G}^{d}\,d_{\mathrm{GTG}}$ clipped between `u_gtg_min` and `u_gtg_max`.
- **$d_{\mathrm{VT}}$ and $v$** follow from it. Inside `goal_tolerance` (dashed) everything is zero (`GOAL_REACHED`).
- **Speed limit:** the distance where $v$ reaches its limit shows how early the robot starts slowing down.
- **Final approach:** the floor set by `u_gtg_min` is the speed of the final approach. It must stay above `distance_deadband`, or the robot stops short of the goal.

**6. GTG only: vs goal bearing.** This shows turning toward a goal without obstacles, at the goal distance slider.
- **$\theta_{\mathrm{VT}}$ and $\omega$** show how fast the robot turns toward a goal at each bearing; positive means the goal is on the left. A wide flat region of $\omega$ means the turn rate is limited by `max_angular_speed` for most bearings.
- **$v$** falls with the cosine of the bearing: zero with the goal to the side, negative (backing up) with the goal behind in signed mode.
- **The jump at $\pm180^\circ$** comes from the angle wrap: for a goal straight behind, the turn direction is arbitrary.

---

## Tests

```zsh
cd src/team85_lab4
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_virtual_target.py test/test_low_level_controller.py test/test_path_zone.py
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` is needed because the ROS `launch_testing` pytest plugins in the environment are incompatible with the installed pytest. The tests cover:

- the planner: GTG, FW and AO behaviors, continuity, priority, saturation, invalid inputs, and frame invariance;
- the controller: signs, deadbands, saturation, and stopping;
- the path zone: direction, capping, and the avoidance decision and wall side.

---

## Notes

- **One `/cmd_vel` publisher.** `low_level_controller` publishes `/cmd_vel` continuously while `go_to_goal` runs. Stop `teleop_keyboard` before running: it also publishes every 0.1 s, even without key presses, and the robot then follows whichever command arrived last.
- **Restart after changes.** Running nodes keep the code and parameters they started with.
- **Separate robots.** Use a distinct `ROS_DOMAIN_ID` per robot and host pair. `goal` takes its world origin from the first `/odom` it receives (or the first after `/reset_goals`).
- **Logging.** Warnings and errors always print; `verbose` only gates info messages.
