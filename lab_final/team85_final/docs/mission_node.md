# `mission_node.py` explained

Code: `team85_final/nodes/mission_node.py` (577 lines). Line numbers below refer to that file
as of 2026-11-23. Snippets are trimmed (`...`); read the file for the full code.

Contents
1. The big picture
2. Setup (`__init__`)
3. Inputs: scan, localization, Nav2 readiness, robot pose
4. Sign detections: inspection votes and passive sightings
5. Mid-drive retarget and the planner log
6. The state machine: `step()` and `step_inspect()`
7. Nav2: sending goals and handling results
8. ALIGN: squaring up with `/cmd_vel`
9. Finish, beeps, `main()`
10. RViz markers and the graph dump
11. One full cycle, traced

---

## 1. The big picture

The mission node is the **executive**. It never decides *where* to go; the **planner** does
that. The node runs one loop:

```
ask the planner "what now?"  ->  do it  ->  tell the planner what was seen  ->  ask again
```

The planner (`planning/base.py`) answers with one of four actions:

| Planner returns | Node state | What the node does |
|---|---|---|
| `GoTo(face)` | `NAVIGATE` | Nav2 drives to that cell's center, facing that heading |
| (after Nav2 arrives) | `ALIGN` | turn in place until the heading is within 2 deg |
| `Inspect` | `INSPECT` | stand still, vote over ~5 detection frames of the wall ahead |
| `Finish` / `GiveUp` | `DONE` | stop; beep 3 times on the goal |

The states and how they connect:

```
            Nav2 active + localized
   START ─────────────────────────────> ALIGN ──> PLAN
                                                   │
          ┌────────────────────────────────────────┤ planner.next_action(pose)
          │ GoTo                 Inspect           │ Finish / GiveUp
          v                         v              v
      NAVIGATE                   INSPECT          DONE
          │ Nav2 succeeded          │ enough frames
          │   inspect mode:  ALIGN ─┘ -> PLAN
          │   sighting-only: PLAN
          │ Nav2 failed: retry once, then PLAN
          └─ (retarget mid-drive: new goal, stays NAVIGATE)
```

`PLAN` is the hub. Every action ends by going back to `PLAN`, which reads the robot pose and
asks the planner again.

**Second channel, outside the state machine.** Every camera detection, not only those during
`INSPECT`, is placed on a wall of the known maze (camera ray -> first wall) and sent to the
planner as a **passive** sighting. The grid planner ignores these; the graph planner builds
its sign graph from them.

**How it runs.** Nothing blocks. Callbacks (detections, scan, Nav2 replies) only store data
or switch state. A 10 Hz timer calls `step()`, which does a little work for the current state
and returns.

---

## 2. Setup (`__init__`, lines 62-140)

### Parameters (64-93)

```python
('planner', 'grid'),            # which planner: grid / graph / explore
('maze_file', 'config/maze.yaml'),
('inspect_frames', 5),          # frames to vote over
('inspect_timeout', 3.0),       # s; vote with what we have after this
('inspect_min_area', 0.02),     # ignore boxes smaller than 2% of the image
('inspect_settle', 0.3),        # s after stopping before frames count
('planner_params', ''),         # yaml passed to the planner, e.g. "{inspect: false}"
('graph_dump', ''),             # write the planner's graph here at the end
('camera_offset_x', 0.069),     # camera position on the robot (passive sightings)
('passive_max_range', 2.5),     # m; farther signs get misclassified
('passive_max_lateral', 0.3),   # x cell size; hits near a corner are ambiguous
('passive_edge_px', 3),         # drop boxes cut off by the image border
('lidar_tolerance', 0.25),      # m; LIDAR must see a wall where the ray hits
('nav_retries', 1),
('align_tol', 0.035),           # rad (2 deg)
('align_max_turn', 0.4), ('align_min_turn', 0.08), ('align_still', 0.01), ('align_timeout', 12.0),
('goal_sound', 1), ('goal_beeps', 3), ('beep_gap', 2.5),
('wait_localized', False),      # wait for set_initial_pose's /localization_ready
```

Real values come from `config/final.yaml` and launch arguments; these are only defaults.

### Building the planner (95-99)

```python
self.grid = MazeGrid.from_yaml(share_path(self.cfg['maze_file']))
planner_params = yaml.safe_load(self.cfg['planner_params'] or '{}') or {}
self.planner = make_planner(self.cfg['planner'], self.grid, **planner_params)
```

`planner_params` is a YAML string turned into keyword arguments. That is how
`planner_params:={inspect: false}` on the command line becomes `GraphPlanner(inspect=False)`.

### Connections (101-114)

```python
self.tf = Buffer(); self.tf_listener = TransformListener(self.tf, self)   # robot pose
self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')           # Nav2
self.bt_state = self.create_client(GetState, '/bt_navigator/get_state')     # is Nav2 active?
self.create_subscription(LaserScan, '/scan', self.on_scan, ...)
self.create_subscription(SignDetectionArray, '/sign_detections', self.on_detections, 10)
self.localized = not self.cfg['wait_localized']
self.create_subscription(Bool, '/localization_ready', self.on_localized,
                         QoSProfile(..., durability=TRANSIENT_LOCAL))
```

`/localization_ready` is **latched** (TRANSIENT_LOCAL): a late subscriber still gets the last
message. If `wait_localized` is false, `localized` starts as True and the wait is skipped.

### Run-time state (116-138)

| Variable | Used by | Meaning |
|---|---|---|
| `state` | all | `START`, `PLAN`, `NAVIGATE`, `ALIGN`, `INSPECT`, `DONE` |
| `early_obs` | START | passive sightings that arrive before `planner.reset()` |
| `inspect_votes`, `inspect_frames_seen`, `inspect_started`, `inspect_pose` | INSPECT | the vote being collected and which wall face is being read |
| `nav_target`, `nav_attempts` | NAVIGATE | current goal (a `CellPose`) and tries so far |
| `nav_seq` | NAVIGATE | id of the newest Nav2 goal; replies about older goals are ignored |
| `aligned` | PLAN | robot is squared up at the current face |
| `align_started`, `align_last_yaw`, `align_active` | ALIGN | timing and "has the robot stopped yet" |
| `readings` | RViz | inspected results per face |
| `beeps_left`, `beep_timer` | DONE | goal signal |

### Timers (139-140)

```python
self.create_timer(0.1, self.step)              # the state machine, 10 Hz
self.create_timer(0.5, self.publish_markers)   # RViz, 2 Hz
```

---

## 3. Inputs (lines 143-192)

### Scan and localization flag

```python
def on_scan(self, msg):
    self.scan = msg                      # only the latest scan is kept

def on_localized(self, msg):
    if msg.data and not self.localized:
        self.localized = True            # START checks this flag
```

### Is Nav2 ready? (`navigator_active`, 151-164)

The `navigate_to_pose` action server exists before Nav2 can accept goals. Goals sent too early
were rejected (15 rejections in 1.5 s on the real robot). So the node asks the bt_navigator
lifecycle node for its state, **without blocking**:

```python
def navigator_active(self):
    if self.bt_active:
        return True                                   # active: stop asking (reset to False
                                                      # after a rejected/unanswered goal, §7)
    f = self.bt_state_future
    if f is not None and f.done():                    # an earlier request has answered
        self.bt_active = f.result().current_state.id == State.PRIMARY_STATE_ACTIVE
        self.bt_state_future = None
    if not self.bt_active and self.bt_state_future is None and self.bt_state.service_is_ready():
        self.bt_state_future = self.bt_state.call_async(GetState.Request())   # ask again
    return self.bt_active
```

This is the general pattern for calling a service from a timer: send the request
(`call_async`), keep the future, and check `future.done()` on later ticks. Waiting on the
future inside a callback would deadlock the single-threaded executor.

### Robot pose (`robot_pose`, 166-184)

```python
def robot_pose(self, stamp=None, frame=None):
    frame = frame or self.cfg['base_frame']
    try:
        try:
            t = self.tf.lookup_transform('map', frame, Time.from_msg(stamp) if stamp else Time())
        except TransformException:
            if stamp is None: raise
            t = self.tf.lookup_transform('map', frame, Time())     # stamp too old/new: use latest
    except TransformException:
        return None
    ...
    return x, y, yaw
```

- With no `stamp`, it gives the latest pose (`Time()` means "latest available").
- With a `stamp` (an image or scan time), it gives the pose **when that data was taken**.
  This matters for passive sightings: the robot may have turned 10 deg since the image was
  captured.
- If TF has no data for that stamp (robot clock skew, buffer too short), it falls back to the
  latest pose rather than dropping the data.
- `frame` can be another frame, e.g. the LIDAR's frame, to get the scan origin.

### LIDAR range at a bearing (`range_at`, 186-192)

```python
i = round((bearing - s.angle_min) / s.angle_increment) % len(s.ranges)
r = s.ranges[i]
return r if s.range_min < r < s.range_max else None
```

This converts an angle to a scan index. Only used by the LIDAR check on passive sightings
(`lidar_agrees`).

---

## 4. Sign detections (lines 194-259)

`on_detections` runs for **every** detector message (about the camera frame rate). It does two
independent things.

### 4a. Inspection votes (only in state INSPECT, 211-226)

```python
if self.state == 'INSPECT' and now >= self.inspect_started + self.cfg['inspect_settle']:
    self.inspect_frames_seen += 1
    front = [d for d in msg.detections
             if self.on_inspected_wall(d, msg.header.stamp)                     # on the faced wall
             and d.width * d.height / area >= self.cfg['inspect_min_area']]     # big enough
    if front:
        best = max(front, key=lambda d: d.width * d.height)
        self.inspect_votes.append((SignClass(best.sign), best.confidence))
```

- Frames in the first 0.3 s are skipped: the robot may still be moving after ALIGN. The check
  uses receipt time, not the image stamp, because the robot's clock can be skewed.
- **Every frame counts** in `inspect_frames_seen`, even with no sign in front. A frame with
  nothing in front is effectively a vote for EMPTY (see the confidence in section 6).
- If several boxes qualify, the largest wins.

**Which detection is "in front"?** (`on_inspected_wall`, 194-207)

```python
pose = self.robot_pose(stamp)
if pose is None:
    return abs(d.bearing) <= self.cfg['inspect_center_tol']      # fallback: bearing window
cx, cy, cyaw = camera_pose(*pose, self.cfg['camera_offset_x'], self.cfg['camera_offset_y'])
hit = raycast(self.grid, cx, cy, cyaw + d.bearing)               # camera ray -> first wall face
return hit is not None and hit.face == self.inspect_pose
```

It shoots a ray from the camera at the detection's bearing through the known maze and checks
that the first wall it hits is the face being inspected. A plain "bearing near 0" test failed:
close to the wall and slightly off-center, the sign in front can be 25 deg off-axis, while a
neighbouring wall's sign looks nearly centered.

### 4b. Passive sightings (every message, 227-259)

```python
pose = self.robot_pose(msg.header.stamp)          # robot pose when the image was taken
cx, cy, cyaw = camera_pose(*pose, ...)
scan_pose = self.robot_pose(self.scan.header.stamp, self.scan.header.frame_id)   # LIDAR origin
for d in msg.detections:
    if box touches the image's left/right edge:            continue   # sign cut off
    hit = raycast(self.grid, cx, cy, cyaw + d.bearing, self.cfg['passive_max_range'])
    if hit is None or abs(hit.lateral) > 0.3 * cell_size:  continue   # too far / near a corner
    if not lidar_agrees(hit, *scan_pose, self.range_at, 0.25): continue  # LIDAR sees no wall there
    obs = SignObservation(hit.face, SignClass(d.sign), d.confidence, Source.PASSIVE, time.time())
    if self.state == 'START':
        self.early_obs.append(obs)                 # planner not reset yet: keep for later
        continue
    if self.planner.observe(obs):                  # True = the graph changed
        log 'graph: face = SIGN -> successor'
        self.maybe_retarget()
```

The four filters, in order:

1. **Image edge:** a half-visible sign is often misclassified.
2. **Range:** the ray stops at `passive_max_range` (2.5 m); farther signs are unreliable.
3. **Lateral:** `hit.lateral` is how far from the face center the ray lands. Near a corner,
   the ray could belong to either wall.
4. **LIDAR:** the LIDAR must measure a wall about where the ray hits. This catches pose errors:
   if the pose is off, the ray hits the wrong wall and the LIDAR distance won't match.

`planner.observe()` returns True when the graph's belief for that face changed. Only then is
it logged, and only then can a retarget happen.

**`early_obs`:** during START the robot spins in place for localization and sees many signs.
The graph planner creates its graph in `reset()`, which runs later. Calling `observe()` before
that crashed (`'passive'` AttributeError), so these sightings are buffered and replayed right
after `reset()`.

---

## 5. Mid-drive retarget and the planner log (lines 261-282)

```python
def maybe_retarget(self):
    retarget = getattr(self.planner, 'retarget', None)      # only the graph planner has it
    if retarget is None or self.state != 'NAVIGATE' or self.nav_target is None:
        return
    pose = self.robot_pose()
    new = retarget(self.nav_target, pose[:2] if pose is not None else None)
    if new is None:
        return
    self.flush_planner_log()
    log 'retarget mid-drive: old -> new'
    self.nav_attempts = 0
    self.send_nav(new)                                       # replaces the running Nav2 goal
```

`planner.retarget()` returns None in the normal case. It returns a new target only when all of
these hold:

- the planner allows it (sighting-only mode, shortcuts on);
- the robot is within `retarget_within` cells of the current target;
- the sign chain beyond the target is now known, so the planner can skip ahead
  (at most `max_skip` walls).

It is called from two places:

- **After a sighting changes the graph** (section 4): new knowledge may extend the chain.
- **Every tick while driving** (section 6): the gate depends on distance, so the robot getting
  close enough can enable a retarget even with no new sighting.

`send_nav` with a new goal makes Nav2 drop the old one. Nav2 then reports the old goal as
aborted/canceled; the `nav_seq` check in section 7 ignores that report.

```python
def flush_planner_log(self):
    log = getattr(self.planner, 'log', None)
    for line in log[self._planner_log_seen:]:
        self.get_logger().info(f'planner: {line}')
    self._planner_log_seen = len(log)
```

The graph planner appends explanations (re-reads, shortcuts, recovery) to a plain Python
list, because the planner is rclpy-free. This prints only the lines added since the last flush.

---

## 6. The state machine (lines 285-367)

### `step()`, 10 Hz

```python
def step(self):
    if self.state == 'ALIGN':    self.step_align(); return        # section 8
    if self.state == 'NAVIGATE': self.maybe_retarget(); return    # Nav2 is driving; just watch
    if self.state == 'DONE':     return
    pose = self.robot_pose()
    if pose is None: return                                       # no TF yet
    cell_pose = self.grid.locate(*pose)                           # snap to cell + nearest heading
    if cell_pose is None: warn 'robot outside the maze grid'; return
    ...
```

`grid.locate` turns a metric pose `(x, y, yaw)` into a `CellPose(row, col, heading)`: the
cell the robot is in, and E/N/W/S by the nearest 90 deg. Everything the planner sees is in
these grid terms.

**START** (304-322):

```python
if self.state == 'START':
    if not self.nav.server_is_ready() or not self.navigator_active(): return   # Nav2 not up
    if not self.localized: log 'waiting for /localization_ready'; return       # ICP + spin running
    self.planner.reset(cell_pose)
    for obs in self.early_obs: self.planner.observe(obs)                       # replay (section 4)
    self.early_obs = []
    self.nav_target = cell_pose
    self.start_align()                     # placed by hand: square up before the first decision
    return
```

`nav_target = cell_pose` gives ALIGN a target heading: the nearest of E/N/W/S.

**INSPECT** (324-326): handled in `step_inspect()` below.

**PLAN** (everything else, 328-350):

```python
action = self.planner.next_action(cell_pose)
self.flush_planner_log()
if isinstance(action, Inspect) and not self.aligned:
    self.nav_target = cell_pose
    self.start_align()                     # a reading needs a square robot; ask again after ALIGN
    return
log f'{cell_pose}: {action}'
if isinstance(action, GoTo):
    self.nav_attempts = 0
    self.send_nav(action.target)                                   # -> NAVIGATE
elif isinstance(action, Inspect):
    self.inspect_votes, self.inspect_frames_seen = [], 0
    self.inspect_started = now
    self.inspect_pose = cell_pose
    self.state = 'INSPECT'
elif isinstance(action, Finish):
    self.finish('goal reached', goal=True)                         # -> DONE + beeps
elif isinstance(action, GiveUp):
    self.finish(f'gave up: {action.reason}')
```

The `Inspect and not aligned` case exists for sighting-only mode: there, arrival skips ALIGN
(section 7), so if the planner then asks for a reading, the node squares up first. After ALIGN
it is back in PLAN and asks the planner again; the planner returns the same `Inspect`, and now
`aligned` is True.

### `step_inspect()` (352-367)

```python
enough = self.inspect_frames_seen >= self.cfg['inspect_frames']
if not enough and now - self.inspect_started < self.cfg['inspect_timeout']:
    return                                                    # keep collecting
if self.inspect_votes:
    counts = Counter(s for s, _ in self.inspect_votes)
    sign, n = counts.most_common(1)[0]                        # majority sign
    conf = n / max(1, self.inspect_frames_seen)               # share of ALL frames
else:
    sign, conf = SignClass.EMPTY, 1.0                         # nothing in front at all
self.readings[self.inspect_pose] = (sign, conf)
self.planner.observe(SignObservation(self.inspect_pose, sign, conf, Source.INSPECT, time.time()))
self.state = 'PLAN'
```

The reading ends after 5 frames or 3 s, whichever comes first. The confidence is the winning
sign's share of **all** frames, including frames with no sign in front: 3 LEFT votes in 5
frames gives conf 0.6. The planner decides what to do with a low-confidence reading.

---

## 7. Nav2 (lines 448-498)

### Sending a goal

```python
def send_nav(self, target):
    x, y, yaw = self.grid.pose_to_map(target)          # CellPose -> cell center + heading yaw
    goal = NavigateToPose.Goal(); ... fill pose, quaternion from yaw ...
    self.nav_target = target
    self.nav_attempts += 1
    self.aligned = False
    self.state = 'NAVIGATE'
    self.nav_seq += 1
    seq = self.nav_seq
    self.nav.send_goal_async(goal).add_done_callback(
        lambda f, seq=seq: self.on_goal_response(f, seq))
```

An action call has two asynchronous replies: first "accepted or rejected", later "finished
with status X". Each callback is given the `seq` of the goal it belongs to.

`seq=seq` in the lambda freezes the current value. Without it, the lambda would read
`self.nav_seq` when it runs, which may already belong to a newer goal.

### Goal accepted?

```python
def on_goal_response(self, future, seq):
    handle = future.result()
    if seq != self.nav_seq: return          # already replaced by a retarget
    if not handle.accepted:
        warn 'Nav2 rejected the goal'
        self.bt_active = False              # Nav2 may be restarting: re-check before the next goal
        self.state = 'PLAN'; return
    self.nav_accepted, self.nav_goal_handle = True, handle
    handle.get_result_async().add_done_callback(lambda f, seq=seq: self.on_nav_result(f, seq))
```

### Goal finished

```python
def on_nav_result(self, future, seq):
    if seq != self.nav_seq: return          # old goal; Nav2 says aborted because we replaced it
    status = future.result().status
    if status != SUCCEEDED:
        self.nav_failed(f'Nav2 status {status}'); return
    if not getattr(self.planner, 'inspect', True):
        log 'arrived ... (no align)'
        self.state = 'PLAN'; return         # sighting-only: no reading, no squaring up
    self.start_align()                      # normal: square up, then PLAN
```

```python
def nav_failed(self, why):
    if self.nav_attempts <= self.cfg['nav_retries'] and self.navigator_active():
        warn f'{why}, retrying ...'; self.send_nav(self.nav_target)   # one retry (nav_retries = 1)
    else:
        warn f'{why}, re-planning'; self.state = 'PLAN'
```

After a failure with no retries left, the node does not stop. It goes to PLAN, where the
current cell pose is read again and the planner decides from there.

`nav_attempts` starts at 0 and `send_nav` increments it, so the first goal makes it 1. On a
failure, `1 <= nav_retries (1)` allows one retry (attempt 2). A second failure goes to PLAN.

### A goal Nav2 never answers (`nav_watchdog`)

Both replies above are callbacks: if Nav2 never sends one, nothing happens and the mission
waits forever. That happened on the robot (2026-11-23, onboard): under CPU load the map
server missed its 4 s bond heartbeat, the lifecycle manager reset **every** Nav2 server
mid-goal, and the goal in flight never got a reply. So `step()` checks, while in NAVIGATE:

```python
def nav_watchdog(self):
    waited = now - self.nav_sent                      # send_nav stamps nav_sent, clears nav_accepted
    if not self.nav_accepted and waited > self.cfg['nav_ack_timeout']:    # 10 s
        why = 'no answer from Nav2 ...'
    elif self.nav_accepted and waited > self.cfg['nav_timeout']:          # 120 s
        why = 'Nav2 goal still running ...'; self.nav_goal_handle.cancel_goal_async()
    else:
        return False
    self.nav_seq += 1          # a late reply to the abandoned goal is ignored
    self.bt_active = False     # re-ask bt_navigator's state before sending again
    self.nav_failed(why)
    return True
```

`nav_timeout` is generous (longest leg plus a recovery); it only has to end a hang. And PLAN
does not send a goal until `navigator_active()` is true again, so after a reset the mission
waits ("waiting for Nav2 bt_navigator to be active") instead of firing goals Nav2 rejects.

---

## 8. ALIGN (lines 500-537)

Why it exists: Nav2 reports success while its velocity smoother is still ramping a turn down.
The robot then overshot by about 15 deg in sim. Reading signs or deciding headings needs the
robot square to the wall.

```python
def start_align(self):
    self.align_started = now
    self.align_last_yaw = None
    self.align_active = False
    self.state = 'ALIGN'
```

`step_align()` at 10 Hz:

```python
pose = self.robot_pose()
tx, ty, tyaw = self.grid.pose_to_map(self.nav_target)
err = wrap(tyaw - yaw)                                         # signed heading error
moving = last_yaw is not None and |wrap(yaw - last_yaw)| > align_still   # turned > 0.01 rad in 0.1 s
first = last_yaw is None
self.align_last_yaw = yaw
timeout = elapsed > align_timeout

# Phase 1: wait until the robot is still (Nav2's ramp-down finishing)
if (first or moving) and not self.align_active and not timeout:
    return

# Phase 2: correct the heading
self.align_active = True
done = (|err| <= align_tol and not moving) or timeout
cmd = Twist()
if not done and |err| > align_tol:
    w = clamp(1.2 * |err|, align_min_turn, align_max_turn)     # P-controller, 0.08..0.4 rad/s
    cmd.angular.z = copysign(w, err)
self.cmd_vel.publish(cmd)                                      # zero inside tolerance = brake
if done:
    log 'arrived target: N cm, yaw X deg after T s align'
    self.aligned = True
    self.state = 'PLAN'
```

Two phases:

1. **Wait until still.** Do nothing until the yaw stops changing between ticks; the first tick
   always waits because there is no previous yaw. `align_active` makes this happen only once
   per ALIGN, so the robot's own correction turns don't send it back to waiting.
2. **Turn.** A proportional controller: speed = 1.2 x error, clamped to [0.08, 0.4] rad/s.
   The minimum is needed because the burger doesn't move below about 0.08 rad/s.

Finishing requires the error within 2 deg **and** the robot not moving. Inside the tolerance
the node publishes a zero Twist (brake) and checks again next tick. This prevents declaring
success while still coasting through the target.

The 12 s timeout ends ALIGN anyway, with a warning.

ALIGN only turns; it doesn't move the robot. The "N cm" in the log is the distance from the
cell center, for information.

---

## 9. Finish, beeps, `main()` (lines 539-576)

```python
def finish(self, why, goal=False):
    self.state = 'DONE'
    self.dump_graph()                                     # section 10
    if goal:
        self.beeps_left = int(self.cfg['goal_beeps'])     # 3
        self.beep()                                       # first one now
        self.beep_timer = self.create_timer(self.cfg['beep_gap'], self.beep)   # then every 2.5 s

def beep(self):
    if self.beeps_left <= 0:
        self.beep_timer.cancel(); return
    self.beeps_left -= 1
    if not self.sound.service_is_ready():
        warn 'GOAL REACHED (no /sound service to beep)'; return     # Gazebo has no /sound
    self.sound.call_async(Sound.Request(value=self.cfg['goal_sound']))
```

The grade requires the robot to **signal** the goal; without it the run counts as stopped
early (10% off). Beeps are spaced 2.5 s apart because each sound lasts about 2 s and a new
request cuts off the current one.

In DONE the node does not publish a stop on `/cmd_vel`. The robot is already stopped: the last
motion was Nav2's goal (which ends stopped) or ALIGN (which ends with a zero Twist).

```python
def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.state != 'DONE':
            node.dump_graph()               # Ctrl-C mid-run: keep the graph built so far
        node.destroy_node()
        rclpy.try_shutdown()
```

---

## 10. RViz markers and the graph dump (lines 370-445)

Cosmetic only; nothing here affects driving. `publish_markers()` at 2 Hz builds one
`MarkerArray` on `/mission/markers`:

| Namespace | What | When |
|---|---|---|
| `target` | green arrow at the Nav2 goal | while NAVIGATE (deleted otherwise) |
| `state` | text above the robot: state + last action | always |
| `inspected` | `[left 1.0]` in front of each inspected face | after inspections |
| `graph` | each face's believed sign + score/votes; `?` if not firm yet | graph planners |
| `edges` | orange arrows face -> next face for every known sign edge | graph planners |

```python
def face_point(face, frac=0.3):              # a point in front of a wall face
    x, y, yaw = self.grid.pose_to_map(face)  # cell center, facing the wall
    d = frac * self.grid.cell_size
    return x + d * math.cos(yaw), y + d * math.sin(yaw)
```

`face_point` moves from the cell center towards the wall, so labels sit near the wall they
describe.

The list of `graph` and `edges` markers can get shorter between messages. RViz keeps markers
until told otherwise, so the code sends DELETE for the ids that were in the previous message
but not in this one (`_marker_counts`).

`dump_graph()` writes the planner's `graph.to_dict()` (plus planner name and, for explore, the
targets) to `graph_dump` as YAML. `plot_graph.py` turns it into a PNG.

---

## 11. One full cycle, traced

Graph planner, inspect mode, robot placed at (0,5) facing N:

1. **START**: wait for Nav2 active and `/localization_ready`. During the init spin, detections
   go into `early_obs`.
2. `planner.reset((0,5)N)`, replay `early_obs` -> **ALIGN** to N -> **PLAN**.
3. `next_action` -> `Inspect` (aligned) -> **INSPECT**: 5 frames, majority RIGHT ->
   `observe(INSPECT RIGHT)` -> **PLAN**.
4. `next_action` -> `GoTo((0,5)E... next wall)` -> `send_nav` -> **NAVIGATE**. Meanwhile
   passive sightings keep updating the graph.
5. Nav2 SUCCEEDED (seq matches) -> **ALIGN** -> **PLAN** -> `Inspect` -> ...
6. Eventually `next_action` -> `Finish` -> **DONE**, graph dumped, 3 beeps.

Sighting-only mode (`inspect: false`) differs:

- In step 4, `maybe_retarget()` may replace the goal while driving.
- In step 5, arrival goes straight to PLAN (no ALIGN). The planner usually returns another
  `GoTo` immediately from the passive belief.
- An `Inspect` happens only when nothing is known about the wall ahead, or to confirm a GOAL;
  ALIGN runs first in that case.
