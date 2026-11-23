"""
Node mission: the executive. Asks the planner what to do, does it, reports back.

    PLAN       pose from TF (map -> base_link), snapped to the grid; planner.next_action()
    NAVIGATE   GoTo -> NavigateToPose to the target cell center + heading
    ALIGN      after Nav2: wait until the robot stops, then turn in place (/cmd_vel) until
               the heading is within align_tol. Nav2 reports success while the velocity
               smoother is still ramping a turn down, which overshot by ~15 deg in sim.
    INSPECT    stationary; collect sign_detections of the wall in front, majority vote,
               planner.observe(INSPECT), back to PLAN
    DONE       Finish -> stop, beep goal_beeps times on /sound. GiveUp -> stop, log.

Every detection is also placed on a wall face (robot pose at the image stamp + camera
offset -> camera ray at the detection's bearing -> first wall in the known maze, checked
against LIDAR) and passed to the planner as a PASSIVE observation. The grid planner
ignores them; graph-building planners (explore, graph) put them in their SignGraph.

Publishes /mission/markers (RViz): Nav2 target arrow, state + last action above the
robot, inspected readings, and for graph planners every face's belief + known edges.
graph_dump (path): the planner's graph is written there (yaml) when the run ends.
"""

import math
import os
from pathlib import Path
import time
from collections import Counter

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from turtlebot3_msgs.srv import Sound

from team85_final_interfaces.msg import SignDetectionArray

from ..maze.grid import MazeGrid
from ..maze.sign_locator import camera_pose, lidar_agrees, raycast
from ..planning.base import Finish, GiveUp, GoTo, Inspect, SignObservation, Source
from ..planning.registry import make_planner
from ..vision.signs import SignClass


def share_path(path: str) -> str:
    return path if os.path.isabs(path) else os.path.join(
        get_package_share_directory('team85_final'), path)


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission')
        p = self.declare_parameters('', [
            ('planner', 'grid'),
            ('maze_file', 'config/maze.yaml'),
            ('frame_id', 'map'),
            ('base_frame', 'base_link'),
            ('inspect_frames', 5),
            ('inspect_timeout', 3.0),
            ('inspect_min_area', 0.02),
            ('inspect_center_tol', 0.35),  # fallback only (no TF): |bearing| for "in front"
            ('inspect_settle', 0.3),   # s after stopping before frames count
            ('planner_params', ''),   # yaml mapping passed to the planner, e.g. "{steps: 20}"
            ('graph_dump', ''),       # yaml file for the planner's graph at the end ('' = off)
            # Passive sightings: where is a detected sign? (camera ray -> first wall)
            ('camera_offset_x', 0.069),   # m, camera in base_link (sim: camera_rgb_frame)
            ('camera_offset_y', 0.0),
            ('passive_max_range', 2.5),   # m camera -> wall; far signs get misclassified
            ('passive_max_lateral', 0.3),  # x cell size: |hit - face center| (corners = ambiguous)
            ('passive_edge_px', 3),       # drop boxes this close to the image's left/right edge
            ('lidar_tolerance', 0.25),    # m; LIDAR must see the wall where the ray hits it
            ('nav_retries', 1),
            ('align_tol', 0.035),     # rad (2 deg): final heading error before reading signs
            ('align_max_turn', 0.4),  # rad/s
            ('align_min_turn', 0.08), # rad/s; below this the burger doesn't move
            ('align_still', 0.01),    # rad per step: robot counts as stopped
            ('align_timeout', 12.0),  # s (180 deg at 0.4 rad/s ~ 8 s)
            ('goal_sound', 1),        # turtlebot3 Sound value (1 = ON; recovery uses 3 and 1)
            ('goal_beeps', 3),        # how many times to play it at the goal
            ('beep_gap', 2.5),        # s between beeps (each sound lasts ~2 s; a new one cuts it)
            ('wait_localized', False),  # wait for set_initial_pose's /localization_ready
        ])
        self.cfg = {x.name: x.value for x in p}
        self.grid = MazeGrid.from_yaml(share_path(self.cfg['maze_file']))
        planner_params = yaml.safe_load(self.cfg['planner_params'] or '{}') or {}
        self.planner = make_planner(self.cfg['planner'], self.grid, **planner_params)
        self.get_logger().info(f"planner={self.planner.name} {planner_params}\n"
                               f"{self.grid.render()}")

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.bt_state = self.create_client(GetState, '/bt_navigator/get_state')
        self.bt_state_future = None
        self.bt_active = False
        self.scan: LaserScan | None = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(SignDetectionArray, '/sign_detections', self.on_detections, 10)
        self.localized = not self.cfg['wait_localized']
        self.create_subscription(
            Bool, '/localization_ready', self.on_localized,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.state = 'START'
        self.early_obs: list[SignObservation] = []   # passive sightings before planner.reset
        self.inspect_votes: list[tuple[SignClass, float]] = []
        self.inspect_frames_seen = 0
        self.inspect_started = 0.0
        self.inspect_pose = None
        self.nav_goal_handle = None
        self.nav_target = None
        self.nav_attempts = 0
        self.nav_seq = 0                  # id of the current Nav2 goal; older results ignored
        self.aligned = False              # squared up to the current face (ALIGN done)
        self.sound = self.create_client(Sound, '/sound')
        self.beeps_left = 0
        self.beep_timer = None
        self.last_action = ''
        self.readings = {}                       # CellPose -> (SignClass, conf), inspected
        self.markers = self.create_publisher(MarkerArray, '/mission/markers', 1)
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.align_started = 0.0
        self.align_last_yaw = None
        self.align_active = False
        self._marker_counts: dict[str, int] = {}
        self._planner_log_seen = 0
        self.create_timer(0.1, self.step)
        self.create_timer(0.5, self.publish_markers)

    # ---- inputs -------------------------------------------------------
    def on_scan(self, msg: LaserScan):
        self.scan = msg

    def on_localized(self, msg: Bool):
        if msg.data and not self.localized:
            self.get_logger().info('localization ready (set_initial_pose)')
            self.localized = True

    def navigator_active(self) -> bool:
        """Has bt_navigator reached lifecycle state 'active'? (polled without blocking)"""
        if self.bt_active:
            return True
        f = self.bt_state_future
        if f is not None and f.done():
            res = f.result()
            self.bt_active = res is not None and res.current_state.id == State.PRIMARY_STATE_ACTIVE
            self.bt_state_future = None
            if self.bt_active:
                self.get_logger().info('Nav2 bt_navigator is active')
        if not self.bt_active and self.bt_state_future is None and self.bt_state.service_is_ready():
            self.bt_state_future = self.bt_state.call_async(GetState.Request())
        return self.bt_active

    def robot_pose(self, stamp=None, frame=None):
        """(x, y, yaw) of `frame` (default base_frame) in the map frame at `stamp`, or None.

        With a stamp older than the TF buffer (or in the future), falls back to the latest.
        """
        frame = frame or self.cfg['base_frame']
        try:
            try:
                t = self.tf.lookup_transform(self.cfg['frame_id'], frame,
                                             Time.from_msg(stamp) if stamp else Time())
            except TransformException:
                if stamp is None:
                    raise
                t = self.tf.lookup_transform(self.cfg['frame_id'], frame, Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def range_at(self, bearing: float) -> float | None:
        s = self.scan
        if s is None:
            return None
        i = round((bearing - s.angle_min) / s.angle_increment) % len(s.ranges)
        r = s.ranges[i]
        return r if s.range_min < r < s.range_max else None

    def on_inspected_wall(self, d, stamp) -> bool:
        """Is detection d on the wall being inspected? Its camera ray must hit that face.

        (Not a bearing window: up close and off-center, the sign in front can be 25 deg
        off-axis while a neighbouring wall's sign is nearly centered.)"""
        pose = self.robot_pose(stamp)
        if pose is None:
            return abs(d.bearing) <= self.cfg['inspect_center_tol']
        cx, cy, cyaw = camera_pose(*pose, self.cfg['camera_offset_x'], self.cfg['camera_offset_y'])
        hit = raycast(self.grid, cx, cy, cyaw + d.bearing)
        self.get_logger().debug(f'inspect ray {d.label} b{d.bearing:+.2f} from '
                                f'{self.grid.map_to_maze(cx, cy)} yaw {cyaw:+.2f} -> '
                                f'{hit.face if hit else None}')
        return hit is not None and hit.face == self.inspect_pose

    def on_detections(self, msg: SignDetectionArray):
        area = float(msg.image_width * msg.image_height) or 1.0
        if self.state == 'INSPECT' and (self.get_clock().now().nanoseconds * 1e-9
                                        >= self.inspect_started + self.cfg['inspect_settle']):
            # Frames arriving in the first inspect_settle s are skipped (still settling from
            # Nav2). Receipt time, not the image stamp: the robot's clock may be skewed.
            self.inspect_frames_seen += 1
            front = [d for d in msg.detections
                     if self.on_inspected_wall(d, msg.header.stamp)
                     and d.width * d.height / area >= self.cfg['inspect_min_area']]
            if front:
                best = max(front, key=lambda d: d.width * d.height)
                self.inspect_votes.append((SignClass(best.sign), best.confidence))
            self.get_logger().debug(
                f'inspect frame {self.inspect_frames_seen}: ' + ', '.join(
                    f'{d.label} {d.confidence:.2f} b{d.bearing:+.2f} '
                    f'a{d.width * d.height / area:.3f} front={d in front}'
                    for d in msg.detections))
        # Passive sightings, for planners that use them.
        if not msg.detections:
            return
        pose = self.robot_pose(msg.header.stamp)      # where the robot was at the image
        if pose is None:
            return
        cx, cy, cyaw = camera_pose(*pose, self.cfg['camera_offset_x'],
                                   self.cfg['camera_offset_y'])
        scan_pose = (self.robot_pose(self.scan.header.stamp, self.scan.header.frame_id)
                     if self.scan is not None else None)
        edge = self.cfg['passive_edge_px']
        for d in msg.detections:
            if d.cx - d.width / 2 < edge or d.cx + d.width / 2 > msg.image_width - edge:
                continue                                # cut off by the image border
            hit = raycast(self.grid, cx, cy, cyaw + d.bearing, self.cfg['passive_max_range'])
            if hit is None or abs(hit.lateral) > self.cfg['passive_max_lateral'] * self.grid.cell_size:
                continue
            if scan_pose is not None and not lidar_agrees(hit, *scan_pose, self.range_at,
                                                          self.cfg['lidar_tolerance']):
                continue
            obs = SignObservation(hit.face, SignClass(d.sign), d.confidence, Source.PASSIVE,
                                  time.time())
            if self.state == 'START':
                # Sightings while localizing (the spin): planner.reset() hasn't run yet,
                # so keep them and hand them over right after it (see step()).
                self.early_obs.append(obs)
                continue
            if self.planner.observe(obs):
                g = self.planner.graph
                self.get_logger().info(
                    f'graph: {hit.face} = {getattr(g.sign(hit.face), "name", None)} -> {g.successor(hit.face)} '
                    f'({d.label} {d.confidence:.2f} at {hit.distance:.1f} m)')
                self.maybe_retarget()

    def maybe_retarget(self):
        """Sighting-only graph planner: move the Nav2 goal mid-drive when the known chain
        ahead of the current target grows (planner.retarget)."""
        retarget = getattr(self.planner, 'retarget', None)
        if retarget is None or self.state != 'NAVIGATE' or self.nav_target is None:
            return
        pose = self.robot_pose()
        new = retarget(self.nav_target, pose[:2] if pose is not None else None)
        if new is None:
            return
        self.flush_planner_log()
        self.get_logger().info(f'retarget mid-drive: {self.nav_target} -> {new}')
        self.last_action = f'GoTo({new}) [retarget]'
        self.nav_attempts = 0
        self.send_nav(new)

    def flush_planner_log(self):
        log = getattr(self.planner, 'log', None)
        if log is not None:
            for line in log[self._planner_log_seen:]:
                self.get_logger().info(f'planner: {line}')
            self._planner_log_seen = len(log)

    # ---- state machine ------------------------------------------------
    def step(self):
        if self.state == 'ALIGN':
            self.step_align()
            return
        if self.state == 'NAVIGATE':
            # Retarget gate is distance-based: re-check while driving, not only when a
            # sighting changes the graph (the next sign is often known long before).
            self.maybe_retarget()
            return
        if self.state == 'DONE':
            return
        pose = self.robot_pose()
        if pose is None:
            return
        cell_pose = self.grid.locate(*pose)
        if cell_pose is None:
            self.get_logger().warn('robot outside the maze grid', throttle_duration_sec=2.0)
            return

        if self.state == 'START':
            # The action server exists before bt_navigator is active, and goals sent then
            # are rejected (real robot: 15 rejections in 1.5 s). Wait for 'active'.
            if not self.nav.server_is_ready() or not self.navigator_active():
                return
            if not self.localized:     # set_initial_pose: ICP + spin still running
                self.get_logger().info('waiting for /localization_ready',
                                       throttle_duration_sec=5.0)
                return
            self.planner.reset(cell_pose)
            for obs in self.early_obs:
                self.planner.observe(obs)
            self.get_logger().info(f'start at {cell_pose} ({len(self.early_obs)} sightings '
                                   'from before the start)')
            self.early_obs = []
            # Placed by hand: square up to the nearest heading before the first reading.
            self.nav_target = cell_pose
            self.start_align()
            return

        if self.state == 'INSPECT':
            self.step_inspect()
            return

        action = self.planner.next_action(cell_pose)
        # Graph planners explain their non-obvious decisions (re-reads, recovery, lost).
        self.flush_planner_log()
        if isinstance(action, Inspect) and not self.aligned:
            # Sighting-only mode skips ALIGN on arrival; a reading still needs it.
            self.nav_target = cell_pose
            self.start_align()
            return
        self.get_logger().info(f'{cell_pose}: {action}')
        self.last_action = str(action)
        if isinstance(action, GoTo):
            self.nav_attempts = 0
            self.send_nav(action.target)
        elif isinstance(action, Inspect):
            self.inspect_votes, self.inspect_frames_seen = [], 0
            self.inspect_started = self.get_clock().now().nanoseconds * 1e-9   # sim time in sim
            self.inspect_pose = cell_pose
            self.state = 'INSPECT'
            # TODO: align square to the wall with LIDAR before reading (lab recommends it).
        elif isinstance(action, Finish):
            self.finish('goal reached', goal=True)
        elif isinstance(action, GiveUp):
            self.finish(f'gave up: {action.reason}')

    def step_inspect(self):
        enough = self.inspect_frames_seen >= self.cfg['inspect_frames']
        now = self.get_clock().now().nanoseconds * 1e-9
        if not enough and now - self.inspect_started < self.cfg['inspect_timeout']:
            return
        if self.inspect_votes:
            counts = Counter(s for s, _ in self.inspect_votes)
            sign, n = counts.most_common(1)[0]
            conf = n / max(1, self.inspect_frames_seen)
        else:
            sign, conf = SignClass.EMPTY, 1.0
        self.get_logger().info(f'inspect {self.inspect_pose}: {sign.name} ({conf:.2f})')
        self.readings[self.inspect_pose] = (sign, conf)
        self.planner.observe(SignObservation(self.inspect_pose, sign, conf, Source.INSPECT,
                                             time.time()))
        self.state = 'PLAN'

    # ---- RViz ---------------------------------------------------------
    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        out = MarkerArray()

        def marker(ns, mid, mtype, x, y, z=0.0, yaw=0.0, rgb=(1.0, 1.0, 1.0)):
            m = Marker(ns=ns, id=mid, type=mtype, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = self.cfg['frame_id'], stamp
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.color.r, m.color.g, m.color.b, m.color.a = *rgb, 1.0
            out.markers.append(m)
            return m

        if self.nav_target is not None and self.state == 'NAVIGATE':
            x, y, yaw = self.grid.pose_to_map(self.nav_target)
            m = marker('target', 0, Marker.ARROW, x, y, 0.05, yaw, (0.1, 1.0, 0.2))
            m.scale.x, m.scale.y, m.scale.z = 0.35, 0.07, 0.07
        else:
            out.markers.append(Marker(ns='target', id=0, action=Marker.DELETE))
        pose = self.robot_pose()
        if pose is not None:
            m = marker('state', 0, Marker.TEXT_VIEW_FACING, pose[0], pose[1], 0.45,
                       rgb=(1.0, 1.0, 0.3))
            m.scale.z = 0.12
            m.text = f'{self.state}  {self.last_action}'
        colors = {SignClass.LEFT: (0.3, 0.6, 1.0), SignClass.RIGHT: (0.3, 0.6, 1.0),
                  SignClass.GOAL: (1.0, 0.85, 0.0), SignClass.EMPTY: (0.6, 0.6, 0.6)}

        def face_point(face, frac=0.3):                  # in front of the wall face
            x, y, yaw = self.grid.pose_to_map(face)
            d = frac * self.grid.cell_size
            return x + d * math.cos(yaw), y + d * math.sin(yaw)

        for i, (face, (sign, conf)) in enumerate(self.readings.items()):
            m = marker('inspected', i, Marker.TEXT_VIEW_FACING, *face_point(face, 0.25), 0.35,
                       rgb=colors.get(sign, (1.0, 0.3, 0.3)))
            m.scale.z = 0.1
            m.text = f'[{sign.name.lower()} {conf:.1f}]'

        graph = getattr(self.planner, 'graph', None)
        if graph is not None:
            # One label per face with evidence: believed sign (or top vote + '?'), votes.
            for i, f in enumerate(sorted(graph._beliefs, key=str)):
                b, sign = graph.belief(f), graph.sign(f)
                top, score = b.ranked()[0]
                m = marker('graph', i, Marker.TEXT_VIEW_FACING, *face_point(f, 0.38), 0.18,
                           rgb=colors.get(sign, (1.0, 0.3, 0.3)) if sign is not None
                           else (0.55, 0.55, 0.55))
                m.scale.z = 0.08
                m.text = (f'{sign.name.lower()}' if sign is not None
                          else f'{top.name.lower()}?') + f' {score:.1f}/{b.n_obs}'
            # Known edges: face -> next face, as arrows.
            for i, (a, b) in enumerate(graph.edges()):
                m = marker('edges', i, Marker.ARROW, 0.0, 0.0, 0.0, rgb=(1.0, 0.5, 0.1))
                m.pose.orientation.z, m.pose.orientation.w = 0.0, 1.0
                m.scale.x, m.scale.y, m.scale.z = 0.025, 0.07, 0.08
                for x, y in (face_point(a, 0.38), face_point(b, 0.38)):
                    m.points.append(Point(x=x, y=y, z=0.1))
        # Clear markers left from a longer list in the last message.
        n_now = {ns: sum(1 for m in out.markers if m.ns == ns) for ns in ('graph', 'edges')}
        for ns, n in n_now.items():
            for i in range(n, self._marker_counts.get(ns, 0)):
                out.markers.append(Marker(ns=ns, id=i, action=Marker.DELETE))
        self._marker_counts = n_now
        self.markers.publish(out)

    def dump_graph(self):
        graph, path = getattr(self.planner, 'graph', None), self.cfg['graph_dump']
        if graph is None or not path:
            return
        d = graph.to_dict()
        d['planner'] = self.planner.name
        if hasattr(self.planner, 'targets'):
            d['targets'] = [str(t) for t in self.planner.targets]
        Path(path).write_text(yaml.safe_dump(d, default_flow_style=None, sort_keys=False))
        self.get_logger().info(f'graph ({len(d["faces"])} faces) written to {path}')

    # ---- Nav2 ---------------------------------------------------------
    def send_nav(self, target):
        x, y, yaw = self.grid.pose_to_map(target)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.cfg['frame_id']
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.nav_target = target
        self.nav_attempts += 1
        self.aligned = False
        self.state = 'NAVIGATE'
        self.nav_seq += 1
        seq = self.nav_seq
        self.nav.send_goal_async(goal).add_done_callback(
            lambda f, seq=seq: self.on_goal_response(f, seq))

    def on_goal_response(self, future, seq):
        handle = future.result()
        if seq != self.nav_seq:
            return                     # replaced (retarget) before Nav2 answered
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal')
            self.state = 'PLAN'
            return
        handle.get_result_async().add_done_callback(lambda f, seq=seq: self.on_nav_result(f, seq))

    def on_nav_result(self, future, seq):
        if seq != self.nav_seq:
            return                     # a goal we replaced: Nav2 reports it aborted/canceled
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED and self.nav_attempts <= self.cfg['nav_retries']:
            self.get_logger().warn(f'Nav2 status {status}, retrying {self.nav_target}')
            self.send_nav(self.nav_target)
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.state = 'PLAN'          # out of retries: re-plan from wherever we are
            return
        if not getattr(self.planner, 'inspect', True):
            # Sighting-only: no reading here, so no squaring up - straight to the next goal.
            pose = self.robot_pose()
            if pose is not None:
                tx, ty, tyaw = self.grid.pose_to_map(self.nav_target)
                err = math.atan2(math.sin(tyaw - pose[2]), math.cos(tyaw - pose[2]))
                self.get_logger().info(f'arrived {self.nav_target}: '
                                       f'{math.hypot(pose[0] - tx, pose[1] - ty) * 100:.0f} cm, '
                                       f'yaw {math.degrees(-err):+.1f} deg (no align)')
            self.state = 'PLAN'
            return
        self.start_align()

    def start_align(self):
        """Square up to self.nav_target's heading (ALIGN) before the next decision."""
        self.align_started = self.get_clock().now().nanoseconds * 1e-9
        self.align_last_yaw = None
        self.align_active = False
        self.state = 'ALIGN'

    def step_align(self):
        """Square up to the target heading after Nav2 (10 Hz from step())."""
        pose = self.robot_pose()
        if pose is None:
            return
        tx, ty, tyaw = self.grid.pose_to_map(self.nav_target)
        err = math.atan2(math.sin(tyaw - pose[2]), math.cos(tyaw - pose[2]))
        moving = (self.align_last_yaw is not None and
                  abs(math.atan2(math.sin(pose[2] - self.align_last_yaw),
                                 math.cos(pose[2] - self.align_last_yaw))) > self.cfg['align_still'])
        first = self.align_last_yaw is None
        self.align_last_yaw = pose[2]
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.align_started
        timeout = elapsed > self.cfg['align_timeout']
        if (first or moving) and not self.align_active and not timeout:
            return              # let Nav2's velocity smoother finish its ramp-down first
        self.align_active = True
        done = (abs(err) <= self.cfg['align_tol'] and not moving) or timeout
        if timeout:
            self.get_logger().warn(f'align timeout at yaw error {math.degrees(err):+.1f} deg')
        cmd = Twist()
        if not done and abs(err) > self.cfg['align_tol']:
            w = max(self.cfg['align_min_turn'], min(self.cfg['align_max_turn'], 1.2 * abs(err)))
            cmd.angular.z = math.copysign(w, err)
        self.cmd_vel.publish(cmd)          # zero when inside tolerance: brake, then re-check
        if done:
            self.get_logger().info(f'arrived {self.nav_target}: '
                                   f'{math.hypot(pose[0] - tx, pose[1] - ty) * 100:.0f} cm, '
                                   f'yaw {math.degrees(-err):+.1f} deg after {elapsed:.1f} s align')
            self.aligned = True
            self.state = 'PLAN'

    def finish(self, why: str, goal: bool = False):
        self.state = 'DONE'
        self.get_logger().info(f'DONE: {why}')
        self.dump_graph()
        if goal:
            # The robot must signal the goal, or the run counts as stopped by us (c = 1, -10%).
            self.beeps_left = int(self.cfg['goal_beeps'])
            self.beep()
            self.beep_timer = self.create_timer(float(self.cfg['beep_gap']), self.beep)

    def beep(self):
        if self.beeps_left <= 0:
            if self.beep_timer is not None:
                self.beep_timer.cancel()
            return
        self.beeps_left -= 1
        if not self.sound.service_is_ready():
            # No /sound in Gazebo; on the robot it needs FastDDS + the robot's domain.
            self.get_logger().warn('GOAL REACHED (no /sound service to beep)')
            return
        self.sound.call_async(Sound.Request(value=int(self.cfg['goal_sound'])))


def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.state != 'DONE':
            try:
                node.dump_graph()                     # Ctrl-C mid-run: keep what was built
            except Exception as e:                    # noqa: BLE001
                print(f'graph dump failed: {e}')
        node.destroy_node()
        rclpy.try_shutdown()
