#!/usr/bin/env python3

import math
import os
import threading
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose

from turtlebot3_msgs.srv import Sound
from turtlebot3_recovery_lab.srv import DivertRobot


# TurtleBot3 sound commands:
# 0 = OFF
# 1 = ON
# 2 = LOW_BATTERY
# 3 = ERROR
# 4 = BUTTON1
# 5 = BUTTON2
SOUND_ERROR = 3

# Nav2 goal states after which the goal is over
GOAL_DONE = (
    GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_CANCELED,
    GoalStatus.STATUS_ABORTED,
)


class StudentRecoveryServer(Node):
    def __init__(self):
        super().__init__('student_recovery_server')

        default_file = os.path.join(
            get_package_share_directory('turtlebot3_recovery_lab'),
            'config',
            'recovery_candidates.yaml'
        )
        candidates_file = self.declare_parameter(
            'candidates_file', default_file
        ).value
        # Sound played when recovery succeeds. Must differ from SOUND_ERROR,
        # which is played when recovery starts.
        self.success_sound = self.declare_parameter('success_sound', 1).value
        # When to play it:
        #   'arrival' - once the robot reaches the recovery waypoint, i.e. the
        #               behavior tree's FollowPath to it succeeds; no beep if
        #               the recovery is abandoned on the way (that FollowPath
        #               fails, another recovery starts, the Nav2 goal ends, or
        #               arrival_timeout)
        #   'chosen'  - as soon as a recovery waypoint is chosen
        self.success_beep_on = self.declare_parameter(
            'success_beep_on', 'arrival'
        ).value
        if self.success_beep_on not in ('arrival', 'chosen'):
            raise ValueError(
                f"success_beep_on must be 'arrival' or 'chosen', "
                f"not {self.success_beep_on!r}"
            )
        # Give up waiting for arrival after this long (s)
        self.arrival_timeout = self.declare_parameter(
            'arrival_timeout', 60.0
        ).value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        # The buzzer plays one sound at a time: a new request cuts off the one
        # playing, and each sound lasts almost 2 s. Every beep starts at least
        # this long after the previous one (s); a later one is delayed.
        self.min_beep_gap = self.declare_parameter('min_beep_gap', 2.5).value
        self.next_beep_time = -math.inf  # earliest time.monotonic() for a beep
        # Keep the robot still until the activation sound has played (the
        # /divert_robot reply waits). The behavior tree's server_timeout on
        # BeepAndDivertBridge must cover this plus planning (plan_budget).
        self.wait_for_activation_beep = self.declare_parameter(
            'wait_for_activation_beep', True
        ).value
        self.beep_lock = threading.Lock()

        # Path-aware selection: plan to candidates with Nav2's planner and
        # pick the shortest real path, not the nearest straight-line point.
        # Must match planner_id in the behavior tree.
        self.planner_id = self.declare_parameter('planner_id', 'GridBased').value
        # Max time for one plan, and for planning all candidates (seconds).
        # Keep plan_budget well under server_timeout of BeepAndDivertBridge
        # in the behavior tree (5 s), or the BT gives up on /divert_robot.
        self.plan_timeout = self.declare_parameter('plan_timeout', 1.0).value
        self.plan_budget = self.declare_parameter('plan_budget', 3.0).value
        # A path must end this close to the candidate to count as reaching it.
        # NavFn plans to the nearest reachable point within its tolerance
        # (0.5 m), so a path can "succeed" without reaching the candidate.
        self.reach_tolerance = self.declare_parameter(
            'reach_tolerance', 0.25
        ).value

        self.frame_id, self.min_distance, self.candidates = (
            self.load_candidates(candidates_file)
        )

        # A candidate this close is within Nav2's goal tolerance (0.15 m) of
        # the robot, and the diversion barely moves it
        if self.min_distance < 0.3:
            self.get_logger().warn(
                f'min_distance ({self.min_distance} m) is small: the robot may '
                f'divert to a point it is almost standing on'
            )

        # Latest robot position (x, y) in the map frame, from AMCL
        self.robot_xy = None

        # ID of the Nav2 goal currently executing, from the action status
        self.active_goal_id = None

        # Last diversion: (goal ID, candidate index). If Nav2 asks for another
        # recovery during the same goal, that candidate didn't fix the problem.
        self.last_divert = None

        # Candidates that failed during the current goal
        self.failed = set()

        # Recovery waiting for the robot to arrive, for the success beep:
        # dict(goal_id, index, x, y, start) or None
        self.pending_arrival = None
        self.pending_lock = threading.Lock()
        # Latest state of each Nav2 goal, by goal ID
        self.goal_states = {}

        # Robot position for the arrival check: map -> base_link from TF
        # (/amcl_pose only updates after the robot moves 0.25 m, so it lags)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.check_arrival)

        # TurtleBot3 OpenCR buzzer service client
        self.sound_client = self.create_client(
            Sound,
            '/sound'
        )

        # AMCL and the action status topic both publish as transient local,
        # so the latest message arrives even if it was sent before we started.
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            latched
        )

        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.goal_status_callback,
            latched
        )

        # Controller goals: the first FollowPath after a diversion is the
        # drive to the recovery waypoint; its result says whether it got there
        self.follow_states = {}
        self.create_subscription(
            GoalStatusArray,
            '/follow_path/_action/status',
            self.follow_status_callback,
            latched
        )

        # The recovery callback waits for planner results, so it and the
        # planner client go in a reentrant group, and main() uses a
        # multithreaded executor. With one thread, waiting would block the
        # thread that delivers those results.
        self.planning_group = ReentrantCallbackGroup()

        self.planner_client = ActionClient(
            self,
            ComputePathToPose,
            '/compute_path_to_pose',
            callback_group=self.planning_group
        )

        # Recovery service called by the C++ Behavior Tree bridge
        self.srv = self.create_service(
            DivertRobot,
            '/divert_robot',
            self.handle_recovery,
            callback_group=self.planning_group
        )

        self.get_logger().info(
            'Recovery Python Server listening on /divert_robot'
        )

        if self.sound_client.service_is_ready():
            self.get_logger().info(
                'TurtleBot3 sound service available'
            )
        else:
            self.get_logger().warn(
                'TurtleBot3 sound service not currently available'
            )

    def load_candidates(self, path):
        """Read the frame, minimum distance and candidate points from YAML."""

        with open(path, 'r') as f:
            config = yaml.safe_load(f)

        frame_id = config.get('frame_id', 'map')
        min_distance = float(config.get('min_distance', 0.3))
        candidates = [
            (float(c['x']), float(c['y'])) for c in config['candidates']
        ]

        if not candidates:
            raise ValueError(f'No recovery candidates in {path}')

        self.get_logger().info(
            f'Loaded {len(candidates)} recovery candidates from {path}'
        )
        return frame_id, min_distance, candidates

    def amcl_pose_callback(self, msg):
        position = msg.pose.pose.position
        self.robot_xy = (position.x, position.y)

    def goal_status_callback(self, msg):
        # The goal that is executing now. Nav2 runs one NavigateToPose goal
        # at a time, so take the last one if there are several.
        active = [
            s for s in msg.status_list
            if s.status in (GoalStatus.STATUS_ACCEPTED,
                            GoalStatus.STATUS_EXECUTING)
        ]
        if active:
            self.active_goal_id = bytes(active[-1].goal_info.goal_id.uuid).hex()
        self.goal_states = {
            bytes(s.goal_info.goal_id.uuid).hex(): s.status
            for s in msg.status_list
        }

    def follow_status_callback(self, msg):
        self.follow_states = {
            bytes(s.goal_info.goal_id.uuid).hex(): s.status
            for s in msg.status_list
        }

    def robot_position(self):
        """Robot (x, y) in the map frame: TF, else the last /amcl_pose."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.frame_id, self.base_frame, Time()
            ).transform.translation
            return (t.x, t.y)
        except TransformException:
            return self.robot_xy

    def check_arrival(self):
        """Success beep once the drive to the recovery waypoint succeeds."""
        with self.pending_lock:
            pending = self.pending_arrival
            if pending is None:
                return

            name = f"candidate {pending['index']}"
            state = self.goal_states.get(pending['goal_id'])
            if state in GOAL_DONE:
                self.pending_arrival = None
                self.get_logger().warn(
                    f'Recovery abandoned: the Nav2 goal ended before the '
                    f'robot reached {name}; no success beep'
                )
                return
            if time.monotonic() - pending['start'] > self.arrival_timeout:
                self.pending_arrival = None
                self.get_logger().warn(
                    f'Recovery abandoned: {name} not reached within '
                    f'{self.arrival_timeout:.0f} s; no success beep'
                )
                return

            # The diversion's FollowPath: the first controller goal that
            # wasn't there when the waypoint was chosen
            if pending['follow_id'] is None:
                new = [g for g in self.follow_states if g not in pending['known']]
                if not new:
                    return
                pending['follow_id'] = new[0]

            state = self.follow_states.get(pending['follow_id'])
            if state in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
                self.pending_arrival = None
                self.get_logger().warn(
                    f'Recovery abandoned: the drive to {name} failed; '
                    f'no success beep'
                )
                return
            if state != GoalStatus.STATUS_SUCCEEDED:
                return

            self.pending_arrival = None

        position = self.robot_position()
        away = (
            f', {math.dist(position, (pending["x"], pending["y"])):.2f} m away'
            if position is not None else ''
        )
        self.get_logger().info(
            f'Reached recovery waypoint ({name}{away}); recovery successful'
        )
        self.beep(self.success_sound)

    def beep(self, value=SOUND_ERROR):
        """Trigger the TurtleBot3 buzzer, min_beep_gap after the last beep.

        Returns the time.monotonic() at which the sound starts.
        """

        with self.beep_lock:
            now = time.monotonic()
            start = max(now, self.next_beep_time)
            self.next_beep_time = start + self.min_beep_gap

        if start > now:
            # Let the sound playing now finish first
            def fire():
                timer.cancel()
                self.destroy_timer(timer)
                self.send_beep(value)

            timer = self.create_timer(start - now, fire)
            self.get_logger().info(
                f'Sound {value} in {start - now:.1f} s, after the one playing'
            )
            return start

        self.send_beep(value)
        return start

    def send_beep(self, value):
        """Send the sound request to the TurtleBot3 buzzer asynchronously."""

        if not self.sound_client.service_is_ready():
            self.get_logger().warn(
                'Cannot beep: /sound service unavailable'
            )
            return

        sound_request = Sound.Request()
        sound_request.value = value

        try:
            future = self.sound_client.call_async(sound_request)
            future.add_done_callback(self.sound_response_callback)

            self.get_logger().info(
                f'Sent sound {value} request to TurtleBot3'
            )

        except Exception as exc:
            self.get_logger().error(
                f'Failed to send sound request: {exc}'
            )

    def sound_response_callback(self, future):
        """Process the asynchronous sound service response."""

        try:
            result = future.result()

            if result.success:
                self.get_logger().info(
                    'TurtleBot3 buzzer activated successfully'
                )
            else:
                self.get_logger().warn(
                    f'TurtleBot3 buzzer failed: {result.message}'
                )

        except Exception as exc:
            self.get_logger().error(
                f'Sound service call failed: {exc}'
            )

    def update_failed_candidates(self):
        """Exclude the last candidate if it didn't fix the current goal."""

        goal_id = self.active_goal_id

        if goal_id is None:
            self.get_logger().warn(
                'No Nav2 goal status received; treating this as the same '
                'goal as the last recovery'
            )

        if self.last_divert is None:
            return

        last_goal_id, last_index = self.last_divert

        if goal_id == last_goal_id:
            # Stuck again on the same goal after diverting to this candidate
            self.failed.add(last_index)
            x, y = self.candidates[last_index]
            self.get_logger().warn(
                f'Candidate {last_index} ({x:.2f}, {y:.2f}) did not fix the '
                f'problem; excluding it for this goal'
            )
        else:
            # New goal: every candidate gets another chance
            self.failed.clear()

    def choose_candidate(self):
        """Return the index of the best candidate, or None if none is usable.

        Best = shortest planned path from the robot. Candidates that failed
        this goal, or are closer than min_distance, are skipped.
        """

        usable = [
            i for i in range(len(self.candidates)) if i not in self.failed
        ]

        # Live position from TF: /amcl_pose can lag by 0.25 m, enough to
        # pick a candidate the robot is already standing on
        robot = self.robot_position()
        if robot is None:
            self.get_logger().warn(
                'Robot position unknown; using the first usable '
                'candidate in the file'
            )
            return usable[0] if usable else None

        def distance(i):
            return math.dist(robot, self.candidates[i])

        # Nearest first, so the search below can stop early
        far_enough = sorted(
            (i for i in usable if distance(i) >= self.min_distance),
            key=distance
        )
        if not far_enough:
            return None

        if not self.planner_client.server_is_ready():
            self.get_logger().warn(
                'Planner not available; using the nearest candidate in a '
                'straight line'
            )
            return far_enough[0]

        best, best_length = None, math.inf
        start = time.monotonic()

        for i in far_enough:
            # A path is never shorter than the straight line, so no
            # remaining candidate can beat the best path found so far.
            if distance(i) >= best_length:
                break
            if time.monotonic() - start > self.plan_budget:
                self.get_logger().warn('Planning budget used up')
                break

            length = self.path_length(i)
            x, y = self.candidates[i]
            if length is None:
                self.get_logger().info(
                    f'  candidate {i} ({x:.2f}, {y:.2f}): unreachable'
                )
                continue

            self.get_logger().info(
                f'  candidate {i} ({x:.2f}, {y:.2f}): path {length:.2f} m '
                f'(straight line {distance(i):.2f} m)'
            )
            if length < best_length:
                best, best_length = i, length

        return best

    def wait_for(self, future, timeout):
        """Block until the future is done; False on timeout."""
        if future.done():
            return True
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        return done.wait(timeout) or future.done()

    def path_length(self, index):
        """Length of the planned path to a candidate, or None if unreachable."""

        x, y = self.candidates[index]
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = self.frame_id
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.w = 1.0
        goal.planner_id = self.planner_id
        goal.use_start = False  # plan from the robot's current pose

        send_future = self.planner_client.send_goal_async(goal)
        if not self.wait_for(send_future, self.plan_timeout):
            return None
        handle = send_future.result()
        if not handle.accepted:
            return None

        result_future = handle.get_result_async()
        if not self.wait_for(result_future, self.plan_timeout):
            handle.cancel_goal_async()
            return None

        result = result_future.result()
        poses = result.result.path.poses
        if result.status != GoalStatus.STATUS_SUCCEEDED or not poses:
            return None

        # NavFn may stop short of an unreachable goal (its tolerance)
        end = poses[-1].pose.position
        if math.dist((end.x, end.y), (x, y)) > self.reach_tolerance:
            return None

        return sum(
            math.dist(
                (a.pose.position.x, a.pose.position.y),
                (b.pose.position.x, b.pose.position.y)
            )
            for a, b in zip(poses, poses[1:])
        )

    def handle_recovery(self, request, response):
        """
        Called when Nav2 triggers the custom recovery behavior.

        1. Trigger the TurtleBot3 buzzer.
        2. Compute a recovery waypoint.
        3. Return the waypoint to the C++ BT bridge if one is available.
        4. Trigger a different Turtlebot3 buzzer if successful: once the
           robot reaches the waypoint (success_beep_on: arrival, default),
           or right away (success_beep_on: chosen).
        """

        self.get_logger().warn(
            'Nav2 stalled! Sounding alarm and calculating diversion route...'
        )

        # --------------------------------------------------
        # STEP 1: Trigger the TurtleBot3 buzzer
        # --------------------------------------------------

        activation_start = self.beep(SOUND_ERROR)

        # A new recovery replaces one still on its way to its waypoint
        with self.pending_lock:
            if self.pending_arrival is not None:
                self.get_logger().warn(
                    f"Recovery abandoned: candidate "
                    f"{self.pending_arrival['index']} not reached before "
                    f"this recovery; no success beep"
                )
                self.pending_arrival = None

        # --------------------------------------------------
        # STEP 2: Compute recovery waypoint
        # --------------------------------------------------

        # Candidate with the shortest planned path from the robot, at least
        # min_distance away, that hasn't already failed during this goal.
        self.update_failed_candidates()
        index = self.choose_candidate()

        response.recovery_pose.header.frame_id = self.frame_id
        response.recovery_pose.header.stamp = (
            self.get_clock().now().to_msg()
        )
        response.recovery_pose.pose.orientation.w = 1.0

        # --------------------------------------------------
        # STEP 3: Return recovery waypoint
        # --------------------------------------------------

        if index is None:
            response.success = False
            self.get_logger().error(
                f'No usable recovery candidate (robot at {self.robot_position()}, '
                f'{len(self.failed)} of {len(self.candidates)} excluded)'
            )
            return response

        # Hold the reply until the activation sound has played: the behavior
        # tree waits for it, so the robot stays still during the sound
        if self.wait_for_activation_beep:
            wait = activation_start + self.min_beep_gap - time.monotonic()
            if wait > 0:
                self.get_logger().info(
                    f'Waiting {wait:.1f} s for the activation sound'
                )
                time.sleep(wait)

        recovery_x, recovery_y = self.candidates[index]
        response.recovery_pose.pose.position.x = recovery_x
        response.recovery_pose.pose.position.y = recovery_y
        response.recovery_pose.pose.position.z = 0.0
        response.success = True

        self.last_divert = (self.active_goal_id, index)

        self.get_logger().info(
            f'Recovery waypoint generated: candidate {index} '
            f'({recovery_x:.2f}, {recovery_y:.2f})'
        )

        # ---------------------------------------------------
        # STEP 4: Trigger a different TurtleBot3 buzzer if a
        # recovery point was found
        # ---------------------------------------------------

        if self.success_beep_on == 'chosen':
            self.beep(self.success_sound)
        else:
            # Beep later, from check_arrival, once the robot gets there
            with self.pending_lock:
                self.pending_arrival = dict(
                    goal_id=self.active_goal_id, index=index,
                    x=recovery_x, y=recovery_y, start=time.monotonic(),
                    known=set(self.follow_states), follow_id=None,
                )

        return response


def main(args=None):
    rclpy.init(args=args)

    node = StudentRecoveryServer()

    # Multithreaded so the recovery callback can wait for planner results
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
