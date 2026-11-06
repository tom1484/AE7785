#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 5.

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    waypoint_navigator

Reads goal points from a waypoints file and sends them to Nav2 one at a time
with the NavigateToPose action, waiting for each to finish before the next.

Waypoints file: one waypoint per line, "x y" or "x y yaw" (m, m, rad) in the
map frame. Blank lines and anything after "#" are ignored. Without a yaw, the
robot faces the direction it traveled (from the previous waypoint, or from
its start position for the first one).

Start it after Nav2 is up. It waits for the initial pose (AMCL publishes
/amcl_pose only once the pose is set, e.g. with 2D Pose Estimate in RViz).
"""

import math
import os
import signal
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}


def load_waypoints(path):
    """Read "x y" or "x y yaw" per line; yaw is None when not given."""
    waypoints = []
    with open(path) as f:
        for number, line in enumerate(f, start=1):
            fields = line.split('#', 1)[0].split()
            if not fields:
                continue
            if len(fields) not in (2, 3):
                raise ValueError(
                    f'{path}:{number}: expected "x y" or "x y yaw", '
                    f'got {line.strip()!r}'
                )
            x, y = float(fields[0]), float(fields[1])
            yaw = float(fields[2]) if len(fields) == 3 else None
            waypoints.append((x, y, yaw))
    return waypoints


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        default_file = os.path.join(
            get_package_share_directory('team85_lab5'), 'wayPoints.txt'
        )
        waypoints_file = self.declare_parameter(
            'waypoints_file', default_file
        ).value
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        self.hold_time = self.declare_parameter('hold_time', 2.0).value
        self.retries = self.declare_parameter('retries', 1).value
        self.wait_for_initial_pose = self.declare_parameter(
            'wait_for_initial_pose', True
        ).value
        # Keep resending a rejected goal for this long (s) before giving up
        self.reject_timeout = self.declare_parameter('reject_timeout', 60.0).value
        # Resend a goal if Nav2 doesn't answer the request within this (s)
        self.response_timeout = self.declare_parameter('response_timeout', 10.0).value

        self.waypoints = load_waypoints(waypoints_file)
        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints from {waypoints_file}: '
            + ', '.join(f'({x:.2f}, {y:.2f})' for x, y, _ in self.waypoints)
        )

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # AMCL publishes as transient local, so the latest pose arrives even
        # if it was sent before this node started.
        self.robot_xy = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
        )

        self.index = 0          # waypoint being driven to
        self.attempt = 0        # resends of the current waypoint
        self.goal_handle = None
        self.goal_start = None
        self.results = []       # (index, status name, seconds)
        self.done = False
        self.hold_timer = None
        self.resend_timer = None
        self.rejected_since = None
        self.request_id = 0
        self.response_timer = None

        if not self.waypoints:
            self.get_logger().error('No waypoints; nothing to do')
            self.done = True
            return

        self.start_timer = self.create_timer(1.0, self.start_when_ready)

    def amcl_pose_callback(self, msg):
        position = msg.pose.pose.position
        self.robot_xy = (position.x, position.y)

    def start_when_ready(self):
        """Wait for Nav2 and the initial pose, then send the first goal."""
        if not self.client.server_is_ready():
            self.get_logger().info(
                'Waiting for Nav2 (navigate_to_pose)...',
                throttle_duration_sec=5.0
            )
            return
        if self.wait_for_initial_pose and self.robot_xy is None:
            self.get_logger().info(
                'Waiting for the initial pose on /amcl_pose (set it with '
                '2D Pose Estimate in RViz)...',
                throttle_duration_sec=5.0
            )
            return

        self.start_timer.cancel()
        self.run_start = time.monotonic()
        self.send_goal()

    def goal_yaw(self):
        """Given yaw, or the direction of travel to the current waypoint."""
        x, y, yaw = self.waypoints[self.index]
        if yaw is not None:
            return yaw
        if self.index > 0:
            prev_x, prev_y, _ = self.waypoints[self.index - 1]
        elif self.robot_xy is not None:
            prev_x, prev_y = self.robot_xy
        else:
            return 0.0
        if math.hypot(x - prev_x, y - prev_y) < 1e-3:
            return 0.0
        return math.atan2(y - prev_y, x - prev_x)

    def send_goal(self, log=True):
        x, y, _ = self.waypoints[self.index]
        yaw = self.goal_yaw()

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        retry = f' (retry {self.attempt}/{self.retries})' if self.attempt else ''
        if log:
            self.get_logger().info(
                f'Waypoint {self.index + 1}/{len(self.waypoints)}: going to '
                f'({x:.2f}, {y:.2f}), yaw {math.degrees(yaw):.0f} deg{retry}'
            )

        if self.attempt == 0 and log:
            self.goal_start = time.monotonic()
        # Each request gets an ID, so a late answer to an old one is ignored
        self.request_id += 1
        request_id = self.request_id
        future = self.client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(
            lambda f: self.goal_response_callback(f, request_id)
        )
        self.response_timer = self.create_timer(
            self.response_timeout, lambda: self.response_timed_out(request_id)
        )

    def response_timed_out(self, request_id):
        """Resend if Nav2 never answered (e.g. it restarted meanwhile)."""
        self.destroy_timer(self.response_timer)
        if request_id != self.request_id:
            return
        self.get_logger().warn(
            f'No answer from Nav2 within {self.response_timeout:.0f} s; '
            f'resending the goal'
        )
        self.send_goal(log=False)

    def goal_response_callback(self, future, request_id):
        handle = future.result()
        if request_id != self.request_id:
            # Answer to a request we already gave up on
            if handle.accepted:
                handle.cancel_goal_async()
            return
        self.destroy_timer(self.response_timer)
        self.request_id += 1  # answered; the timeout no longer applies
        if not handle.accepted:
            self.handle_rejection()
            return
        self.rejected_since = None
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.result_callback)

    def handle_rejection(self):
        """Resend later if Nav2 isn't active yet; give up after a while.

        bt_navigator's action server exists before Nav2 finishes starting, and
        rejects goals until then (e.g. when launched together with Nav2).
        """
        now = time.monotonic()
        if self.rejected_since is None:
            self.rejected_since = now
        if now - self.rejected_since < self.reject_timeout:
            self.get_logger().warn(
                'Nav2 rejected the goal (not active yet?); resending in 2 s',
                throttle_duration_sec=10.0
            )
            self.resend_timer = self.create_timer(2.0, self.resend_after_rejection)
            return
        self.get_logger().error(
            f'Nav2 kept rejecting the goal for {self.reject_timeout:.0f} s'
        )
        self.rejected_since = None
        self.handle_result(GoalStatus.STATUS_ABORTED)

    def resend_after_rejection(self):
        self.resend_timer.cancel()
        self.destroy_timer(self.resend_timer)
        self.send_goal(log=False)

    def feedback_callback(self, msg):
        feedback = msg.feedback
        self.get_logger().info(
            f'  {feedback.distance_remaining:.2f} m to go, '
            f'{feedback.number_of_recoveries} recoveries',
            throttle_duration_sec=5.0
        )

    def result_callback(self, future):
        self.goal_handle = None
        self.handle_result(future.result().status)

    def handle_result(self, status):
        name = STATUS_NAMES.get(status, f'status {status}')
        elapsed = time.monotonic() - self.goal_start

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Waypoint {self.index + 1} reached in {elapsed:.1f} s'
            )
            self.results.append((self.index, name, elapsed))
            # Stay at the waypoint for a moment, then go to the next one
            self.hold_timer = self.create_timer(self.hold_time, self.after_hold)
            return

        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'Waypoint {self.index + 1} was canceled')
            self.results.append((self.index, name, elapsed))
            self.finish()
            return

        # Aborted, e.g. recovery couldn't help. A resend starts a new Nav2
        # goal, so the recovery node gives every candidate another chance.
        if self.attempt < self.retries:
            self.attempt += 1
            self.get_logger().warn(
                f'Waypoint {self.index + 1} {name}; sending it again'
            )
            self.send_goal()
            return

        self.get_logger().error(
            f'Waypoint {self.index + 1} {name} after {self.attempt + 1} '
            f'attempts; skipping it'
        )
        self.results.append((self.index, name, elapsed))
        self.next_waypoint()

    def after_hold(self):
        self.hold_timer.cancel()
        self.destroy_timer(self.hold_timer)
        self.hold_timer = None
        self.next_waypoint()

    def next_waypoint(self):
        self.index += 1
        self.attempt = 0
        if self.index < len(self.waypoints):
            self.send_goal()
        else:
            self.finish()

    def finish(self):
        reached = sum(1 for _, name, _ in self.results if name == 'SUCCEEDED')
        total = time.monotonic() - self.run_start
        self.get_logger().info(
            f'Done: {reached}/{len(self.waypoints)} waypoints reached in '
            f'{total:.1f} s'
        )
        for index, name, elapsed in self.results:
            x, y, _ = self.waypoints[index]
            self.get_logger().info(
                f'  {index + 1}: ({x:.2f}, {y:.2f}) {name} in {elapsed:.1f} s'
            )
        self.done = True

    def cancel_active_goal(self):
        """Stop the robot: cancel the goal Nav2 is executing, if any."""
        if self.goal_handle is None:
            return
        self.get_logger().warn('Canceling the active goal')
        future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done() and future.result().goals_canceling:
            self.get_logger().info('Goal canceled; Nav2 is stopping the robot')
        else:
            self.get_logger().error(
                'Nav2 did not confirm the cancel; the robot may still be moving'
            )


def main(args=None):
    # rclpy's own Ctrl-C handler shuts ROS down before our code runs, which
    # makes canceling the goal impossible and leaves the robot driving.
    # Handle Ctrl-C (and SIGTERM) here instead, and shut down ourselves.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    # Set explicitly: a process started in the background inherits Ctrl-C
    # as ignored, and Python then never raises KeyboardInterrupt.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = WaypointNavigator()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        if rclpy.ok():
            node.cancel_active_goal()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
