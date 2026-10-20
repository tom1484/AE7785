#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 4

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    go_to_goal

Subscribes:
    /odom
      nav_msgs/msg/Odometry
      Needed to express /goal (odom frame) relative to the robot.
    /goal
      geometry_msgs/msg/PointStamped
    /avoidance_state
      team85_lab4_interfaces/msg/AvoidanceState
    TODO: Avoidance output (vector or twist) from obstacle_state.

Publishes:
    /cmd_vel
      geometry_msgs/msg/Twist

Combines the goal and the avoidance output into the steering of the robot.
"""

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Twist
from team85_lab4_interfaces.msg import AvoidanceState

import numpy as np

from team85_lab4.verbose_logger import VerboseLogger


CONTROL_PERIOD = 0.05  # Seconds between control updates (20 Hz)


def yaw_from_quaternion(q) -> float:
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def rotation(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


class GoToGoal(Node):

    def __init__(self):
        super().__init__("go_to_goal")

        self.log = VerboseLogger(self)
        # TODO: Controller parameters (gains, speed limits, tolerances).

        # Latest odometry (odom frame)
        self.position: Optional[np.ndarray] = None
        self.yaw: Optional[float] = None

        # Latest goal (odom frame) and avoidance state
        self.goal: Optional[np.ndarray] = None
        self.avoidance_state: Optional[int] = None

        self.odom_subscription = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.goal_subscription = self.create_subscription(
            PointStamped, "/goal", self.goal_callback, 10
        )
        self.state_subscription = self.create_subscription(
            AvoidanceState, "/avoidance_state", self.state_callback, 10
        )
        # TODO: Subscribe to the avoidance output of obstacle_state.

        self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(CONTROL_PERIOD, self.control_loop)

        self.log.basic("go_to_goal started")

    # =============================================================
    # Callbacks
    # =============================================================

    def odom_callback(self, odom: Odometry):
        pose = odom.pose.pose
        self.position = np.array([pose.position.x, pose.position.y])
        self.yaw = yaw_from_quaternion(pose.orientation)

    def goal_callback(self, msg: PointStamped):
        self.goal = np.array([msg.point.x, msg.point.y])

    def state_callback(self, msg: AvoidanceState):
        self.avoidance_state = msg.state

    # =============================================================
    # Control
    # =============================================================

    def control_loop(self):
        if self.position is None or self.goal is None:
            return

        # Goal relative to the robot (robot frame: x forward, y left)
        goal_vector = rotation(self.yaw).T @ (self.goal - self.position)

        # TODO: Combine goal_vector with the avoidance state/output into the
        #       target direction, compute the steering, and publish it:
        #           self.cmd_publisher.publish(twist)
        #       Also stop (and handle stale inputs) when the goal or avoidance
        #       messages stop arriving.

        self.log.continuous(
            f"goal_vector=({goal_vector[0]:.3f}, {goal_vector[1]:.3f}) "
            f"avoidance_state={self.avoidance_state}",
            throttle_duration_sec=0.5,
        )

    def stop_robot(self):
        self.cmd_publisher.publish(Twist())


def main(args=None):
    # Handle Ctrl+C ourselves, so the robot can be stopped before shutdown
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None

    try:
        node = GoToGoal()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if node.context.ok():
                node.stop_robot()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
