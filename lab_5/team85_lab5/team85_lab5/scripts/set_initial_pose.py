#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 5.

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    set_initial_pose

Publishes an initial pose on /initialpose every second until AMCL reports a
pose near it on /amcl_pose, then exits. AMCL ignores /initialpose until it is
active, so a single message sent at startup can be lost.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('set_initial_pose')

        self.x = float(self.declare_parameter('x', 0.0).value)
        self.y = float(self.declare_parameter('y', 0.0).value)
        self.yaw = float(self.declare_parameter('yaw', 0.0).value)
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        # AMCL's pose must be this close to the requested one to count
        self.tolerance = self.declare_parameter('tolerance', 0.3).value

        self.done = False
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
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
        self.timer = self.create_timer(1.0, self.publish)
        self.get_logger().info(
            f'Setting initial pose ({self.x:.2f}, {self.y:.2f}), yaw '
            f'{math.degrees(self.yaw):.0f} deg; waiting for AMCL...'
        )

    def publish(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.pose.covariance[0] = 0.25       # x variance (m^2)
        msg.pose.covariance[7] = 0.25       # y variance
        msg.pose.covariance[35] = 0.07      # yaw variance (rad^2)
        self.publisher.publish(msg)

    def amcl_pose_callback(self, msg):
        position = msg.pose.pose.position
        if math.hypot(position.x - self.x, position.y - self.y) <= self.tolerance:
            self.get_logger().info(
                f'AMCL accepted the initial pose: ({position.x:.2f}, '
                f'{position.y:.2f})'
            )
            self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
