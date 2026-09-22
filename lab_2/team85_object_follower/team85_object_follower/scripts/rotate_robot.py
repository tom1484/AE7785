#!/usr/bin/env python3

from typing import cast

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist


ANGULAR_VELOCITY = 0.3
POS_ERROR_THRESHOLD = 0.03


class RotateRobot(Node):

    def __init__(self):
        super().__init__("rotate_robot")

        self.pos_subscription = self.create_subscription(
            Point, "/detection", self.pos_callback, 10
        )
        self.twist_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def pos_callback(self, pos: Point):
        error = pos.x - 0.5
        angular_velocity = 0.0
        if abs(error) > POS_ERROR_THRESHOLD:
            if error < 0.0:
                angular_velocity = ANGULAR_VELOCITY
            if error > 0.0:
                angular_velocity = -ANGULAR_VELOCITY

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_velocity

        self.twist_publisher.publish(twist)

        # self.get_logger().info("Unknown error happened in object detector.")


def main(args=None):
    rclpy.init(args=args)

    rotate_robot = RotateRobot()
    rclpy.spin(rotate_robot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rotate_robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
