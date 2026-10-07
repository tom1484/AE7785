#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 3

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    chase_object

Subscribes:
    /object_position
      geometry_msgs/msg/Point

Coordinate convention:
    x = right positive
    y = forward positive
    z = up positive

Publishes:
    /cmd_vel
      geometry_msgs/msg/Twist

Controller:
    P controller for linear motion
    P controller for angular motion
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist


class ChaseObject(Node):

    def __init__(self):
        super().__init__("chase_object")

        # =========================================================
        # Controller parameters
        # =========================================================

        # Desired horizontal distance from robot to object
        self.desired_distance = 0.60  # meters

        # Proportional controller gains
        self.kp_linear = 0.40
        self.kp_angular = 1.20

        # Deadbands
        self.distance_tolerance = 0.05  # meters
        self.angle_tolerance = 0.05  # radians

        # Maximum commanded speeds
        self.max_linear_speed = 0.15  # m/s
        self.max_angular_speed = 0.80  # rad/s

        # Stop if object-position data stops arriving
        self.timeout = 0.50  # seconds

        # Latest target data
        self.object_x = None
        self.object_y = None
        self.object_z = None

        self.last_point_time = None

        # =========================================================
        # ROS interfaces
        # =========================================================

        self.position_sub = self.create_subscription(
            Point, "/object_position", self.position_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Controller runs at 20 Hz
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("chase_object started")
        self.get_logger().info(f"Desired distance = {self.desired_distance:.2f} m")
        self.get_logger().info("Coordinate system: +x right, +y forward, +z up")

    # =============================================================
    # Target-position callback
    # =============================================================

    def position_callback(self, msg):

        # Reject invalid numerical data
        if not (math.isfinite(msg.x) and math.isfinite(msg.y) and math.isfinite(msg.z)):
            self.object_x = None
            self.object_y = None
            self.object_z = None
            self.get_logger().warning("Invalid object position received")
            return

        self.object_x = float(msg.x)
        self.object_y = float(msg.y)
        self.object_z = float(msg.z)

        self.last_point_time = self.get_clock().now()

    # =============================================================
    # Controller
    # =============================================================

    def control_loop(self):

        # No target received yet
        if (
            self.object_x is None
            or self.object_y is None
            or self.object_z is None
            or self.last_point_time is None
        ):
            self.stop_robot()
            return

        # ---------------------------------------------------------
        # Watchdog
        # ---------------------------------------------------------

        now = self.get_clock().now()
        point_age = (now - self.last_point_time).nanoseconds / 1e9
        if point_age > self.timeout:
            self.stop_robot()
            return

        x = self.object_x
        y = self.object_y
        z = self.object_z

        # ---------------------------------------------------------
        # Convert XYZ to distance and bearing
        #
        # Planar distance:
        #
        #     distance = sqrt(x^2 + y^2)
        #
        # Bearing:
        #
        #     angle = atan2(x, y)
        #
        # because:
        #     +y = forward
        #     +x = right
        #
        # Therefore:
        #     right -> positive angle
        #     left  -> negative angle
        # ---------------------------------------------------------

        distance = math.sqrt(x * x + y * y)
        angle = math.atan2(x, y)

        # ---------------------------------------------------------
        # Errors
        # ---------------------------------------------------------

        distance_error = distance - self.desired_distance
        angle_error = angle

        # ---------------------------------------------------------
        # Linear P controller
        # ---------------------------------------------------------

        if abs(distance_error) <= self.distance_tolerance:
            linear_cmd = 0.0
        else:
            linear_cmd = self.kp_linear * distance_error

        # ---------------------------------------------------------
        # Angular P controller
        #
        # Our angle:
        #   positive = target is RIGHT
        #
        # ROS angular.z:
        #   positive = turn LEFT
        #
        # Therefore the minus sign is required.
        # ---------------------------------------------------------

        if abs(angle_error) <= self.angle_tolerance:
            angular_cmd = 0.0
        else:
            angular_cmd = -self.kp_angular * angle_error

        # ---------------------------------------------------------
        # Velocity saturation
        # ---------------------------------------------------------

        linear_cmd = max(-self.max_linear_speed, min(self.max_linear_speed, linear_cmd))
        angular_cmd = max(
            -self.max_angular_speed, min(self.max_angular_speed, angular_cmd)
        )

        # ---------------------------------------------------------
        # Publish Twist
        # ---------------------------------------------------------

        cmd = Twist()
        cmd.linear.x = float(linear_cmd)
        cmd.angular.z = float(angular_cmd)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"xyz=({x:.2f}, {y:.2f}, {z:.2f}) | "
            f"distance={distance:.2f} m, "
            f"angle={angle:.2f} rad | "
            f"ed={distance_error:.2f}, "
            f"e_angle={angle_error:.2f} | "
            f"v={linear_cmd:.2f}, "
            f"w={angular_cmd:.2f}",
            throttle_duration_sec=0.5,
        )

    # =============================================================
    # Stop robot
    # =============================================================

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ChaseObject()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
