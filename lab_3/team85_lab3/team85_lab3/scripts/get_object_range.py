#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 3

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    get_object_range

Subscribes:
    /object_angle
      std_msgs/msg/Float32
    /scan
      sensor_msgs/msg/LaserScan

Publishes:
    /object_position
      geometry_msgs/msg/Point
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data

import numpy as np


# Frame of the bot:
#   x: forward
#   y: left
#   z: up

# Frame of object position
#   x: right
#   y: forward
#   z: down

# Offset from PiCam to LiDAR
#   x: -5.6 cm (estimated)
#   y:  0.0 cm
#   z:  4.0 cm (estimated)
SCAN_OFFSET_X = -0.056
SCAN_OFFSET_Y =  0.000
SCAN_OFFSET_Z =  0.040

SCAN_ANGLE_MIN = -1.0
SCAN_ANGLE_MAX =  1.0

class GetObjectRange(Node):

    def __init__(self):
        super().__init__("get_object_range")

        self.angle_subscription = self.create_subscription(
            Float32, "/object_angle", self.angle_callback, 10
        )
        self.laser_subscription = self.create_subscription(
            LaserScan, "/scan", self.laser_callback, qos_profile_sensor_data
        )
        self.object_publisher = self.create_publisher(Point, "/object_position", 10)

        self.scan = None

    def wrap_angle(self, angle: float):
        if angle > np.pi:
            return angle - 2 * np.pi
        return angle

    def angle_callback(self, angle: Float32):
        if self.scan is None:
            return

        # Create the line with an angle to the front line
        # We will find the scan point closest to this line
        angle: float = angle.data
        norm = np.array([[np.sin(angle)], [np.cos(angle)]], dtype=float)

        # Build the list of all scanned points on xy plane
        scan = self.scan
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, ranges.shape[0])
        polar_points = np.stack([angles, ranges]).T

        # Filter points
        valid = np.zeros((ranges.shape[0],), dtype=bool)
        for i, (angle, dis) in enumerate(polar_points):
            angle = self.wrap_angle(angle)
            valid[i] = (dis >= scan.range_min and dis <= scan.range_max) and \
                       (angle >= SCAN_ANGLE_MIN and angle <= SCAN_ANGLE_MAX)
        
        polar_points = polar_points[valid]

        # Convert polar points to Cartesian points
        points = np.zeros(polar_points.shape)
        for i, (angle, dis) in enumerate(polar_points):
            points[i, 0] = dis * np.cos(angle) + SCAN_OFFSET_X
            points[i, 1] = dis * np.sin(angle) + SCAN_OFFSET_Y

        # Compute each point's distance to the line
        # Then find the one of mimimal distance
        distances = np.abs((points @ norm).T[0])
        idx = np.argmin(distances)
        x, y = points[idx]

        # Convert to object frame and publish
        object_position = Point()
        object_position.x = -y
        object_position.y = x
        self.object_publisher.publish(object_position)

    def laser_callback(self, scan: LaserScan):
        self.scan = scan


def main(args=None):
    rclpy.init(args=args)

    get_object_range = GetObjectRange()
    rclpy.spin(get_object_range)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    get_object_range.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
