#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 4

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    get_object_range

Subscribes:
    /scan
      sensor_msgs/msg/LaserScan

Publishes:
    /object_position
      geometry_msgs/msg/Point
      Closest obstacle point in the robot frame (LiDAR origin):
        x: forward
        y: left
        z: up

Segments the scan, finds the closest segment point within the field of view
(see ObstacleDetector), and publishes its position. The estimated surface
normal is not published for now.

Set the parameter log_result to true to print the latest positions; it can
also be toggled while running:
    ros2 param set /get_object_range log_result true
"""

import sys
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

import numpy as np

from team85_lab4.scan_segmentation import ScanSegmenter
from team85_lab4.obstacle_detector import ObstacleDetector


# Frame of the bot (identical to the LiDAR frame):
#   x: forward
#   y: left
#   z: up


class PositionHistory:
    def __init__(self) -> None:
        self._entries = deque(maxlen=5)
        self._lines_shown = 0

    def update(self, time: datetime, x: float, y: float) -> None:
        angle = np.degrees(np.arctan2(y, x))
        distance = np.hypot(x, y)
        self._entries.append(
            f"[{time:%Y-%m-%d %H:%M:%S.%f}] "
            f"x={x:6.3f} y={y:6.3f} angle={angle:6.1f} dist={distance:6.3f}"
        )

        if self._lines_shown:
            sys.stdout.write(f"\x1b[{self._lines_shown}F")
        sys.stdout.write("\x1b[J")  # Clear the previous display.
        sys.stdout.write("\n".join(self._entries) + "\n")
        sys.stdout.flush()
        self._lines_shown = len(self._entries)


class GetObjectRange(Node):

    def __init__(self):
        super().__init__("get_object_range")

        self.segmenter = ScanSegmenter(
            min_points=int(self.declare_parameter("min_points", 3).value),
            base_threshold=float(self.declare_parameter("base_threshold", 0.05).value),
            k=float(self.declare_parameter("k", 2.0).value),
        )
        self.detector = ObstacleDetector(
            field_of_view=float(self.declare_parameter("field_of_view", 2.0).value),
            normal_radius=float(self.declare_parameter("normal_radius", 0.1).value),
            min_normal_points=int(self.declare_parameter("min_normal_points", 5).value),
        )

        self.laser_subscription = self.create_subscription(
            LaserScan, "/scan", self.laser_callback, qos_profile_sensor_data
        )
        self.object_publisher = self.create_publisher(Point, "/object_position", 10)

        # Read on every scan, so it can be toggled with `ros2 param set`
        self.declare_parameter("log_result", False)
        self.history = PositionHistory()

        self.get_logger().info("Listening to scan...")

    def laser_callback(self, scan: LaserScan):
        obstacle = self.detector.detect(self.segmenter.segment(scan))
        if obstacle is None:
            return

        # The normal (obstacle.normal) is not used for now.
        x, y = obstacle.point

        object_position = Point()
        object_position.x = float(x)
        object_position.y = float(y)
        self.object_publisher.publish(object_position)

        if self.get_parameter("log_result").value:
            self.history.update(datetime.now(), float(x), float(y))


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = GetObjectRange()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print()
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
