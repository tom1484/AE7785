#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 4

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    scan_processor

Subscribes:
    /scan
      sensor_msgs/msg/LaserScan

Publishes:
    /scan_segments
      team85_lab4_interfaces/msg/ScanSegments
      Segmented scan; points not in any segment are treated as noise.
    /obstacle_position
      geometry_msgs/msg/PointStamped
      Closest segment point within the field of view.
    /obstacle_normal
      geometry_msgs/msg/Vector3Stamped
      Unit surface normal at that point, pointing towards the robot.

All outputs are in the robot frame (x: forward, y: left) and carry the stamp
of the source scan, so the obstacle position and normal can be paired.
Nothing is published on the obstacle topics when no obstacle is in view.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point32, PointStamped, Vector3Stamped
from team85_lab4_interfaces.msg import ScanSegments, Segment as SegmentMsg

import numpy as np

from team85_lab4.scan_segmentation import ScanSegmenter
from team85_lab4.obstacle_detector import ObstacleDetector
from team85_lab4.verbose_logger import VerboseLogger


class ScanProcessor(Node):

    def __init__(self):
        super().__init__("scan_processor")

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
        self.log = VerboseLogger(self)

        self.laser_subscription = self.create_subscription(
            LaserScan, "/scan", self.laser_callback, qos_profile_sensor_data
        )
        self.segments_publisher = self.create_publisher(ScanSegments, "/scan_segments", 10)
        self.position_publisher = self.create_publisher(PointStamped, "/obstacle_position", 10)
        self.normal_publisher = self.create_publisher(Vector3Stamped, "/obstacle_normal", 10)

        self.log.basic("Listening to scan...")

    def laser_callback(self, scan: LaserScan):
        segments = self.segmenter.segment(scan)

        segments_msg = ScanSegments()
        segments_msg.header = scan.header
        for segment in segments:
            segment_msg = SegmentMsg()
            segment_msg.indices = segment.indices.astype(int).tolist()
            segment_msg.points = [Point32(x=float(x), y=float(y)) for x, y in segment.points]
            segments_msg.segments.append(segment_msg)
        self.segments_publisher.publish(segments_msg)

        obstacle = self.detector.detect(segments)
        if obstacle is None:
            return

        position = PointStamped()
        position.header = scan.header
        position.point.x, position.point.y = (float(v) for v in obstacle.point)
        self.position_publisher.publish(position)

        normal = Vector3Stamped()
        normal.header = scan.header
        normal.vector.x, normal.vector.y = (float(v) for v in obstacle.normal)
        self.normal_publisher.publish(normal)

        x, y = obstacle.point
        self.log.continuous(
            f"obstacle x={x:6.3f} y={y:6.3f} "
            f"angle={np.degrees(np.arctan2(y, x)):6.1f} dist={obstacle.distance:6.3f}",
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ScanProcessor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
