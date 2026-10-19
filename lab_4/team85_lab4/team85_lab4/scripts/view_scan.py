#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 4

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    view_scan

Subscribes:
    /scan
      sensor_msgs/msg/LaserScan

Debug tool: draws the LiDAR points in a cv2 window, colored by segment.
Points that belong to no segment are drawn in gray.
The closest obstacle point in the field of view is circled in white, with its
surface normal drawn as a white arrow (orange if the point is at a segment
end, i.e. the normal was fitted from one side only).
Press 'q' / Esc, close the window, or Ctrl+C to quit.
"""

import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan

import cv2
import numpy as np

from team85_lab4.scan_segmentation import ScanSegmenter
from team85_lab4.obstacle_detector import Obstacle, ObstacleDetector


# Frame of the bot (identical to the LiDAR frame):
#   x: forward
#   y: left
# Scan angle 0 is on +x and increases counterclockwise (towards +y).

# Image layout:
#   center: LiDAR origin
#   up:     +x
#   left:   +y

WINDOW_NAME = "scan"
RING_STEP = 0.5  # Distance between range rings, in meters

# BGR colors cycled through segments
SEGMENT_COLORS = [
    (75, 25, 230), (75, 180, 60), (25, 225, 255), (200, 130, 0),
    (48, 130, 245), (180, 30, 145), (240, 240, 70), (230, 50, 240),
    (60, 245, 210), (212, 190, 250), (128, 128, 0), (40, 110, 170),
]
UNSEGMENTED_COLOR = (90, 90, 90)
FOV_COLOR = (0, 120, 120)
OBSTACLE_COLOR = (255, 255, 255)
ONE_SIDED_COLOR = (0, 165, 255)  # Obstacle point at a segment end
NORMAL_LENGTH = 0.3  # Length of the drawn normal arrow, in meters

QUIT_KEYS = (ord("q"), 27)  # 'q' and Esc


class ViewScan(Node):

    def __init__(self):
        super().__init__("view_scan")

        self.image_size = int(self.declare_parameter("image_size", 800).value)
        self.max_range = float(self.declare_parameter("max_range", 3.5).value)

        self.segmenter = ScanSegmenter(
            min_points=int(self.declare_parameter("min_points", 3).value),
            base_threshold=float(self.declare_parameter("base_threshold", 0.05).value),
            k=float(self.declare_parameter("k", 2.0).value),
        )

        self.detector = ObstacleDetector(
            field_of_view=float(self.declare_parameter("field_of_view", 0.8 * np.pi).value),
            normal_radius=float(self.declare_parameter("normal_radius", 0.1).value),
            min_normal_points=int(self.declare_parameter("min_normal_points", 5).value),
        )

        # Pixels per meter, so that max_range reaches the image border
        self.scale = (self.image_size / 2) / self.max_range

        self.laser_subscription = self.create_subscription(
            LaserScan, "/scan", self.laser_callback, qos_profile_sensor_data
        )

        self.running = True

        # Show the empty plot until the first scan arrives
        self.image = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
        self.draw_background(self.image)
        cv2.namedWindow(WINDOW_NAME)

        self.get_logger().info("Listening to scan...")

    def to_pixel(self, x: np.ndarray, y: np.ndarray):
        # Robot frame -> image frame: +x goes up (-row), +y goes left (-col)
        c = self.image_size / 2
        cols = np.round(c - y * self.scale).astype(int)
        rows = np.round(c - x * self.scale).astype(int)
        return cols, rows

    def draw_background(self, image: np.ndarray):
        c = self.image_size // 2

        # Range rings
        for r in np.arange(RING_STEP, self.max_range + 1e-6, RING_STEP):
            cv2.circle(image, (c, c), int(round(r * self.scale)), (60, 60, 60), 1)
            cv2.putText(
                image, f"{r:.1f}", (c + 3, c - int(round(r * self.scale)) - 3),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1,
            )

        # Field of view boundaries
        if self.detector.field_of_view < 2 * np.pi:
            half = self.detector.field_of_view / 2
            for a in (-half, half):
                cols, rows = self.to_pixel(
                    np.array([self.max_range * np.cos(a)]) * np.sqrt(2),
                    np.array([self.max_range * np.sin(a)]) * np.sqrt(2),
                )
                cv2.line(image, (c, c), (int(cols[0]), int(rows[0])), FOV_COLOR, 1)

        # Axes: +x (red) up, +y (green) left
        axis_len = int(round(min(0.5, self.max_range) * self.scale))
        cv2.arrowedLine(image, (c, c), (c, c - axis_len), (0, 0, 255), 2, tipLength=0.1)
        cv2.arrowedLine(image, (c, c), (c - axis_len, c), (0, 255, 0), 2, tipLength=0.1)
        cv2.putText(image, "x", (c + 5, c - axis_len), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 1)
        cv2.putText(image, "y", (c - axis_len, c - 8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 1)

    def draw_points(self, image: np.ndarray, x: np.ndarray, y: np.ndarray, color):
        cols, rows = self.to_pixel(x, y)
        inside = (cols >= 0) & (cols < self.image_size) & (rows >= 0) & (rows < self.image_size)
        for col, row in zip(cols[inside], rows[inside]):
            cv2.circle(image, (int(col), int(row)), 2, color, -1)

    def draw_obstacle(self, image: np.ndarray, obstacle: Obstacle):
        start = obstacle.point
        end = start + obstacle.normal * NORMAL_LENGTH
        cols, rows = self.to_pixel(np.array([start[0], end[0]]), np.array([start[1], end[1]]))

        color = ONE_SIDED_COLOR if obstacle.at_end else OBSTACLE_COLOR

        cv2.circle(image, (int(cols[0]), int(rows[0])), 6, color, 1)
        cv2.arrowedLine(
            image, (int(cols[0]), int(rows[0])), (int(cols[1]), int(rows[1])),
            color, 2, tipLength=0.2,
        )
        cv2.putText(
            image, f"{obstacle.distance:.2f} m", (int(cols[0]) + 8, int(rows[0]) + 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1,
        )

    def laser_callback(self, scan: LaserScan):
        ranges = np.array(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(ranges.shape[0]) * scan.angle_increment

        # Drop invalid returns
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        ranges = ranges[valid]
        angles = angles[valid]

        # Polar -> Cartesian in the robot frame
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        image = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
        self.draw_background(image)

        self.draw_points(image, x, y, UNSEGMENTED_COLOR)
        segments = self.segmenter.segment(scan)
        for i, segment in enumerate(segments):
            color = SEGMENT_COLORS[i % len(SEGMENT_COLORS)]
            self.draw_points(image, segment.points[:, 0], segment.points[:, 1], color)

        obstacle = self.detector.detect(segments)
        if obstacle is not None:
            self.draw_obstacle(image, obstacle)

        self.image = image

    def update_window(self):
        # Runs in the main loop (not the scan callback), so the window keeps
        # responding to keys and the close button even when no scans arrive.
        cv2.imshow(WINDOW_NAME, self.image)
        key = cv2.waitKey(1) & 0xFF
        closed = cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1
        if key in QUIT_KEYS or closed:
            self.running = False


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ViewScan()
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.03)
            node.update_window()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        cv2.waitKey(1)  # Let the GUI process the close (needed on macOS)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

    # Skip the interpreter's normal teardown: OpenCV's Qt backend otherwise
    # prints "QThreadStorage: entry 0 destroyed before end of thread" while
    # Qt's globals are destroyed at exit. All cleanup is already done above.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
