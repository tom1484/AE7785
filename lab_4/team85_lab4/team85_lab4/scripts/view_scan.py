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
    /scan_segments
      team85_lab4_interfaces/msg/ScanSegments
    /obstacle_position
      geometry_msgs/msg/PointStamped
    /obstacle_normal
      geometry_msgs/msg/Vector3Stamped

Debug tool: draws what scan_processor computes in a cv2 window. While
/scan_segments arrives, only segmented points are drawn, colored by segment;
otherwise the raw scan is drawn in gray. The obstacle point is circled in
white, with its surface normal as a white arrow.
Press 'q' / Esc, close the window, or Ctrl+C to quit.
"""

import os
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Vector3Stamped
from team85_lab4_interfaces.msg import ScanSegments

import cv2
import numpy as np

from team85_lab4.verbose_logger import VerboseLogger


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
NORMAL_LENGTH = 0.3  # Length of the drawn normal arrow, in meters
SEGMENTS_TIMEOUT = 0.5  # Draw the raw scan when no segments arrived for this long (s)

QUIT_KEYS = (ord("q"), 27)  # 'q' and Esc


class ViewScan(Node):

    def __init__(self):
        super().__init__("view_scan")

        self.image_size = int(self.declare_parameter("image_size", 800).value)
        self.max_range = float(self.declare_parameter("max_range", 3.5).value)
        self.log = VerboseLogger(self)

        # Display only: scan_processor's field of view, drawn as two lines
        self.field_of_view = float(self.declare_parameter("field_of_view", 0.8 * np.pi).value)

        # Pixels per meter, so that max_range reaches the image border
        self.scale = (self.image_size / 2) / self.max_range

        # Latest messages; drawn together on every update
        self.scan: Optional[LaserScan] = None
        self.segments: Optional[ScanSegments] = None
        self.segments_received: Optional[Time] = None
        self.obstacle_position: Optional[PointStamped] = None
        self.obstacle_normal: Optional[Vector3Stamped] = None

        self.laser_subscription = self.create_subscription(
            LaserScan, "/scan", self.laser_callback, qos_profile_sensor_data
        )
        self.segments_subscription = self.create_subscription(
            ScanSegments, "/scan_segments", self.segments_callback, 10
        )
        self.position_subscription = self.create_subscription(
            PointStamped, "/obstacle_position", self.position_callback, 10
        )
        self.normal_subscription = self.create_subscription(
            Vector3Stamped, "/obstacle_normal", self.normal_callback, 10
        )

        self.running = True

        # Show the empty plot until the first scan arrives
        self.image = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
        self.draw_background(self.image)
        cv2.namedWindow(WINDOW_NAME)

        self.log.basic("Listening to scan...")

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
        if self.field_of_view < 2 * np.pi:
            half = self.field_of_view / 2
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

    def draw_obstacle(self, image: np.ndarray, position: np.ndarray, normal: np.ndarray):
        end = position + normal * NORMAL_LENGTH
        cols, rows = self.to_pixel(np.array([position[0], end[0]]), np.array([position[1], end[1]]))

        cv2.circle(image, (int(cols[0]), int(rows[0])), 6, OBSTACLE_COLOR, 1)
        cv2.arrowedLine(
            image, (int(cols[0]), int(rows[0])), (int(cols[1]), int(rows[1])),
            OBSTACLE_COLOR, 2, tipLength=0.2,
        )
        cv2.putText(
            image, f"{np.linalg.norm(position):.2f} m", (int(cols[0]) + 8, int(rows[0]) + 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, OBSTACLE_COLOR, 1,
        )

    def laser_callback(self, scan: LaserScan):
        self.scan = scan
        self.render()

    def segments_callback(self, msg: ScanSegments):
        self.segments = msg
        self.segments_received = self.get_clock().now()
        self.render()

    def position_callback(self, msg: PointStamped):
        self.obstacle_position = msg
        self.render()

    def normal_callback(self, msg: Vector3Stamped):
        self.obstacle_normal = msg
        self.render()

    def segments_fresh(self) -> bool:
        if self.segments_received is None:
            return False
        age = (self.get_clock().now() - self.segments_received).nanoseconds * 1e-9
        return age <= SEGMENTS_TIMEOUT

    def render(self):
        image = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
        self.draw_background(image)

        segmented = self.segments_fresh()

        # Raw scan in gray, only while scan_processor publishes no segments
        if not segmented and self.scan is not None:
            scan = self.scan
            ranges = np.array(scan.ranges, dtype=float)
            angles = scan.angle_min + np.arange(ranges.shape[0]) * scan.angle_increment
            valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
            x = ranges[valid] * np.cos(angles[valid])
            y = ranges[valid] * np.sin(angles[valid])
            self.draw_points(image, x, y, UNSEGMENTED_COLOR)

        if segmented:
            for i, segment in enumerate(self.segments.segments):
                color = SEGMENT_COLORS[i % len(SEGMENT_COLORS)]
                x = np.array([p.x for p in segment.points])
                y = np.array([p.y for p in segment.points])
                self.draw_points(image, x, y, color)

            # Only draw an obstacle computed from the same scan as the segments,
            # since nothing is published when no obstacle is in view
            stamp = self.segments.header.stamp
            position, normal = self.obstacle_position, self.obstacle_normal
            if (position is not None and normal is not None
                    and position.header.stamp == stamp and normal.header.stamp == stamp):
                self.draw_obstacle(
                    image,
                    np.array([position.point.x, position.point.y]),
                    np.array([normal.vector.x, normal.vector.y]),
                )

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
