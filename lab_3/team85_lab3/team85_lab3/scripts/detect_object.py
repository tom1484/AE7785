#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 3

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    detect_object

Subscribes:
    /camera_info
      sensor_msgs/msg/CameraInfo
    /image_raw/compressed
      sensor_msgs/msg/CompressedImage

Publishes:
    /object_angle
      std_msgs/msg/Float32
"""

from typing import cast, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage, CameraInfo

import cv2
import numpy as np

from team85_lab3.object_detector import ObjectDetector


import sys
from typing import Optional


class DetectionStats:
    def __init__(self) -> None:
        self.frames_received = 0
        self.frames_detected = 0
        self._line_width = 0

    def update(self, angle: Optional[float], detected: bool) -> None:
        self.frames_received += 1
        if detected:
            self.frames_detected += 1

        rate = 100 * self.frames_detected / self.frames_received
        shown_angle = f"{angle:.2f}" if detected and angle is not None else "None"

        line = (
            f"Detected: {self.frames_detected}/{self.frames_received} "
            f"({rate:.1f}%) | Angle: {shown_angle}"
        )
        self._line_width = max(self._line_width, len(line))
        sys.stdout.write("\r" + line.ljust(self._line_width))
        sys.stdout.flush()


class DetectObject(Node):

    def __init__(self):
        super().__init__("detect_object")

        self.camera_info_subscription = self.create_subscription(
            CameraInfo, "/camera_info", self.camera_info_callback, 10
        )
        self.image_subscription = self.create_subscription(
            CompressedImage, "/image_raw/compressed", self.image_callback, 10
        )
        self.angle_publisher = self.create_publisher(Float32, "/object_angle", 10)

        self.declare_parameter("headless", False)
        self.headless = self.get_parameter("headless").get_parameter_value().bool_value

        self.object_detector = ObjectDetector(headless=self.headless)
        self.object_detector.create()

        self.running = True
        self.camera_info_received = False

        self.stats = DetectionStats()

    def camera_info_callback(self, camera_info: CameraInfo):
        if self.camera_info_received:
            return

        self.camera_K = np.array(camera_info.k).reshape(3, 3)
        self.camera_D = np.array(camera_info.d)
        self.camera_shape = (camera_info.width, camera_info.height)

        self.camera_info_received = True
        self.get_logger().info("Camera Info Received")

    def convert_point_radians(self, x: float, y: float):
        u = x * self.camera_shape[0]
        v = y * self.camera_shape[1]
        point = np.array([[[u, v]]], dtype=np.float64)

        x, y = cv2.undistortPoints(point, self.camera_K, self.camera_D)[0, 0]
        rad_x = np.arctan(x)  # radians
        rad_y = np.arctan(y)  # radians

        return float(rad_x), float(rad_y)

    def image_callback(self, image_msg: CompressedImage):
        if not self.camera_info_received:
            return

        np_arr = np.frombuffer(image_msg.data, dtype=np.uint8)
        frame = cast(np.ndarray, cv2.imdecode(np_arr, cv2.IMREAD_COLOR))

        try:
            ret, detection = self.object_detector.detect(frame)
            if ret == 1:
                self.running = False
            elif detection is not None:
                rad_x, rad_y = self.convert_point_radians(
                    detection[0] / frame.shape[1],
                    detection[1] / frame.shape[0],
                )
                angle = Float32()
                angle.data = rad_x
                self.angle_publisher.publish(angle)

                self.stats.update(rad_x, True)
            else:
                self.stats.update(None, False)
        except Exception as e:
            self.get_logger().info(f"Error happened in object detector:\n{e}")

    def destroy_node(self):
        self.object_detector.destroy()
        super().destroy_node()
        self.get_logger().info("Exiting...")


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = DetectObject()
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
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
