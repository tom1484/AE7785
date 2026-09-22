#!/usr/bin/env python3

from typing import cast

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point

import cv2
import numpy as np

from team85_object_follower.object_detector import ObjectDetector


class FindObject(Node):

    def __init__(self):
        super().__init__("find_object")

        self.image_subscription = self.create_subscription(
            CompressedImage, "/image_raw/compressed", self.image_callback, 10
        )
        self.pos_publisher = self.create_publisher(Point, "/detection", 10)

        self.object_detector = ObjectDetector()
        self.object_detector.create()

        self.running = True

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cast(np.ndarray, cv2.imdecode(np_arr, cv2.IMREAD_COLOR))

        try:
            ret, detection = self.object_detector.detect(frame)
            if ret == 1:
                self.running = False
            elif detection is not None:
                pos = Point()
                pos.x = detection[0] / frame.shape[1]
                pos.y = detection[1] / frame.shape[0]
                pos.z = 0.0
                self.pos_publisher.publish(pos)
        except Exception:
            self.get_logger().info("Unknown error happened in object detector.")

    def destroy_node(self):
        self.object_detector.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    find_object = FindObject()

    while rclpy.ok() and find_object.running:
        rclpy.spin_once(find_object, timeout_sec=0.1)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    find_object.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
