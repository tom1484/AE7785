"""
Node sign_detector: runs the YOLO sign detector on every camera frame it can keep
up with and publishes all signs in view.

Subscribes: image_topic (sensor_msgs/CompressedImage, or Image if compressed: false)
Publishes:  /sign_detections (team85_final_interfaces/SignDetectionArray)
            /sign_detections/image (Image bgr8, boxes drawn; if publish_debug_image; RViz)
            /sign_detections/image/compressed (same, jpeg; rqt_image_view over wifi)
"""

import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from team85_final_interfaces.msg import SignDetection, SignDetectionArray

from ..vision.detector import SignDetector


def share_path(path: str) -> str:
    return path if os.path.isabs(path) else os.path.join(
        get_package_share_directory('team85_final'), path)


class SignDetectorNode(Node):
    def __init__(self):
        super().__init__('sign_detector')
        p = self.declare_parameters('', [
            ('image_topic', '/image_raw/compressed'),
            ('compressed', True),
            ('model', 'models/signs.onnx'),
            ('class_names', ['left', 'right', 'do_not_enter', 'stop', 'goal']),
            ('imgsz', 416),
            ('conf_threshold', 0.5),
            ('nms_threshold', 0.45),
            ('hfov', 1.085),              # fallback until camera_info arrives
            ('camera_info_topic', '/camera_info'),
            ('max_rate', 10.0),
            ('publish_debug_image', False),
        ])
        v = {x.name: x.value for x in p}
        self.hfov = v['hfov']
        self.min_period = 1.0 / v['max_rate'] if v['max_rate'] > 0 else 0.0
        self.debug = v['publish_debug_image']
        self.compressed = v['compressed']
        self.detector = SignDetector(share_path(v['model']), list(v['class_names']), v['imgsz'],
                                     v['conf_threshold'], v['nms_threshold'])
        self.last_t = 0.0

        self.pub = self.create_publisher(SignDetectionArray, '/sign_detections', 10)
        if self.debug:
            self.debug_pub = self.create_publisher(Image, '/sign_detections/image', 1)
            self.debug_jpg = self.create_publisher(CompressedImage,
                                                   '/sign_detections/image/compressed', 1)
        else:
            self.debug_pub = self.debug_jpg = None
        msg_type = CompressedImage if self.compressed else Image
        self.create_subscription(msg_type, v['image_topic'], self.on_image, qos_profile_sensor_data)
        # Bearings from the camera's calibration when it publishes one (real distortion and
        # principal point); else the pinhole model with `hfov`.
        self.calib = None
        self.create_subscription(CameraInfo, v['camera_info_topic'], self.on_info,
                                 qos_profile_sensor_data)
        self.get_logger().info(f"sign_detector on {v['image_topic']} with {v['model']}")

    def on_info(self, msg: CameraInfo):
        if any(msg.k) and (self.calib is None):
            self.get_logger().info(f'using camera calibration: K {list(msg.k)} D {list(msg.d)}')
        if any(msg.k):
            self.calib = (list(msg.k), list(msg.d), msg.width, msg.height)

    def on_image(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_t < self.min_period:
            return
        self.last_t = now
        img = self.decode(msg)
        if img is None:
            return
        h, w = img.shape[:2]
        dets = self.detector(img)

        out = SignDetectionArray(header=msg.header, image_width=w, image_height=h)
        for d in dets:
            out.detections.append(SignDetection(
                sign=int(d.sign), label=d.label, confidence=d.confidence,
                cx=d.cx, cy=d.cy, width=d.width, height=d.height,
                bearing=(d.bearing_calibrated(*self.calib[:2]) if self.calib and self.calib[2] == w
                         else d.bearing(w, self.hfov)),
            ))
        self.pub.publish(out)

        if self.debug_pub is not None:
            for d in dets:
                x0, y0 = int(d.cx - d.width / 2), int(d.cy - d.height / 2)
                cv2.rectangle(img, (x0, y0), (int(x0 + d.width), int(y0 + d.height)), (0, 255, 0), 2)
                cv2.putText(img, f'{d.label} {d.confidence:.2f}', (x0, max(0, y0 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            self.debug_pub.publish(Image(header=msg.header, height=h, width=w, encoding='bgr8',
                                         step=3 * w, data=img.tobytes()))
            ok, buf = cv2.imencode('.jpg', img)
            if ok:
                self.debug_jpg.publish(CompressedImage(header=msg.header, format='jpeg',
                                                       data=buf.tobytes()))

    def decode(self, msg):
        if self.compressed:
            return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        # TODO: handle encodings other than rgb8/bgr8 if the sim camera uses them.
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img.copy()


def main():
    rclpy.init()
    node = SignDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
