"""YOLO (detection) sign detector running an exported ONNX model with cv2.dnn.

No torch/ultralytics at runtime: the model is trained and exported in
~/Workspace/Courses/AE7785/lab_final_vision and copied to models/ here.
Expected ONNX output: ultralytics YOLOv8/11 detect export, shape (1, 4 + nc, N),
rows = cx, cy, w, h (input pixels) then per-class scores (no objectness).
Preprocessing must match training: BGR->RGB, letterbox to imgsz (pad 114), /255.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .signs import SignClass


@dataclass(frozen=True)
class Detection:
    sign: SignClass
    label: str
    confidence: float
    cx: float      # source-image pixels
    cy: float
    width: float
    height: float

    def bearing(self, image_width: int, hfov: float) -> float:
        """Angle of the box center from the optical axis (rad, + = left), pinhole model."""
        f = (image_width / 2) / math.tan(hfov / 2)
        return math.atan2((image_width / 2) - self.cx, f)

    def bearing_calibrated(self, K, D) -> float:
        """Same, from a camera calibration (CameraInfo k: 3x3 row-major, d: plumb_bob).
        Uses the real principal point and removes lens distortion (the robot's Pi camera:
        k1 = 0.26, cx 6 px off center)."""
        return calibrated_bearing(self.cx, self.cy, K, D)


def calibrated_bearing(u: float, v: float, K, D) -> float:
    """Bearing (rad, + = left) of pixel (u, v) with intrinsics K (9 values) and distortion D."""
    import cv2
    import numpy as np
    k = np.asarray(K, dtype=np.float64).reshape(3, 3)
    d = np.asarray(D, dtype=np.float64)
    xn = cv2.undistortPoints(np.array([[[u, v]]], dtype=np.float64), k, d)[0, 0, 0]
    return math.atan(-xn)


class SignDetector:
    def __init__(self, model_path: str, class_names: list[str], imgsz: int = 320,
                 conf_threshold: float = 0.5, nms_threshold: float = 0.45):
        import cv2

        self._cv2 = cv2
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.class_names = list(class_names)
        self.signs = [SignClass.from_name(n) for n in self.class_names]
        self.imgsz = imgsz
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold

    def __call__(self, bgr: np.ndarray) -> list[Detection]:
        blob, scale, pad = self.preprocess(bgr)
        self.net.setInput(blob)
        out = self.net.forward()
        return self.postprocess(out, scale, pad)

    def preprocess(self, bgr: np.ndarray) -> tuple[np.ndarray, float, tuple[float, float]]:
        cv2 = self._cv2
        h, w = bgr.shape[:2]
        scale = min(self.imgsz / h, self.imgsz / w)
        nh, nw = round(h * scale), round(w * scale)
        canvas = np.full((self.imgsz, self.imgsz, 3), 114, np.uint8)
        top, left = (self.imgsz - nh) // 2, (self.imgsz - nw) // 2
        canvas[top:top + nh, left:left + nw] = cv2.resize(bgr, (nw, nh),
                                                          interpolation=cv2.INTER_LINEAR)
        blob = cv2.dnn.blobFromImage(canvas, 1 / 255.0, swapRB=True)
        return blob, scale, (left, top)

    def postprocess(self, out: np.ndarray, scale: float,
                    pad: tuple[float, float]) -> list[Detection]:
        pred = np.squeeze(out, 0).T                     # (N, 4 + nc)
        scores = pred[:, 4:]
        cls = scores.argmax(1)
        conf = scores[np.arange(len(cls)), cls]
        keep = conf >= self.conf_threshold
        pred, cls, conf = pred[keep], cls[keep], conf[keep]
        if not len(pred):
            return []
        cx = (pred[:, 0] - pad[0]) / scale
        cy = (pred[:, 1] - pad[1]) / scale
        bw, bh = pred[:, 2] / scale, pred[:, 3] / scale
        boxes = np.stack([cx - bw / 2, cy - bh / 2, bw, bh], 1)
        idx = self._cv2.dnn.NMSBoxes(boxes.tolist(), conf.tolist(),
                                     self.conf_threshold, self.nms_threshold)
        return [
            Detection(self.signs[cls[i]], self.class_names[cls[i]], float(conf[i]),
                      float(cx[i]), float(cy[i]), float(bw[i]), float(bh[i]))
            for i in np.array(idx).flatten()
        ]
