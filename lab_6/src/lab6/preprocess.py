"""Image preprocessing shared by split generation (training) and predict() (grading).

Deliberately depends only on cv2 + numpy so it can be copied verbatim into the submission.
Training images are written to disk as the output of `prepare_image` (lossless PNG), and
ultralytics then only converts BGR->RGB and divides by 255, which `to_blob` reproduces.
"""

import cv2
import numpy as np


def prepare_image(image_bgr, imgsz=224, resize_mode="squash"):
    """BGR uint8 frame of any size -> BGR uint8 (imgsz, imgsz, 3)."""
    if resize_mode == "center_crop":
        h, w = image_bgr.shape[:2]
        s = min(h, w)
        y0, x0 = (h - s) // 2, (w - s) // 2
        image_bgr = image_bgr[y0 : y0 + s, x0 : x0 + s]
    elif resize_mode != "squash":
        raise ValueError(f"unknown resize_mode {resize_mode!r}")
    return cv2.resize(image_bgr, (imgsz, imgsz), interpolation=cv2.INTER_AREA)


def to_blob(prepared_bgr):
    """Prepared BGR uint8 (H, W, 3) -> float32 NCHW RGB in [0, 1], as ultralytics feeds the net."""
    rgb = cv2.cvtColor(prepared_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    return np.ascontiguousarray(rgb.transpose(2, 0, 1)[None])
