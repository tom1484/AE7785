"""AE7785 Lab 6 sign classifier (YOLOv8-cls trained with ultralytics, served via ONNX + cv2.dnn)."""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

CLASS_NAMES = {
    0: "empty",
    1: "left",
    2: "right",
    3: "do_not_enter",
    4: "stop",
    5: "goal",
}
