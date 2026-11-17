"""CPU inference backends for an exported ONNX classifier (what grading will use)."""

from __future__ import annotations

import numpy as np

from lab6.preprocess import prepare_image, to_blob


class Cv2DnnClassifier:
    def __init__(self, onnx_path, imgsz=224, resize_mode="squash"):
        import cv2

        self.net = cv2.dnn.readNetFromONNX(str(onnx_path))
        self.imgsz, self.resize_mode = imgsz, resize_mode

    def probs(self, image_bgr) -> np.ndarray:
        self.net.setInput(to_blob(prepare_image(image_bgr, self.imgsz, self.resize_mode)))
        return np.asarray(self.net.forward()).reshape(-1)

    def predict(self, image_bgr) -> int:
        return int(np.argmax(self.probs(image_bgr)))


class OrtClassifier(Cv2DnnClassifier):
    def __init__(self, onnx_path, imgsz=224, resize_mode="squash"):
        import onnxruntime as ort

        self.sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
        self.input_name = self.sess.get_inputs()[0].name
        self.imgsz, self.resize_mode = imgsz, resize_mode

    def probs(self, image_bgr) -> np.ndarray:
        blob = to_blob(prepare_image(image_bgr, self.imgsz, self.resize_mode))
        return np.asarray(self.sess.run(None, {self.input_name: blob})[0]).reshape(-1)


BACKENDS = {"cv2": Cv2DnnClassifier, "ort": OrtClassifier}
