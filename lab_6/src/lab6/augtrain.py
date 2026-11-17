"""Train-time corruption augmentation plugged into ultralytics' classification trainer.

Split images are named `<source>__<stem>.png`; for training samples we instead load the RAW frame
data/raw/<source>/<stem>.png, apply lab6.corrupt.composite (same code as the robustness test) with
probability p, then lab6.preprocess.prepare_image -- exactly the predict() path. Ultralytics' own
torch transforms (RandomResizedCrop, ColorJitter, RandomErasing) still run afterwards.
Validation samples are untouched (clean).
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
from ultralytics.data.dataset import ClassificationDataset
from ultralytics.models.yolo.classify import ClassificationTrainer

from lab6 import REPO_ROOT
from lab6.corrupt import composite
from lab6.preprocess import prepare_image


def raw_path(split_file: str | Path) -> Path:
    source, stem = Path(split_file).stem.split("__", 1)
    return REPO_ROOT / "data" / "raw" / source / f"{stem}.png"


class CorruptingDataset(ClassificationDataset):
    def __init__(self, *args, corrupt_cfg: dict, imgsz: int, resize_mode: str, **kwargs):
        self.p = float(corrupt_cfg.get("p", 0.8))
        self.k_range = tuple(corrupt_cfg.get("k_range", (1, 4)))
        self.imgsz, self.resize_mode = imgsz, resize_mode
        super().__init__(*args, **kwargs)

    def imread(self, f: str) -> np.ndarray:  # overrides ClassificationDataset.imread (staticmethod)
        img = cv2.imread(str(raw_path(f)))
        if img is None:
            raise FileNotFoundError(raw_path(f))
        rng = np.random.default_rng()  # fresh OS entropy: distinct per call and per worker
        if rng.random() < self.p:
            img, _ = composite(img, rng, self.k_range)
        return prepare_image(img, self.imgsz, self.resize_mode)


def make_trainer(corrupt_cfg: dict, imgsz: int, resize_mode: str):
    class CorruptingTrainer(ClassificationTrainer):
        def build_dataset(self, img_path, mode="train", batch=None):
            if mode != "train":
                return super().build_dataset(img_path, mode, batch)
            return CorruptingDataset(
                img_path,
                self.args,
                augment=True,
                prefix="train",
                names=self.data["names"],
                corrupt_cfg=corrupt_cfg,
                imgsz=imgsz,
                resize_mode=resize_mode,
            )

    return CorruptingTrainer
