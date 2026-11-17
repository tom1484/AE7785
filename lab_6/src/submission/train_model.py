#!/usr/bin/env python3
"""Train the Lab 6 sign classifier and export it to ONNX for model_grader.py.

Model: YOLOv8m-cls (ImageNet-pretrained, ultralytics), fine-tuned on the course sign images with
robustness augmentation: every training sample is re-read from its original frame and, with
probability 0.8, 1-4 random corruptions (lighting, color, sensor noise, JPEG, blur, geometry,
glare, occlusion; see supplemental/corrupt.py) are applied BEFORE resizing, followed by
ultralytics' own crop / color-jitter / random-erasing. Horizontal flips are disabled because a
mirrored left arrow is a right arrow.

Preprocessing (supplemental/preprocess.py, mirrored inline in model_grader.predict):
squash-resize the whole frame to 224x224 with INTER_AREA, BGR->RGB, /255.

Usage (all paths relative):
    python3 train_model.py --data_dirs ./2025F_imgs ./2025F_Gimgs ./2026F_imgs --out ./model.onnx
"""

import argparse
import csv
import json
import shutil
from pathlib import Path

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from ultralytics.data.dataset import ClassificationDataset
from ultralytics.models.yolo.classify import ClassificationTrainer

from supplemental.corrupt import composite
from supplemental.preprocess import prepare_image

CLASS_NAMES = {0: "empty", 1: "left", 2: "right", 3: "do_not_enter", 4: "stop", 5: "goal"}


def read_labels(folder):
    folder = Path(folder)
    with open(folder / "labels.txt") as f:
        return [(folder / f"{r[0].strip()}.png", int(r[1])) for r in csv.reader(f) if r]


def build_dataset(data_dirs, root, imgsz, val_ratio, seed):
    """Write root/{train,val}/<k_name>/<i>.png (preprocessed) + root/raw_index.json.

    Training uses ALL images. The val folder is a stratified subset of the same images used only
    to monitor training (the exported checkpoint is the last epoch, not selected on val).
    raw_index maps each written file name to its original frame for train-time corruption.
    """
    if root.exists():
        shutil.rmtree(root)
    samples = [s for d in data_dirs for s in read_labels(d)]
    rng = np.random.default_rng(seed)
    val_idx = set()
    for label in CLASS_NAMES:
        idx = [i for i, (_, l) in enumerate(samples) if l == label]
        rng.shuffle(idx)
        val_idx.update(idx[: max(1, round(len(idx) * val_ratio))])

    raw_index = {}
    for part in ("train", "val"):
        for label, name in CLASS_NAMES.items():
            (root / part / f"{label}_{name}").mkdir(parents=True, exist_ok=True)
    for i, (path, label) in enumerate(samples):
        img = cv2.imread(str(path))
        if img is None:
            raise FileNotFoundError(path)
        fname = f"{i:05d}.png"
        raw_index[fname] = str(path.resolve())
        prepared = prepare_image(img, imgsz, "squash")
        cv2.imwrite(str(root / "train" / f"{label}_{CLASS_NAMES[label]}" / fname), prepared)
        if i in val_idx:
            cv2.imwrite(str(root / "val" / f"{label}_{CLASS_NAMES[label]}" / fname), prepared)
    (root / "raw_index.json").write_text(json.dumps(raw_index))
    print(f"{len(samples)} images -> {root} (monitor val subset: {len(val_idx)})")


class CorruptingDataset(ClassificationDataset):
    """Training images are re-read from the raw frame and corrupted before resizing."""

    def __init__(self, *args, raw_index, imgsz, p, k_range, **kwargs):
        self.raw_index, self.imgsz, self.p, self.k_range = raw_index, imgsz, p, tuple(k_range)
        super().__init__(*args, **kwargs)

    def imread(self, f):
        img = cv2.imread(self.raw_index[Path(f).name])
        rng = np.random.default_rng()
        if rng.random() < self.p:
            img, _ = composite(img, rng, self.k_range)
        return prepare_image(img, self.imgsz, "squash")


class CorruptingTrainer(ClassificationTrainer):
    aug = {}  # set in main(): raw_index, imgsz, p, k_range

    def build_dataset(self, img_path, mode="train", batch=None):
        if mode != "train":
            return super().build_dataset(img_path, mode, batch)
        return CorruptingDataset(
            img_path, self.args, augment=True, prefix="train", names=self.data["names"], **self.aug
        )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data_dirs", nargs="+", required=True, help="folders with labels.txt + NNN.png")
    ap.add_argument("--out", default="./model.onnx")
    ap.add_argument("--work_dir", default="./work")
    ap.add_argument("--model", default="yolov8m-cls.pt", help="pretrained weights (auto-downloaded)")
    ap.add_argument("--imgsz", type=int, default=224)
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--corrupt_p", type=float, default=0.8)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--device", default=None, help="cuda / mps / cpu (default: best available)")
    args = ap.parse_args()

    device = args.device or (
        "cuda" if torch.cuda.is_available() else "mps" if torch.backends.mps.is_available() else "cpu"
    )
    work = Path(args.work_dir)
    data_root = work / "data"
    build_dataset(args.data_dirs, data_root, args.imgsz, val_ratio=0.2, seed=args.seed)

    CorruptingTrainer.aug = {
        "raw_index": json.loads((data_root / "raw_index.json").read_text()),
        "imgsz": args.imgsz,
        "p": args.corrupt_p,
        "k_range": (1, 4),
    }
    model = YOLO(args.model)
    model.train(
        trainer=CorruptingTrainer,
        data=str(data_root),
        imgsz=args.imgsz,
        epochs=args.epochs,
        patience=0,  # no early stopping; export the last epoch
        batch=args.batch,
        device=device,
        workers=4,
        seed=args.seed,
        fliplr=0.0,  # mirroring would swap left/right arrows
        flipud=0.0,
        scale=0.3,  # RandomResizedCrop keeps 70-100% of the area
        hsv_h=0.015,
        hsv_s=0.5,
        hsv_v=0.4,
        erasing=0.2,
        auto_augment=None,  # RandAugment's solarize/color ops distort the sign colors
        project=str((work / "runs").resolve()),
        name="train",
        exist_ok=True,
        verbose=False,
    )

    last = work / "runs" / "train" / "weights" / "last.pt"
    exported = YOLO(str(last)).export(format="onnx", imgsz=args.imgsz, opset=12, simplify=True, device="cpu")
    shutil.copy(exported, args.out)
    print(f"exported {args.out}")


if __name__ == "__main__":
    main()
