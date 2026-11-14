"""Train a YOLOv8-cls model from a config, export ONNX, check parity, evaluate on CPU."""

from __future__ import annotations

import json
import shutil
import time

import cv2
import numpy as np

from lab6 import REPO_ROOT
from lab6.data import build_split, load_manifest, split_id
from lab6.evaluate import evaluate
from lab6.infer import BACKENDS
from lab6.preprocess import prepare_image

TRAIN_KEYS = (
    "epochs patience batch device workers seed optimizer fliplr flipud scale "
    "hsv_h hsv_s hsv_v erasing auto_augment"
).split()


def run_id(cfg: dict) -> str:
    return f"{cfg['name']}__{split_id(cfg)}"


def parity(pt_model, onnx_path, samples, imgsz, resize_mode, k=20) -> dict:
    """Max |prob| difference between PyTorch and each ONNX backend on k val images."""
    imgs = [prepare_image(cv2.imread(str(s.path)), imgsz, resize_mode) for s in samples[:k]]
    ref = np.stack([r.probs.data.cpu().numpy() for r in pt_model.predict(imgs, imgsz=imgsz, verbose=False, device="cpu")])
    out = {}
    for name, cls in BACKENDS.items():
        clf = cls(onnx_path, imgsz, "squash")  # images already prepared; squash at imgsz is identity
        got = np.stack([clf.probs(im) for im in imgs])
        out[name] = {
            "max_abs_diff": float(np.abs(got - ref).max()),
            "argmax_agree": float((got.argmax(1) == ref.argmax(1)).mean()),
        }
    return out


def train(cfg: dict, force_split: bool = False) -> dict:
    from ultralytics import YOLO

    rid = run_id(cfg)
    split_root = build_split(cfg, force=force_split)
    pp, tc = cfg["preprocess"], cfg["train"]

    weights_dir = REPO_ROOT / "outputs" / "weights"
    weights_dir.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(weights_dir / tc["model"]))

    t0 = time.time()
    model.train(
        data=str(split_root),
        imgsz=pp["imgsz"],
        project=str(REPO_ROOT / "outputs" / "runs"),
        name=rid,
        exist_ok=True,
        plots=True,
        verbose=False,
        **{k: tc[k] for k in TRAIN_KEYS if k in tc},
    )
    train_minutes = (time.time() - t0) / 60

    run_dir = REPO_ROOT / "outputs" / "runs" / rid
    best = YOLO(str(run_dir / "weights" / "best.pt"))
    ex = cfg["export"]
    exported = best.export(
        format=ex["format"], imgsz=pp["imgsz"], opset=ex["opset"], simplify=ex["simplify"], device="cpu"
    )
    onnx_path = REPO_ROOT / "outputs" / "models" / f"{rid}.onnx"
    shutil.copy(exported, onnx_path)

    val = load_manifest(split_root, "val")
    report = {
        "run_id": rid,
        "config": cfg,
        "train_minutes": train_minutes,
        "onnx": str(onnx_path.relative_to(REPO_ROOT)),
        "onnx_mb": onnx_path.stat().st_size / 1e6,
        "parity": parity(best, onnx_path, val, pp["imgsz"], pp["resize_mode"]),
        "eval": {b: evaluate(onnx_path, val, pp["imgsz"], pp["resize_mode"], b) for b in BACKENDS},
    }
    (REPO_ROOT / "outputs" / "reports" / f"{rid}.json").write_text(json.dumps(report, indent=1))
    return report
