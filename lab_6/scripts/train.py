#!/usr/bin/env python3
"""Train + export + evaluate one experiment.

Usage:
  python scripts/train.py configs/experiments/yolov8m_cls.yaml
  python scripts/train.py configs/experiments/yolov8m_cls.yaml --set train.epochs=1 name=smoke
"""

import argparse
import json

import yaml

from lab6.config import load_config
from lab6.train import train


def apply_overrides(cfg: dict, overrides: list[str]) -> dict:
    for item in overrides:
        key, value = item.split("=", 1)
        node = cfg
        *parents, leaf = key.split(".")
        for p in parents:
            node = node.setdefault(p, {})
        node[leaf] = yaml.safe_load(value)
    return cfg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config")
    ap.add_argument("--set", nargs="*", default=[], help="dotted overrides, e.g. train.epochs=5")
    ap.add_argument("--force-split", action="store_true", help="rebuild data/splits/<id>")
    args = ap.parse_args()

    cfg = apply_overrides(load_config(args.config), args.set)
    report = train(cfg, force_split=args.force_split)
    summary = {
        "run_id": report["run_id"],
        "train_minutes": round(report["train_minutes"], 1),
        "onnx_mb": round(report["onnx_mb"], 1),
        "parity": report["parity"],
        **{
            f"{b}_acc": round(e["accuracy"], 4)
            for b, e in report["eval"].items()
        },
        "cv2_latency_ms": round(report["eval"]["cv2"]["latency_ms_median"], 1),
        "acc_by_source": report["eval"]["cv2"]["accuracy_by_source"],
        "confusion": report["eval"]["cv2"]["confusion"],
        "errors": report["eval"]["cv2"]["errors"],
    }
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
