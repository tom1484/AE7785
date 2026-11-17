"""Accuracy / confusion / latency of an ONNX model on a split's val set, on CPU."""

from __future__ import annotations

import time

import cv2
import numpy as np

from lab6 import CLASS_NAMES
from lab6.data import Sample
from lab6.infer import BACKENDS


def evaluate(onnx_path, samples: list[Sample], imgsz: int, resize_mode: str, backend="cv2") -> dict:
    clf = BACKENDS[backend](onnx_path, imgsz, resize_mode)
    n = len(CLASS_NAMES)
    cm = np.zeros((n, n), dtype=int)
    times, errors = [], []
    clf.predict(cv2.imread(str(samples[0].path)))  # warm-up
    for s in samples:
        img = cv2.imread(str(s.path))
        t0 = time.perf_counter()
        pred = clf.predict(img)
        times.append(time.perf_counter() - t0)
        cm[s.label, pred] += 1
        if pred != s.label:
            errors.append({"image": f"{s.source}/{s.path.name}", "true": s.label, "pred": pred})
    acc = float(np.trace(cm) / cm.sum())
    by_source = {}
    for src in sorted({s.source for s in samples}):
        idx = [i for i, s in enumerate(samples) if s.source == src]
        wrong = {e["image"] for e in errors}
        by_source[src] = 1 - sum(f"{src}/{samples[i].path.name}" in wrong for i in idx) / len(idx)
    return {
        "backend": backend,
        "n": len(samples),
        "accuracy": acc,
        "accuracy_by_source": by_source,
        "confusion": cm.tolist(),
        "latency_ms_median": 1000 * float(np.median(times)),
        "errors": errors,
    }
