"""Export a trained detector to ONNX and evaluate it through the ROS runtime code.

Uses team85_final's SignDetector (cv2.dnn, letterbox, NMS) on the run's val images, so
the numbers are for exactly what runs on the robot. Reports per-class precision/recall
at IoU 0.5, a class confusion table over IoU-matched boxes (left<->right called out),
the lab-6-style "front sign" check, and CPU latency.

    python scripts/eval_det.py outputs/runs/det/<run> [--conf 0.5]
Writes outputs/models/<run>.onnx and outputs/reports/det__<run>.md.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
ROS_PKG = Path.home() / 'Workspace' / 'turtlebot3_ws' / 'src' / 'team85_final'
sys.path.insert(0, str(ROS_PKG))
sys.path.insert(0, str(ROOT / 'src'))
from team85_final.vision.detector import SignDetector  # noqa: E402
from lfv import labels as L  # noqa: E402
from lfv.zeroshot import iou  # noqa: E402

NAMES = ['left', 'right', 'do_not_enter', 'stop', 'goal']


def export(run: Path, imgsz: int) -> Path:
    out = ROOT / 'outputs' / 'models' / f'{run.name}.onnx'
    if out.exists() and out.stat().st_mtime > (run / 'weights' / 'best.pt').stat().st_mtime:
        return out
    from ultralytics import YOLO
    cwd = os.getcwd()
    os.chdir(ROOT / 'outputs' / 'weights')
    try:
        path = YOLO(str(run / 'weights' / 'best.pt')).export(
            format='onnx', imgsz=imgsz, opset=12, simplify=True, dynamic=False, nms=False)
    finally:
        os.chdir(cwd)
    out.parent.mkdir(parents=True, exist_ok=True)
    os.replace(path, out)
    return out


def gt_boxes(label_file: Path, w: int, h: int) -> list[list[float]]:
    out = []
    for line in label_file.read_text().splitlines():
        c, cx, cy, bw, bh = (float(v) for v in line.split())
        out.append([(cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h,
                    int(c)])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--conf', type=float, default=0.5)
    a = ap.parse_args()

    run = Path(a.run).resolve()
    args = yaml.safe_load((run / 'args.yaml').read_text())
    imgsz = int(args['imgsz'])
    data = yaml.safe_load(Path(args['data']).read_text())
    val_dir = Path(data['path']) / data['val']
    onnx = export(run, imgsz)
    det = SignDetector(str(onnx), NAMES, imgsz=imgsz, conf_threshold=a.conf)

    images = sorted(val_dir.iterdir())
    tp = np.zeros(5, int)
    fp = np.zeros(5, int)
    fn = np.zeros(5, int)
    conf_mat = np.zeros((5, 6), int)                  # gt class x (pred class | missed)
    front_ok = front_n = 0
    lab6 = {f'{i.split("/")[0]}__{Path(i.split("/")[1]).stem}': c for i, c in L.image_ids()}
    times = []
    for p in images:
        img = cv2.imread(str(p))
        h, w = img.shape[:2]
        t0 = time.perf_counter()
        dets = det(img)
        times.append(time.perf_counter() - t0)
        preds = [[d.cx - d.width / 2, d.cy - d.height / 2, d.cx + d.width / 2,
                  d.cy + d.height / 2, NAMES.index(d.label), d.confidence] for d in dets]
        gts = gt_boxes(Path(str(p.parent).replace('/images/', '/labels/')) / f'{p.stem}.txt', w, h)
        # Greedy class-agnostic matching by IoU, highest confidence first.
        used = set()
        for pr in sorted(preds, key=lambda d: -d[5]):
            best, bi = 0.5, None
            for gi, g in enumerate(gts):
                if gi not in used and iou(pr, g) >= best:
                    best, bi = iou(pr, g), gi
            if bi is None:
                fp[pr[4]] += 1
                continue
            used.add(bi)
            g = gts[bi]
            conf_mat[g[4], pr[4]] += 1
            if g[4] == pr[4]:
                tp[g[4]] += 1
            else:
                fp[pr[4]] += 1
                fn[g[4]] += 1
        for gi, g in enumerate(gts):
            if gi not in used:
                fn[g[4]] += 1
                conf_mat[g[4], 5] += 1
        # lab 6 check: frames labeled with a sign -> the largest detection has that class
        c6 = lab6.get(p.stem, 0)
        if c6:
            front_n += 1
            if preds:
                big = max(preds, key=lambda d: (d[2] - d[0]) * (d[3] - d[1]))
                front_ok += int(big[4] == c6 - 1)

    prec = tp / np.maximum(tp + fp, 1)
    rec = tp / np.maximum(tp + fn, 1)
    lr = conf_mat[0, 1] + conf_mat[1, 0]
    ms = np.median(times[1:]) * 1000
    lines = [f'# {run.name}  (conf {a.conf}, imgsz {imgsz}, val {val_dir.parent.parent.name})', '',
             f'Runtime path: team85_final SignDetector + cv2.dnn {cv2.__version__}, '
             f'{len(images)} images, median {ms:.1f} ms/img CPU, ONNX {onnx.stat().st_size / 1e6:.1f} MB.',
             '', '| class | precision | recall | TP | FP | FN |', '|---|---|---|---|---|---|']
    for c, n in enumerate(NAMES):
        lines.append(f'| {n} | {prec[c]:.3f} | {rec[c]:.3f} | {tp[c]} | {fp[c]} | {fn[c]} |')
    lines.append(f'| **all** | {tp.sum() / max(tp.sum() + fp.sum(), 1):.3f} | '
                 f'{tp.sum() / max(tp.sum() + fn.sum(), 1):.3f} | {tp.sum()} | {fp.sum()} | {fn.sum()} |')
    lines += ['', f'Left<->right swaps: **{lr}**. Front-sign check (largest detection = lab 6 '
              f'label): **{front_ok}/{front_n}** = {front_ok / max(front_n, 1):.3f}', '',
              '| gt \\ pred | ' + ' | '.join(NAMES) + ' | missed |', '|---' * 7 + '|']
    for c, n in enumerate(NAMES):
        lines.append(f'| {n} | ' + ' | '.join(str(v) for v in conf_mat[c]) + ' |')
    report = ROOT / 'outputs' / 'reports' / f'det__{run.name}.md'
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
