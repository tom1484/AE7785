"""Train a YOLO detector on an exported dataset (scripts/export_yolo.py).

    python scripts/train_det.py --data data/yolo/holdout-2026F_imgs/data.yaml --model yolo11n.pt --imgsz 416

Run name: <model>_<dataset>_<imgsz>. Weights download to outputs/weights, runs go to
outputs/runs/det. fliplr must stay 0 (mirroring swaps left and right arrows).
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--model', default='yolo11n.pt')
    ap.add_argument('--imgsz', type=int, default=416)
    ap.add_argument('--epochs', type=int, default=100)
    ap.add_argument('--batch', type=int, default=16)
    ap.add_argument('--device', default='mps')
    ap.add_argument('--name', default='')
    a = ap.parse_args()

    weights = ROOT / 'outputs' / 'weights'
    weights.mkdir(parents=True, exist_ok=True)
    os.chdir(weights)                                   # ultralytics downloads into cwd
    from ultralytics import YOLO

    data = Path(a.data).resolve() if Path(a.data).is_absolute() else (ROOT / a.data).resolve()
    name = a.name or f'{Path(a.model).stem}_{data.parent.name}_{a.imgsz}'
    YOLO(a.model).train(
        data=str(data), imgsz=a.imgsz, epochs=a.epochs, batch=a.batch, device=a.device,
        project=str(ROOT / 'outputs' / 'runs' / 'det'), name=name, exist_ok=True,
        fliplr=0.0,            # left/right arrows: never mirror
        patience=30, seed=0, deterministic=False, plots=True, verbose=False,
    )


if __name__ == '__main__':
    main()
