"""Many candidate sign boxes per image, for labeling the small / far-away signs.

autolabel.py keeps each model's top-3 for the main sign; this keeps up to --top-k boxes
per model, drops whole-wall boxes, and merges the two models class-agnostically
(boxes overlapping >= --merge-iou become one, keeping each model's best score).
Output: data/labels/candidates.json  {image_id: [[x0, y0, x1, y1, owl, gdino], ...]}
(score 0 = that model did not propose it). Resumable; --redo to rerun.

    env -i HOME=$HOME PATH=/usr/bin:/bin .venv/bin/python scripts/candidates.py
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from lfv import labels as L  # noqa: E402
from lfv.zeroshot import grounding_dino, iou, owlv2  # noqa: E402

CANDIDATES = L.LABEL_DIR / 'candidates.json'
MIN_SCORE = {'owlv2': 0.05, 'gdino': 0.15}   # keep generously; the GUI filters further
MAX_AREA = 0.5


def merge(preds: dict[str, list], w: int, h: int, merge_iou: float) -> list[list[float]]:
    """Class-agnostic merge of both models' boxes -> [x0, y0, x1, y1, owl, gdino]."""
    out: list[list[float]] = []
    col = {'owlv2': 4, 'gdino': 5}
    flat = sorted(((b, m) for m, bs in preds.items() for b in bs), key=lambda t: -t[0][4])
    for b, m in flat:
        if b[4] < MIN_SCORE[m] or (b[2] - b[0]) * (b[3] - b[1]) >= MAX_AREA * w * h:
            continue
        hit = next((c for c in out if iou(c, b) >= merge_iou), None)
        if hit is None:
            hit = [round(v, 1) for v in b[:4]] + [0.0, 0.0]
            out.append(hit)
        hit[col[m]] = max(hit[col[m]], round(b[4], 3))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', default='mps')
    ap.add_argument('--top-k', type=int, default=20)
    ap.add_argument('--merge-iou', type=float, default=0.5)
    ap.add_argument('--limit', type=int, default=0)
    ap.add_argument('--redo', action='store_true')
    a = ap.parse_args()

    items = L.image_ids()[:a.limit or None]
    cands = {} if a.redo else L.load(CANDIDATES)
    todo = [i for i, _ in items if i not in cands]
    print(f'{len(items)} images, {len(todo)} to run', flush=True)
    if not todo:
        return
    models = {'owlv2': owlv2(a.device, top_k=a.top_k),
              'gdino': grounding_dino(a.device, top_k=a.top_k)}
    t0 = time.perf_counter()
    for n, image_id in enumerate(todo, 1):
        img = cv2.imread(str(L.RAW / image_id))
        h, w = img.shape[:2]
        cands[image_id] = merge({m: f(img) for m, f in models.items()}, w, h, a.merge_iou)
        if n % 20 == 0 or n == len(todo):
            L.save(CANDIDATES, cands)
            rate = (time.perf_counter() - t0) / n
            print(f'{n}/{len(todo)}  {rate:.2f} s/img  eta {rate * (len(todo) - n) / 60:.1f} min',
                  flush=True)
    sizes = [len(v) for v in cands.values()]
    print(f'done: {len(cands)} images, {sum(sizes)} candidates (max {max(sizes)}/image)')


if __name__ == '__main__':
    main()
