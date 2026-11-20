"""Propose a sign box for every lab_6 image with two zero-shot detectors.

Runs OWLv2 and Grounding DINO on every image, keeps each model's top-3 boxes, fuses
the top-1s into a proposal, flags suspicious results, and writes data/labels/auto.json.
Resumable: images already in auto.json are skipped (use --redo to rerun everything).
Human labels live in labels.json (scripts/review.py) and are never touched here.

    env -i HOME=$HOME PATH=/usr/bin:/bin .venv/bin/python scripts/autolabel.py
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

AGREE_IOU = 0.8
HUGE_AREA = 0.5           # fraction of the image
LOW_SCORE = {'owlv2': 0.3, 'gdino': 0.35}
EMPTY_SCORE = {'owlv2': 0.3, 'gdino': 0.4}


def analyse(cls: int, preds: dict[str, list], w: int, h: int) -> dict:
    """Fuse the two models' top-1 boxes and flag anything worth a closer look."""
    flags = []
    tops = {m: (p[0] if p else None) for m, p in preds.items()}
    area = lambda b: (b[2] - b[0]) * (b[3] - b[1]) / (w * h)  # noqa: E731

    if cls == 0:
        if any(t and t[4] >= EMPTY_SCORE[m] for m, t in tops.items()):
            flags.append('sign_on_empty')
        return {'flags': flags, 'proposal': [], 'agree_iou': None}

    if any(t is None for t in tops.values()):
        flags.append('no_box')
    for m, t in tops.items():
        if t and t[4] < LOW_SCORE[m]:
            flags.append('low_score')
            break
    a, b = tops['owlv2'], tops['gdino']
    agree = iou(a, b) if a and b else 0.0
    if a and b and agree < AGREE_IOU:
        flags.append('disagree')

    # OWLv2 first: once whole-wall boxes are filtered out, it is the tighter one (it left
    # out a paper scrap next to an arrow that GDINO included).
    sane = [t for t in (a, b) if t and area(t) < HUGE_AREA]
    if a and b and agree >= AGREE_IOU and a in sane and b in sane:
        box = [(a[i] + b[i]) / 2 for i in range(4)]
    elif sane:
        box = list(sane[0][:4])
    else:
        box = list((b or a)[:4]) if (a or b) else None
    if box is not None:
        if area(box + [0]) >= HUGE_AREA:
            flags.append('huge_box')
        if box[0] <= 1 or box[1] <= 1 or box[2] >= w - 1 or box[3] >= h - 1:
            flags.append('edge')
    proposal = [[round(v, 1) for v in box] + [cls]] if box is not None else []
    return {'flags': sorted(set(flags)), 'proposal': proposal, 'agree_iou': round(agree, 3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', default='mps')
    ap.add_argument('--limit', type=int, default=0, help='only the first N images (testing)')
    ap.add_argument('--redo', action='store_true')
    ap.add_argument('--reanalyse', action='store_true',
                    help='recompute flags/proposals from stored predictions (no models)')
    a = ap.parse_args()

    items = L.image_ids()
    if a.limit:
        items = items[:a.limit]
    auto = {} if a.redo else L.load(L.AUTO)
    if a.reanalyse:
        for r in auto.values():
            r.update(analyse(r['cls'], r['preds'], *r['size']))
        L.save(L.AUTO, auto)
        print_flags(auto)
        return
    todo = [(i, c) for i, c in items if i not in auto]
    print(f'{len(items)} images, {len(todo)} to run', flush=True)
    if not todo:
        return
    models = {'owlv2': owlv2(a.device), 'gdino': grounding_dino(a.device)}

    t0 = time.perf_counter()
    for n, (image_id, cls) in enumerate(todo, 1):
        img = cv2.imread(str(L.RAW / image_id))
        h, w = img.shape[:2]
        preds = {m: [[round(v, 3) for v in b] for b in f(img)] for m, f in models.items()}
        auto[image_id] = {'cls': cls, 'size': [w, h], 'preds': preds,
                          **analyse(cls, preds, w, h)}
        if n % 20 == 0 or n == len(todo):
            L.save(L.AUTO, auto)
            rate = (time.perf_counter() - t0) / n
            print(f'{n}/{len(todo)}  {rate:.2f} s/img  eta {rate * (len(todo) - n) / 60:.1f} min',
                  flush=True)

    print_flags(auto)


def print_flags(auto: dict) -> None:
    counts = {}
    for r in auto.values():
        for f in r['flags']:
            counts[f] = counts.get(f, 0) + 1
    n_flagged = sum(1 for r in auto.values() if r['flags'])
    print(f'{len(auto)} images, {n_flagged} flagged:', counts or 'none')


if __name__ == '__main__':
    main()
