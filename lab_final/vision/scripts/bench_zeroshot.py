"""Benchmark zero-shot detectors for bootstrapping sign boxes.

For a fixed sample of lab_6 images (one sign per image), run each detector with text
prompts, keep the top-1 box, and record warm per-image latency. Writes a contact sheet
per model (outputs/review/bench_<model>.jpg) and a summary table.

    python scripts/bench_zeroshot.py --models yoloe,owlv2 --device mps
Run with conda deactivated (the humble env's libomp clashes with torch's).
"""

from __future__ import annotations

import argparse
import random
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT.parent / 'lab_6' / 'data' / 'raw'
OUT = ROOT / 'outputs' / 'review'
CLASS = {1: 'left', 2: 'right', 3: 'do_not_enter', 4: 'stop', 5: 'goal'}
PROMPTS = ['arrow', 'stop sign', 'star', 'u-turn arrow sign']


def sample(n_per_class: int, seed: int) -> list[tuple[Path, int]]:
    rng = random.Random(seed)
    items: dict[int, list[tuple[Path, int]]] = {}
    for d in sorted(RAW.iterdir()):
        for line in (d / 'labels.txt').read_text().splitlines():
            if not line.strip():
                continue
            k, v = (s.strip() for s in line.split(','))
            if int(v) in CLASS:
                items.setdefault(int(v), []).append((d / f'{k}.png', int(v)))
    out = []
    for c in sorted(items):
        out += rng.sample(items[c], n_per_class)
    return out


# ---- detectors: each returns fn(bgr) -> (x0, y0, x1, y1, score) | None --------
def make_ultralytics(weights: str, device: str, world: bool):
    from ultralytics import YOLO, YOLOE
    if world:
        m = YOLO(weights)
        m.set_classes(PROMPTS)
    else:
        m = YOLOE(weights)
        m.set_classes(PROMPTS, m.get_text_pe(PROMPTS))

    def run(bgr):
        r = m.predict(bgr, conf=0.01, device=device, verbose=False)[0]
        if not len(r.boxes):
            return None
        i = int(r.boxes.conf.argmax())
        return (*r.boxes.xyxy[i].tolist(), float(r.boxes.conf[i]))
    return run


def make_owlv2(device: str, name='google/owlv2-base-patch16-ensemble'):
    import torch
    from transformers import Owlv2ForObjectDetection, Owlv2Processor
    proc = Owlv2Processor.from_pretrained(name)
    model = Owlv2ForObjectDetection.from_pretrained(name).to(device).eval()

    def run(bgr):
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        inp = proc(text=[PROMPTS], images=rgb, return_tensors='pt').to(device)
        with torch.no_grad():
            out = model(**inp)
        # OWLv2 pads to a square; boxes are relative to the padded size.
        s = max(h, w)
        res = proc.post_process_grounded_object_detection(out, threshold=0.0,
                                                 target_sizes=torch.tensor([[s, s]]))[0]
        if not len(res['scores']):
            return None
        i = int(res['scores'].argmax())
        x0, y0, x1, y1 = res['boxes'][i].tolist()
        return (max(0, x0), max(0, y0), min(w, x1), min(h, y1), float(res['scores'][i]))
    return run


def make_gdino(device: str, name='IDEA-Research/grounding-dino-tiny'):
    import torch
    from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor
    proc = AutoProcessor.from_pretrained(name)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(name).to(device).eval()
    text = '. '.join(PROMPTS) + '.'

    def run(bgr):
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        inp = proc(images=rgb, text=text, return_tensors='pt').to(device)
        with torch.no_grad():
            out = model(**inp)
        res = proc.post_process_grounded_object_detection(
            out, inp.input_ids, threshold=0.05, text_threshold=0.05, target_sizes=[(h, w)])[0]
        if not len(res['scores']):
            return None
        i = int(res['scores'].argmax())
        return (*res['boxes'][i].tolist(), float(res['scores'][i]))
    return run


MODELS = {
    'yoloe': lambda dev: make_ultralytics('yoloe-11s-seg.pt', dev, world=False),
    'yoloe-l': lambda dev: make_ultralytics('yoloe-11l-seg.pt', dev, world=False),
    'yoloworld': lambda dev: make_ultralytics('yolov8s-worldv2.pt', dev, world=True),
    'owlv2': make_owlv2,
    'gdino': make_gdino,
}


def sheet(images, results, labels, path: Path, cols=6):
    tiles = []
    for img, r, c in zip(images, results, labels):
        t = img.copy()
        if r is not None:
            x0, y0, x1, y1, s = r
            cv2.rectangle(t, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 3)
            cv2.putText(t, f'{s:.2f}', (int(x0), max(14, int(y0) - 4)), 0, 0.6, (0, 255, 0), 2)
        cv2.putText(t, CLASS[c], (6, 22), 0, 0.7, (255, 255, 255), 2)
        tiles.append(cv2.resize(t, (205, 154)))
    while len(tiles) % cols:
        tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)]
    cv2.imwrite(str(path), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 85])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--models', default='yoloe,yoloworld,owlv2,gdino')
    ap.add_argument('--device', default='mps')
    ap.add_argument('--per-class', type=int, default=6)
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()

    items = sample(a.per_class, a.seed)
    images = [cv2.imread(str(p)) for p, _ in items]
    labels = [c for _, c in items]
    OUT.mkdir(parents=True, exist_ok=True)
    print(f'{len(items)} images, device={a.device}')
    print('| model | load s | warm ms/img | found | mean top-1 score |')
    print('|---|---|---|---|---|')
    for name in a.models.split(','):
        t0 = time.perf_counter()
        try:
            run = MODELS[name](a.device)
            run(images[0])                      # warm-up (graph build / MPS kernels)
        except Exception as e:                  # report and continue with other models
            print(f'| {name} | failed: {type(e).__name__}: {str(e)[:120]} |')
            continue
        load = time.perf_counter() - t0
        t1 = time.perf_counter()
        results = [run(img) for img in images]
        ms = (time.perf_counter() - t1) / len(images) * 1000
        found = [r for r in results if r is not None]
        mean = np.mean([r[4] for r in found]) if found else 0.0
        print(f'| {name} | {load:.1f} | {ms:.0f} | {len(found)}/{len(images)} | {mean:.2f} |')
        sheet(images, results, labels, OUT / f'bench_{name}.jpg')


if __name__ == '__main__':
    main()
