"""Cross-check every labeled box with the lab 6 YOLOv8-cls classifier.

The classifier was trained on whole 410x308 frames with one sign at a typical size, so
each box is shown to it the same way: a window with the camera's aspect ratio centered
on the box, sized so the box fills about the usual fraction of the frame (two framings,
probabilities averaged). Preprocessing = lab_6 model_grader.predict (squash to 224,
INTER_AREA, BGR->RGB, /255).

Writes outputs/reports/validate_cls.md (table of mismatches), contact sheets
outputs/review/mismatch_*.jpg, and data/labels/mismatches.txt (image ids, for
`review_multi.py --ids data/labels/mismatches.txt`).

    env -i HOME=$HOME PATH=/usr/bin:/bin .venv/bin/python scripts/validate_cls.py
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from lfv import labels as L  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT.parent / 'lab_6' / 'outputs' / 'models' / 'final_yolov8m_cls_aug_4folders224.onnx'
MULTI = L.LABEL_DIR / 'multi.json'
FILL = (0.45, 0.30)        # box height / window height for the two framings
IMGSZ = 224


def window(img: np.ndarray, box, fill: float) -> np.ndarray:
    """Camera-aspect window around `box`; out-of-image parts replicate the border."""
    h, w = img.shape[:2]
    cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
    bh, bw = box[3] - box[1], box[2] - box[0]
    wh = max(bh / fill, bw / fill * h / w, 24.0)       # also fit wide boxes
    ww = wh * w / h
    x0, y0 = int(round(cx - ww / 2)), int(round(cy - wh / 2))
    x1, y1 = x0 + int(round(ww)), y0 + int(round(wh))
    pad = max(0, -x0, -y0, x1 - w, y1 - h)
    if pad:
        img = cv2.copyMakeBorder(img, pad, pad, pad, pad, cv2.BORDER_REPLICATE)
        x0, y0, x1, y1 = x0 + pad, y0 + pad, x1 + pad, y1 + pad
    return img[y0:y1, x0:x1]


def classify(net, crop: np.ndarray) -> np.ndarray:
    x = cv2.resize(crop, (IMGSZ, IMGSZ), interpolation=cv2.INTER_AREA)
    x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    net.setInput(np.ascontiguousarray(x.transpose(2, 0, 1)[None]))
    return np.asarray(net.forward()).reshape(-1)


def tile(img, box, crop, text_lines, size=(300, 225)):
    """Full image with the box drawn | the crop the classifier saw, plus text."""
    full = img.copy()
    x0, y0, x1, y1 = (int(round(v)) for v in box[:4])
    cv2.rectangle(full, (x0, y0), (x1, y1), (0, 0, 255), 2)
    left = cv2.resize(full, size)
    right = cv2.resize(crop, (size[1] * crop.shape[1] // crop.shape[0], size[1]))
    t = np.hstack([left, right])
    bar = np.full((44, t.shape[1], 3), 25, np.uint8)
    for k, s in enumerate(text_lines):
        cv2.putText(bar, s, (6, 17 + 20 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
    return np.vstack([t, bar])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=str(MODEL))
    ap.add_argument('--per-sheet', type=int, default=12)
    a = ap.parse_args()

    net = cv2.dnn.readNetFromONNX(a.model)
    multi, first = L.load(MULTI), L.load(L.LABELS)
    rows, tiles, n_boxes = [], [], 0
    for image_id in sorted(set(first) | set(multi)):
        rec = multi.get(image_id) if multi.get(image_id, {}).get('reviewed') else first[image_id]
        img = cv2.imread(str(L.RAW / image_id))
        for k, box in enumerate(rec['boxes']):
            n_boxes += 1
            crops = [window(img, box, f) for f in FILL]
            probs = np.mean([classify(net, c) for c in crops], axis=0)
            pred, label = int(probs.argmax()), int(box[4])
            if pred == label:
                continue
            h = box[3] - box[1]
            rows.append((image_id, k, label, pred, float(probs[pred]), float(probs[label]), h))
            tiles.append(tile(img, box, crops[0], [
                f'{image_id} #{k}  h={h:.0f}px',
                f'label {L.CLASS_NAMES[label]}  vs  cls {L.CLASS_NAMES[pred]} '
                f'{probs[pred]:.2f} (label p {probs[label]:.2f})']))

    out_rep = ROOT / 'outputs' / 'reports'
    out_rev = ROOT / 'outputs' / 'review'
    out_rep.mkdir(parents=True, exist_ok=True)
    out_rev.mkdir(parents=True, exist_ok=True)
    for old in out_rev.glob('mismatch_*.jpg'):
        old.unlink()
    order = sorted(range(len(rows)), key=lambda i: -rows[i][4])     # most confident first
    for s in range(0, len(order), a.per_sheet):
        chunk = [tiles[i] for i in order[s:s + a.per_sheet]]
        w = max(t.shape[1] for t in chunk)
        chunk = [cv2.copyMakeBorder(t, 0, 4, 0, w - t.shape[1] + 4, cv2.BORDER_CONSTANT)
                 for t in chunk]
        cols = 2
        while len(chunk) % cols:
            chunk.append(np.zeros_like(chunk[0]))
        sheet = np.vstack([np.hstack(chunk[i:i + cols]) for i in range(0, len(chunk), cols)])
        cv2.imwrite(str(out_rev / f'mismatch_{s // a.per_sheet + 1:02d}.jpg'), sheet,
                    [cv2.IMWRITE_JPEG_QUALITY, 85])

    lines = [f'# Label check with lab 6 yolo-cls ({Path(a.model).name})', '',
             f'{n_boxes} boxes, {len(rows)} mismatches. Most confident disagreement first.', '',
             '| image | box | label | cls says | p(cls) | p(label) | box h px |',
             '|---|---|---|---|---|---|---|']
    for i in order:
        r = rows[i]
        lines.append(f'| {r[0]} | {r[1]} | {L.CLASS_NAMES[r[2]]} | {L.CLASS_NAMES[r[3]]} | '
                     f'{r[4]:.2f} | {r[5]:.2f} | {r[6]:.0f} |')
    (out_rep / 'validate_cls.md').write_text('\n'.join(lines) + '\n')
    (L.LABEL_DIR / 'mismatches.txt').write_text(
        '\n'.join(dict.fromkeys(rows[i][0] for i in order)) + '\n')
    print(f'{n_boxes} boxes, {len(rows)} mismatches in '
          f'{len(set(r[0] for r in rows))} images -> outputs/reports/validate_cls.md')


if __name__ == '__main__':
    main()
