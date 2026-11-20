"""Export the reviewed labels to an ultralytics YOLO detection dataset.

Labels: data/labels/multi.json (second pass, all signs). Class id = lab_6 id - 1, names
[left, right, do_not_enter, stop, goal] (the contract with team85_final). Images with no
boxes are kept as background images.

Split: --val-folder holds a whole raw folder out (frames inside a folder are near-duplicate
sequences, so a random split would leak). --val-folder none = everything in train
(final model).

    python scripts/export_yolo.py --val-folder 2026F_imgs   -> data/yolo/holdout-2026F_imgs/
    python scripts/export_yolo.py --val-folder none          -> data/yolo/all/
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from lfv import labels as L  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
MULTI = L.LABEL_DIR / 'multi.json'
NAMES = ['left', 'right', 'do_not_enter', 'stop', 'goal']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--val-folder', default='2026F_imgs', help="raw folder for val, or 'none'")
    a = ap.parse_args()

    multi = L.load(MULTI)
    unreviewed = [i for i, v in multi.items() if not v.get('reviewed')]
    if unreviewed:
        sys.exit(f'{len(unreviewed)} images not reviewed in multi.json')
    name = 'all' if a.val_folder == 'none' else f'holdout-{a.val_folder}'
    out = ROOT / 'data' / 'yolo' / name
    shutil.rmtree(out, ignore_errors=True)

    counts = {'train': [0, 0], 'val': [0, 0]}          # images, boxes
    for image_id, rec in sorted(multi.items()):
        folder, fname = image_id.split('/')
        split = 'val' if folder == a.val_folder else 'train'
        stem = f'{folder}__{Path(fname).stem}'
        img_dst = out / 'images' / split / f'{stem}{Path(fname).suffix}'
        lbl_dst = out / 'labels' / split / f'{stem}.txt'
        img_dst.parent.mkdir(parents=True, exist_ok=True)
        lbl_dst.parent.mkdir(parents=True, exist_ok=True)
        src = L.RAW / image_id
        img_dst.symlink_to(src.resolve())
        w, h = rec_size(src)
        lines = []
        for x0, y0, x1, y1, c in rec['boxes']:
            cx, cy, bw, bh = (x0 + x1) / 2 / w, (y0 + y1) / 2 / h, (x1 - x0) / w, (y1 - y0) / h
            lines.append(f'{int(c) - 1} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}')
        lbl_dst.write_text('\n'.join(lines) + ('\n' if lines else ''))
        counts[split][0] += 1
        counts[split][1] += len(lines)

    data = {'path': str(out), 'train': 'images/train', 'names': dict(enumerate(NAMES))}
    data['val'] = 'images/val' if counts['val'][0] else 'images/train'
    (out / 'data.yaml').write_text(yaml.safe_dump(data, sort_keys=False))
    print(f'{out}: train {counts["train"][0]} imgs / {counts["train"][1]} boxes, '
          f'val {counts["val"][0]} imgs / {counts["val"][1]} boxes')


_SIZES: dict[Path, tuple[int, int]] = {}


def rec_size(path: Path) -> tuple[int, int]:
    import cv2
    if path not in _SIZES:
        h, w = cv2.imread(str(path)).shape[:2]
        _SIZES[path] = (w, h)
    return _SIZES[path]


if __name__ == '__main__':
    main()
