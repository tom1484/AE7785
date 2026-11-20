"""Review and fix sign boxes, one image at a time (OpenCV window).

Images come most-suspicious first (flags from autolabel.py, then low scores), then
everything else; all images are in the queue. Every accepted image is written to
data/labels/labels.json immediately, so you can quit and resume any time.

    env -i HOME=$HOME PATH=/usr/bin:/bin .venv/bin/python scripts/review.py
    ... scripts/review.py --order seq          # folder/file order instead of priority
    ... scripts/review.py --unreviewed         # skip images already reviewed

Keys
    space / enter   accept the boxes shown and go to the next image
    right  /  .     next image (no change)        left / ,   previous image
    f               jump to the next unreviewed image
    drag (left)     draw a new box (class = the image's label; change with 1-5)
                    Box EVERY sign whose class you can tell, also far-away ones and
                    ones in "empty" images (they would otherwise be learned as background).
    click (left)    select a box                   right-click   delete the box under it
    x / backspace   delete the selected box        e             no sign (remove all boxes)
    1..5            class of the selected box: 1 left 2 right 3 do_not_enter 4 stop 5 goal
    o / g / p       use OWLv2's / Grounding DINO's / the fused proposal box
    m               show/hide both models' boxes (thin)
    r               reset this image to its saved label (or the proposal)
    h               help on/off                    q / esc       quit (everything accepted is saved)
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from lfv import labels as L  # noqa: E402

SCALE = 2
BAR = 64                                        # status bar height (px)
COLORS = {1: (255, 160, 40), 2: (40, 200, 255), 3: (60, 60, 255),     # BGR
          4: (200, 60, 255), 5: (60, 255, 60)}
MODEL_COLORS = {'owlv2': (255, 255, 0), 'gdino': (255, 0, 255)}
KEYS_NEXT = {ord('.'), 63235, 65363, 2555904}
KEYS_PREV = {ord(','), 63234, 65361, 2424832}
KEYS_ACCEPT = {ord(' '), 13, 10}
KEYS_DELETE = {ord('x'), 127, 8, 63272, 65535}
KEYS_QUIT = {ord('q'), 27}


def priority(r: dict) -> tuple:
    """Sort key: most suspicious first."""
    weight = sum(L.FLAG_WEIGHT.get(f, 1) for f in r.get('flags', []))
    tops = [p[0][4] for p in r.get('preds', {}).values() if p]
    return (-weight, min(tops) if tops else 0.0)


class Reviewer:
    """All state and logic; no window calls, so it can be driven by tests."""

    def __init__(self, auto: dict, labels: dict, order: str = 'priority',
                 unreviewed: bool = False, raw: Path = L.RAW,
                 save=lambda labels: L.save(L.LABELS, labels)):
        self.auto, self.labels, self.raw, self._save = auto, labels, raw, save
        ids = sorted(auto)
        if order == 'priority':
            ids.sort(key=lambda i: (priority(auto[i]), i))
        if unreviewed:
            ids = [i for i in ids if not labels.get(i, {}).get('reviewed')]
        self.ids = ids
        self.idx = 0
        self.show_models = True
        self.show_help = False
        self.selected: int | None = None
        self.drag: tuple[int, int, int, int] | None = None   # image px x0, y0, x1, y1
        self.done = False
        self._img_cache: dict[str, np.ndarray] = {}
        self.load_current()

    # ---- current image -------------------------------------------------
    @property
    def image_id(self) -> str:
        return self.ids[self.idx]

    @property
    def rec(self) -> dict:
        return self.auto[self.image_id]

    def load_current(self) -> None:
        saved = self.labels.get(self.image_id)
        src = saved['boxes'] if saved else self.rec['proposal']
        self.boxes = [list(b) for b in src]
        self.selected = None
        self.drag = None

    def image(self) -> np.ndarray:
        i = self.image_id
        if i not in self._img_cache:
            if len(self._img_cache) > 64:
                self._img_cache.clear()
            self._img_cache[i] = cv2.imread(str(self.raw / i))
        return self._img_cache[i]

    def go(self, idx: int) -> None:
        self.idx = max(0, min(len(self.ids) - 1, idx))
        self.load_current()

    def accept(self) -> None:
        prop = self.rec['proposal']
        self.labels[self.image_id] = {
            'boxes': [[round(v, 1) for v in b[:4]] + [int(b[4])] for b in self.boxes],
            'reviewed': True,
            'edited': self.boxes != [list(b) for b in prop],
            'time': time.strftime('%Y-%m-%dT%H:%M:%S'),
        }
        self._save(self.labels)

    def n_reviewed(self) -> int:
        return sum(1 for i in self.ids if self.labels.get(i, {}).get('reviewed'))

    # ---- input -----------------------------------------------------------
    def key(self, k: int) -> None:
        if k < 0:
            return
        ch = chr(k) if k < 256 else ''
        if k in KEYS_QUIT:
            self.done = True
        elif k in KEYS_ACCEPT:
            self.accept()
            if self.idx == len(self.ids) - 1:
                return
            self.go(self.idx + 1)
        elif k in KEYS_NEXT:
            self.go(self.idx + 1)
        elif k in KEYS_PREV:
            self.go(self.idx - 1)
        elif ch == 'f':
            nxt = [j for j in range(self.idx + 1, len(self.ids))
                   if not self.labels.get(self.ids[j], {}).get('reviewed')]
            if nxt:
                self.go(nxt[0])
        elif k in KEYS_DELETE:
            if self.selected is not None:
                del self.boxes[self.selected]
                self.selected = None
        elif ch == 'e':
            self.boxes, self.selected = [], None
        elif ch in '12345' and ch:
            c = int(ch)
            if self.selected is not None:
                self.boxes[self.selected][4] = c
            elif len(self.boxes) == 1:
                self.boxes[0][4] = c
        elif ch in ('o', 'g'):
            preds = self.rec['preds']['owlv2' if ch == 'o' else 'gdino']
            if preds:
                self.boxes = [list(preds[0][:4]) + [self.default_cls()]]
                self.selected = 0
        elif ch == 'p':
            self.boxes = [list(b) for b in self.rec['proposal']]
            self.selected = None
        elif ch == 'r':
            self.load_current()
        elif ch == 'm':
            self.show_models = not self.show_models
        elif ch == 'h':
            self.show_help = not self.show_help

    def default_cls(self) -> int:
        c = self.rec['cls']
        return c if c in COLORS else 1

    def mouse(self, event: int, x: int, y: int) -> None:
        """x, y in window pixels."""
        ix, iy = x / SCALE, y / SCALE
        h, w = self.image().shape[:2]
        ix, iy = min(max(ix, 0), w), min(max(iy, 0), h)
        if event == cv2.EVENT_LBUTTONDOWN:
            hit = self.box_at(ix, iy)
            self.selected = hit
            self.drag = None if hit is not None else (ix, iy, ix, iy)
        elif event == cv2.EVENT_MOUSEMOVE and self.drag is not None:
            self.drag = (self.drag[0], self.drag[1], ix, iy)
        elif event == cv2.EVENT_LBUTTONUP and self.drag is not None:
            x0, y0, x1, y1 = self.drag
            self.drag = None
            if abs(x1 - x0) >= 4 and abs(y1 - y0) >= 4:
                self.boxes.append([min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1),
                                   self.default_cls()])
                self.selected = len(self.boxes) - 1
        elif event == cv2.EVENT_RBUTTONDOWN:
            hit = self.box_at(ix, iy)
            if hit is not None:
                del self.boxes[hit]
                self.selected = None

    def box_at(self, x: float, y: float) -> int | None:
        """Smallest box containing (x, y), so nested boxes stay selectable."""
        hits = [(b[2] - b[0]) * (b[3] - b[1]) for b in self.boxes]
        inside = [i for i, b in enumerate(self.boxes) if b[0] <= x <= b[2] and b[1] <= y <= b[3]]
        return min(inside, key=lambda i: hits[i]) if inside else None

    # ---- drawing -----------------------------------------------------------
    def render(self) -> np.ndarray:
        img = cv2.resize(self.image(), None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_LINEAR)
        s = lambda v: int(round(v * SCALE))  # noqa: E731
        if self.show_models:
            for m, preds in self.rec['preds'].items():
                if preds:
                    b = preds[0]
                    cv2.rectangle(img, (s(b[0]), s(b[1])), (s(b[2]), s(b[3])),
                                  MODEL_COLORS[m], 1, cv2.LINE_AA)
                    cv2.putText(img, f'{m} {b[4]:.2f}', (s(b[0]) + 3, s(b[3]) - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, MODEL_COLORS[m], 1, cv2.LINE_AA)
        for i, b in enumerate(self.boxes):
            color = COLORS.get(int(b[4]), (255, 255, 255))
            thick = 4 if i == self.selected else 2
            cv2.rectangle(img, (s(b[0]), s(b[1])), (s(b[2]), s(b[3])), color, thick)
            _label(img, L.CLASS_NAMES[int(b[4])], (s(b[0]), s(b[1])), color)
        if self.drag:
            x0, y0, x1, y1 = self.drag
            cv2.rectangle(img, (s(x0), s(y0)), (s(x1), s(y1)), (255, 255, 255), 1)

        bar = np.full((BAR, img.shape[1], 3), 30, np.uint8)
        saved = self.labels.get(self.image_id, {})
        state = ('REVIEWED' + (' (edited)' if saved.get('edited') else '')) if saved.get(
            'reviewed') else 'not reviewed'
        flags = ','.join(self.rec['flags']) or '-'
        iou = self.rec.get('agree_iou')
        line1 = (f'[{self.idx + 1}/{len(self.ids)}] {self.image_id}   label: '
                 f'{L.CLASS_NAMES[self.rec["cls"]]}   {state}')
        line2 = (f'flags: {flags}   iou: {iou if iou is not None else "-"}   '
                 f'reviewed {self.n_reviewed()}/{len(self.ids)}   h = help')
        cv2.putText(bar, line1, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (120, 255, 120) if saved.get('reviewed') else (230, 230, 230), 1, cv2.LINE_AA)
        cv2.putText(bar, line2, (8, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 200, 255) if self.rec['flags'] else (180, 180, 180), 1, cv2.LINE_AA)
        out = np.vstack([img, bar])
        if self.show_help:
            _help(out)
        return out


def _label(img, text, org, color):
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
    x, y = org
    y = max(th + 6, y)
    cv2.rectangle(img, (x, y - th - 6), (x + tw + 6, y), color, -1)
    cv2.putText(img, text, (x + 3, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)


def _help(img):
    lines = [ln.strip() for ln in __doc__.split('Keys')[1].strip().splitlines()]
    overlay = img.copy()
    cv2.rectangle(overlay, (10, 10), (img.shape[1] - 10, 30 + 22 * len(lines)), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.75, img, 0.25, 0, img)
    for i, ln in enumerate(lines):
        cv2.putText(img, ln, (22, 36 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--order', choices=['priority', 'seq'], default='priority')
    ap.add_argument('--unreviewed', action='store_true', help='only images not yet reviewed')
    a = ap.parse_args()

    auto = L.load(L.AUTO)
    if not auto:
        sys.exit('no data/labels/auto.json; run scripts/autolabel.py first')
    rv = Reviewer(auto, L.load(L.LABELS), a.order, a.unreviewed)
    if not rv.ids:
        sys.exit('nothing to review')
    # Resume at the first unreviewed image in the queue.
    first = next((j for j, i in enumerate(rv.ids) if not rv.labels.get(i, {}).get('reviewed')), 0)
    rv.go(first)

    win = 'review (h = help)'
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, lambda ev, x, y, *_: rv.mouse(ev, x, y))
    while not rv.done:
        cv2.imshow(win, rv.render())
        rv.key(cv2.waitKeyEx(20))
        if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:   # window closed
            break
    cv2.destroyAllWindows()
    print(f'reviewed {rv.n_reviewed()}/{len(rv.ids)}; labels in {L.LABELS}')


if __name__ == '__main__':
    main()
