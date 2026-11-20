"""Second labeling pass: box EVERY sign in sight, including small / far-away ones.

Starts from the first-pass boxes (labels.json) and writes data/labels/multi.json; the
first pass is never modified. Zero-shot candidate boxes (scripts/candidates.py) are shown
as thin yellow boxes; adopt one by selecting it and pressing its class key. Images with
the most unclaimed candidates come first.

    env -i HOME=$HOME PATH=/usr/bin:/bin .venv/bin/python scripts/review_multi.py
    ... --order seq | --unreviewed

Keys
    space / enter   accept this image's boxes, next image
    right / .   next (no change)     left / ,   previous     f   next unreviewed
    1..5            active class (1 left 2 right 3 do_not_enter 4 stop 5 goal); also sets
                    the class of the selected box, or adopts the selected candidate
    drag (left)     new box with the active class
    click (left)    select a box, else a candidate     right-click   delete box under cursor
    c               select the next unclaimed candidate (view follows it when zoomed)
    x / backspace   delete selected box     e   remove all boxes     r   reset image
    = / -           zoom in / out at the cursor     0   zoom reset     w a s d   pan
    [ / ]           show more / fewer candidates (score thresholds)   m   candidates on/off
    h               help on/off     q / esc   quit (accepted images are already saved)
Policy: box every sign whose class you can tell, also partly visible ones.
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
from lfv.zeroshot import iou  # noqa: E402

MULTI = L.LABEL_DIR / 'multi.json'
CANDIDATES = L.LABEL_DIR / 'candidates.json'
SCALE = 2
BAR = 64
ZOOMS = (1, 2, 4, 8)
CLAIM_IOU = 0.3                    # a candidate overlapping a box this much is "claimed"
THRESH0 = (0.15, 0.30)             # default (OWLv2, GDINO) display thresholds
COLORS = {1: (255, 160, 40), 2: (40, 200, 255), 3: (60, 60, 255),
          4: (200, 60, 255), 5: (60, 255, 60)}
CAND_COLOR = (0, 230, 255)
KEYS_NEXT = {ord('.'), 63235, 65363, 2555904}
KEYS_PREV = {ord(','), 63234, 65361, 2424832}
KEYS_ACCEPT = {ord(' '), 13, 10}
KEYS_DELETE = {ord('x'), 127, 8, 63272, 65535}
KEYS_QUIT = {ord('q'), 27}


class MultiReviewer:
    """State and logic only (no window calls), so tests can drive it."""

    def __init__(self, auto: dict, first: dict, multi: dict, cands: dict,
                 order: str = 'priority', unreviewed: bool = False, raw: Path = L.RAW,
                 save=lambda multi: L.save(MULTI, multi)):
        self.auto, self.first, self.multi, self.cands = auto, first, multi, cands
        self.raw, self._save = raw, save
        self.thresh = list(THRESH0)
        self.active = 1
        self.show_cands = True
        self.show_help = False
        self.done = False
        self._img_cache: dict[str, np.ndarray] = {}
        ids = sorted(auto)
        if order == 'priority':
            ids.sort(key=lambda i: (-len(self.unclaimed(i, self.saved_boxes(i))), i))
        if unreviewed:
            ids = [i for i in ids if not multi.get(i, {}).get('reviewed')]
        self.ids = ids
        self.idx = 0
        self.mouse_xy = (0, 0)
        self.load_current()

    # ---- data ------------------------------------------------------------
    @property
    def image_id(self) -> str:
        return self.ids[self.idx]

    def saved_boxes(self, image_id: str) -> list[list[float]]:
        src = self.multi.get(image_id) or self.first.get(image_id) or {'boxes': []}
        return [list(b) for b in src['boxes']]

    def visible_cands(self, image_id: str) -> list[list[float]]:
        return [c for c in self.cands.get(image_id, [])
                if c[4] >= self.thresh[0] or c[5] >= self.thresh[1]]

    def unclaimed(self, image_id: str, boxes) -> list[list[float]]:
        return [c for c in self.visible_cands(image_id)
                if all(iou(c, b) < CLAIM_IOU for b in boxes)]

    def load_current(self) -> None:
        self.boxes = self.saved_boxes(self.image_id)
        self.sel_box: int | None = None
        self.sel_cand: list[float] | None = None
        self.drag = None
        self.zoom, self.view = 1, (0.0, 0.0)
        self._cand_cursor = -1

    def image(self) -> np.ndarray:
        i = self.image_id
        if i not in self._img_cache:
            if len(self._img_cache) > 64:
                self._img_cache.clear()
            self._img_cache[i] = cv2.imread(str(self.raw / i))
        return self._img_cache[i]

    def size(self) -> tuple[int, int]:
        h, w = self.image().shape[:2]
        return w, h

    def go(self, idx: int) -> None:
        self.idx = max(0, min(len(self.ids) - 1, idx))
        self.load_current()

    def accept(self) -> None:
        self.multi[self.image_id] = {
            'boxes': [[round(v, 1) for v in b[:4]] + [int(b[4])] for b in self.boxes],
            'reviewed': True,
            'time': time.strftime('%Y-%m-%dT%H:%M:%S'),
        }
        self._save(self.multi)

    def n_reviewed(self) -> int:
        return sum(1 for i in self.ids if self.multi.get(i, {}).get('reviewed'))

    # ---- view transform ----------------------------------------------------
    def scale(self) -> float:
        return SCALE * self.zoom

    def to_img(self, x: float, y: float) -> tuple[float, float]:
        w, h = self.size()
        ix, iy = self.view[0] + x / self.scale(), self.view[1] + y / self.scale()
        return min(max(ix, 0.0), w), min(max(iy, 0.0), h)

    def to_win(self, ix: float, iy: float) -> tuple[int, int]:
        return (int(round((ix - self.view[0]) * self.scale())),
                int(round((iy - self.view[1]) * self.scale())))

    def set_view(self, cx: float, cy: float) -> None:
        """Center the view on image point (cx, cy), clamped to the image."""
        w, h = self.size()
        vw, vh = w / self.zoom, h / self.zoom
        self.view = (min(max(cx - vw / 2, 0.0), w - vw), min(max(cy - vh / 2, 0.0), h - vh))

    def set_zoom(self, z: int, at: tuple[float, float] | None = None) -> None:
        """Zoom keeping image point `at` (default: view center) under the same window pixel."""
        w, h = self.size()
        if at is None:
            at = (self.view[0] + w / self.zoom / 2, self.view[1] + h / self.zoom / 2)
        wx, wy = self.to_win(*at)
        self.zoom = z
        vx, vy = at[0] - wx / self.scale(), at[1] - wy / self.scale()
        vw, vh = w / z, h / z
        self.view = (min(max(vx, 0.0), w - vw), min(max(vy, 0.0), h - vh))

    # ---- input -------------------------------------------------------------
    def key(self, k: int) -> None:
        if k < 0:
            return
        ch = chr(k) if k < 256 else ''
        if k in KEYS_QUIT:
            self.done = True
        elif k in KEYS_ACCEPT:
            self.accept()
            if self.idx < len(self.ids) - 1:
                self.go(self.idx + 1)
        elif k in KEYS_NEXT:
            self.go(self.idx + 1)
        elif k in KEYS_PREV:
            self.go(self.idx - 1)
        elif ch == 'f':
            nxt = [j for j in range(self.idx + 1, len(self.ids))
                   if not self.multi.get(self.ids[j], {}).get('reviewed')]
            if nxt:
                self.go(nxt[0])
        elif ch and ch in '12345':
            self.set_class(int(ch))
        elif ch == 'c':
            self.next_candidate()
        elif k in KEYS_DELETE:
            if self.sel_box is not None:
                del self.boxes[self.sel_box]
                self.sel_box = None
        elif ch == 'e':
            self.boxes, self.sel_box = [], None
        elif ch == 'r':
            z, v = self.zoom, self.view
            self.load_current()
            self.zoom, self.view = z, v
        elif ch in ('=', '+'):
            i = ZOOMS.index(self.zoom)
            if i < len(ZOOMS) - 1:
                self.set_zoom(ZOOMS[i + 1], self.to_img(*self.mouse_xy))
        elif ch in ('-', '_'):
            i = ZOOMS.index(self.zoom)
            if i > 0:
                self.set_zoom(ZOOMS[i - 1], self.to_img(*self.mouse_xy))
        elif ch == '0':
            self.zoom, self.view = 1, (0.0, 0.0)
        elif ch in 'wasd' and ch:
            w, h = self.size()
            step = (w / self.zoom / 4, h / self.zoom / 4)
            dx, dy = {'a': (-1, 0), 'd': (1, 0), 'w': (0, -1), 's': (0, 1)}[ch]
            vw, vh = w / self.zoom, h / self.zoom
            cx = self.view[0] + vw / 2 + dx * step[0]
            cy = self.view[1] + vh / 2 + dy * step[1]
            self.set_view(cx, cy)
        elif ch == '[':
            self.thresh = [t * 0.7 for t in self.thresh]
        elif ch == ']':
            self.thresh = [t / 0.7 for t in self.thresh]
        elif ch == 'm':
            self.show_cands = not self.show_cands
        elif ch == 'h':
            self.show_help = not self.show_help

    def set_class(self, c: int) -> None:
        self.active = c
        if self.sel_box is not None:
            self.boxes[self.sel_box][4] = c
        elif self.sel_cand is not None:
            self.boxes.append(list(self.sel_cand[:4]) + [c])
            self.sel_box, self.sel_cand = len(self.boxes) - 1, None

    def next_candidate(self) -> None:
        cs = self.unclaimed(self.image_id, self.boxes) if self.show_cands else []
        if not cs:
            self.sel_cand = None
            return
        cs.sort(key=lambda c: (c[0], c[1]))           # left to right
        self._cand_cursor = (self._cand_cursor + 1) % len(cs)
        self.sel_cand, self.sel_box = cs[self._cand_cursor], None
        if self.zoom > 1:
            c = self.sel_cand
            self.set_view((c[0] + c[2]) / 2, (c[1] + c[3]) / 2)

    def mouse(self, event: int, x: int, y: int) -> None:
        self.mouse_xy = (x, y)
        ix, iy = self.to_img(x, y)
        # Press only records the point. Release: a drag draws a new box (even when it starts
        # inside a box or a big junk candidate); a plain click selects a box, else a candidate.
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag = (ix, iy, ix, iy)
        elif event == cv2.EVENT_MOUSEMOVE and self.drag is not None:
            self.drag = (self.drag[0], self.drag[1], ix, iy)
        elif event == cv2.EVENT_LBUTTONUP and self.drag is not None:
            x0, y0 = self.drag[:2]
            self.drag = None
            if abs(ix - x0) >= 2 and abs(iy - y0) >= 2:      # image px; small signs are small
                self.boxes.append([min(x0, ix), min(y0, iy), max(x0, ix), max(y0, iy),
                                   self.active])
                self.sel_box, self.sel_cand = len(self.boxes) - 1, None
                return
            hit = _smallest_at(self.boxes, ix, iy)
            cand = None
            if hit is None and self.show_cands:
                cs = self.unclaimed(self.image_id, self.boxes)
                j = _smallest_at(cs, ix, iy)
                cand = cs[j] if j is not None else None
            self.sel_box, self.sel_cand = hit, cand
        elif event == cv2.EVENT_RBUTTONDOWN:
            hit = _smallest_at(self.boxes, ix, iy)
            if hit is not None:
                del self.boxes[hit]
                self.sel_box = None

    # ---- drawing -------------------------------------------------------------
    def render(self) -> np.ndarray:
        w, h = self.size()
        s = self.scale()
        M = np.float32([[s, 0, -self.view[0] * s], [0, s, -self.view[1] * s]])
        img = cv2.warpAffine(self.image(), M, (w * SCALE, h * SCALE),
                             flags=cv2.INTER_NEAREST if self.zoom >= 4 else cv2.INTER_LINEAR)
        if self.show_cands:
            for c in self.unclaimed(self.image_id, self.boxes):
                p0, p1 = self.to_win(c[0], c[1]), self.to_win(c[2], c[3])
                sel = c is self.sel_cand or c == self.sel_cand
                cv2.rectangle(img, p0, p1, CAND_COLOR, 3 if sel else 1, cv2.LINE_AA)
                cv2.putText(img, f'o{c[4]:.2f} g{c[5]:.2f}', (p0[0], p1[1] + 13),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, CAND_COLOR, 1, cv2.LINE_AA)
        for i, b in enumerate(self.boxes):
            color = COLORS.get(int(b[4]), (255, 255, 255))
            p0, p1 = self.to_win(b[0], b[1]), self.to_win(b[2], b[3])
            cv2.rectangle(img, p0, p1, color, 4 if i == self.sel_box else 2)
            _label(img, L.CLASS_NAMES[int(b[4])], p0, color)
        if self.drag:
            x0, y0, x1, y1 = self.drag
            cv2.rectangle(img, self.to_win(x0, y0), self.to_win(x1, y1),
                          COLORS[self.active], 1)

        bar = np.full((BAR, img.shape[1], 3), 30, np.uint8)
        rev = self.multi.get(self.image_id, {}).get('reviewed')
        n_un = len(self.unclaimed(self.image_id, self.boxes))
        line1 = (f'[{self.idx + 1}/{len(self.ids)}] {self.image_id}   lab6: '
                 f'{L.CLASS_NAMES[self.auto[self.image_id]["cls"]]}   boxes {len(self.boxes)}   '
                 f'{"REVIEWED" if rev else "not reviewed"}')
        line2 = (f'unclaimed cand {n_un}   zoom x{self.zoom}   thr o{self.thresh[0]:.2f} '
                 f'g{self.thresh[1]:.2f}   done {self.n_reviewed()}/{len(self.ids)}   h = help')
        cv2.putText(bar, line1, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (120, 255, 120) if rev else (230, 230, 230), 1, cv2.LINE_AA)
        cv2.putText(bar, line2, (8, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 200, 255) if n_un else (180, 180, 180), 1, cv2.LINE_AA)
        # active class chip, right side
        name = f'active: {self.active} {L.CLASS_NAMES[self.active]}'
        (tw, _), _ = cv2.getTextSize(name, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        x = bar.shape[1] - tw - 16
        cv2.rectangle(bar, (x - 6, 8), (bar.shape[1] - 6, 32), COLORS[self.active], -1)
        cv2.putText(bar, name, (x, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
        out = np.vstack([img, bar])
        if self.show_help:
            _help(out)
        return out


def _smallest_at(boxes, x, y) -> int | None:
    inside = [i for i, b in enumerate(boxes) if b[0] <= x <= b[2] and b[1] <= y <= b[3]]
    return min(inside, key=lambda i: (boxes[i][2] - boxes[i][0]) * (boxes[i][3] - boxes[i][1])) \
        if inside else None


def _label(img, text, org, color):
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    x, y = org
    y = max(th + 6, y)
    cv2.rectangle(img, (x, y - th - 6), (x + tw + 6, y), color, -1)
    cv2.putText(img, text, (x + 3, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)


def _help(img):
    lines = [ln.rstrip() for ln in __doc__.split('Keys')[1].strip().splitlines()]
    overlay = img.copy()
    cv2.rectangle(overlay, (10, 10), (img.shape[1] - 10, 30 + 21 * len(lines)), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.8, img, 0.2, 0, img)
    for i, ln in enumerate(lines):
        cv2.putText(img, ln.strip() if not ln.startswith('    ') else ln[4:], (22, 34 + 21 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--order', choices=['priority', 'seq'], default='priority')
    ap.add_argument('--unreviewed', action='store_true')
    a = ap.parse_args()

    auto = L.load(L.AUTO)
    cands = L.load(CANDIDATES)
    if not auto or not cands:
        sys.exit('need data/labels/auto.json and candidates.json (scripts/candidates.py)')
    rv = MultiReviewer(auto, L.load(L.LABELS), L.load(MULTI), cands, a.order, a.unreviewed)
    if not rv.ids:
        sys.exit('nothing to review')
    first = next((j for j, i in enumerate(rv.ids) if not rv.multi.get(i, {}).get('reviewed')), 0)
    rv.go(first)

    win = 'multi-sign review (h = help)'
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, lambda ev, x, y, *_: rv.mouse(ev, x, y))
    while not rv.done:
        cv2.imshow(win, rv.render())
        rv.key(cv2.waitKeyEx(20))
        if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
            break
    cv2.destroyAllWindows()
    print(f'reviewed {rv.n_reviewed()}/{len(rv.ids)}; labels in {MULTI}')


if __name__ == '__main__':
    main()
