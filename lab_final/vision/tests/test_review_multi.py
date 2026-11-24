import importlib.util
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
spec = importlib.util.spec_from_file_location('rm', ROOT / 'scripts' / 'review_multi.py')
rm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rm)


@pytest.fixture
def rv(tmp_path):
    (tmp_path / 'f').mkdir()
    for n in ('a', 'b'):
        cv2.imwrite(str(tmp_path / 'f' / f'{n}.png'), np.zeros((308, 410, 3), np.uint8))
    auto = {'f/a.png': {'cls': 1}, 'f/b.png': {'cls': 0}}
    first = {'f/a.png': {'boxes': [[100, 100, 200, 160, 1]], 'reviewed': True},
             'f/b.png': {'boxes': [], 'reviewed': True}}
    cands = {
        'f/a.png': [[101, 100, 199, 160, 0.9, 0.5]],                 # = the existing box
        'f/b.png': [[300, 150, 310, 160, 0.3, 0.0],                  # small far sign
                    [20, 140, 30, 150, 0.0, 0.4],
                    [50, 50, 60, 60, 0.05, 0.1]],                    # below thresholds
    }
    saved = []
    r = rm.MultiReviewer(auto, first, {}, cands, raw=tmp_path, save=lambda d: saved.append(1))
    r.saved, r.first_in = saved, first
    return r


def test_priority_puts_unclaimed_candidates_first(rv):
    assert rv.ids == ['f/b.png', 'f/a.png']
    assert len(rv.unclaimed('f/b.png', [])) == 2          # third is below thresholds
    assert rv.unclaimed('f/a.png', rv.saved_boxes('f/a.png')) == []   # claimed


def test_cycle_and_adopt_candidate(rv):
    rv.key(ord('c'))
    assert rv.sel_cand[:4] == [20, 140, 30, 150]           # left to right
    rv.key(ord('c'))
    assert rv.sel_cand[:4] == [300, 150, 310, 160]
    rv.key(ord('4'))
    assert rv.boxes == [[300, 150, 310, 160, 4]] and rv.active == 4
    assert len(rv.unclaimed(rv.image_id, rv.boxes)) == 1


def test_click_selects_candidate_then_class(rv):
    rv.mouse(cv2.EVENT_LBUTTONDOWN, 50, 290)               # window px of (25, 145)
    rv.mouse(cv2.EVENT_LBUTTONUP, 50, 290)
    assert rv.sel_cand is not None and rv.drag is None
    rv.key(ord('2'))
    assert rv.boxes[-1] == [20, 140, 30, 150, 2]


def test_thresholds(rv):
    for _ in range(4):                                     # x0.7^4: 0.036 / 0.072
        rv.key(ord('['))
    assert len(rv.unclaimed('f/b.png', [])) == 3
    for _ in range(5):                                     # one step above the default
        rv.key(ord(']'))
    assert len(rv.unclaimed('f/b.png', [])) == 1           # gdino 0.4 < 0.43 now hidden


def test_zoom_keeps_cursor_point_and_maps_draws(rv):
    rv.mouse(cv2.EVENT_MOUSEMOVE, 600, 300)
    p = rv.to_img(600, 300)
    rv.key(ord('='))
    rv.key(ord('='))
    assert rv.zoom == 4
    q = rv.to_img(600, 300)
    assert abs(p[0] - q[0]) < 1e-6 and abs(p[1] - q[1]) < 1e-6
    # A 16x16 window-px drag at x4 zoom (scale 8) is a 2x2 image-px box.
    rv.mouse(cv2.EVENT_LBUTTONDOWN, 600, 300)
    rv.mouse(cv2.EVENT_MOUSEMOVE, 616, 316)
    rv.mouse(cv2.EVENT_LBUTTONUP, 616, 316)
    b = rv.boxes[-1]
    assert abs((b[2] - b[0]) - 2) < 1e-6 and abs(b[0] - p[0]) < 1e-6
    rv.key(ord('0'))
    assert rv.zoom == 1 and rv.view == (0.0, 0.0)


def test_drag_inside_candidate_draws_a_box(rv):
    rv.mouse(cv2.EVENT_LBUTTONDOWN, 42, 282)               # inside the (20..30) candidate
    rv.mouse(cv2.EVENT_MOUSEMOVE, 56, 296)
    rv.mouse(cv2.EVENT_LBUTTONUP, 56, 296)
    assert rv.boxes == [[21, 141, 28, 148, 1]] and rv.sel_cand is None


def test_pan_clamps_and_candidate_follow(rv):
    rv.set_zoom(4, (0, 0))
    for _ in range(20):
        rv.key(ord('a'))
        rv.key(ord('w'))
    assert rv.view == (0.0, 0.0)
    for _ in range(20):
        rv.key(ord('d'))
    assert abs(rv.view[0] - (410 - 410 / 4)) < 1e-6
    rv.key(ord('c'))                                        # (20..30): view follows to the left
    assert rv.view[0] == 0.0


def test_accept_writes_multi_only(rv):
    rv.key(ord('c'))
    rv.key(ord('1'))
    rv.key(ord(' '))
    assert rv.multi['f/b.png']['boxes'] == [[20, 140, 30, 150, 1]]
    assert rv.first_in['f/b.png']['boxes'] == []            # first pass untouched
    assert rv.saved and rv.image_id == 'f/a.png'
    rv.go(0)
    assert rv.boxes == [[20, 140, 30, 150, 1]]              # reloads from multi


def test_render_sizes(rv):
    for z in (1, 2, 4, 8):
        rv.set_zoom(z)
        assert rv.render().shape == (308 * 2 + rm.BAR, 410 * 2, 3)
    rv.key(ord('h'))
    rv.render()
