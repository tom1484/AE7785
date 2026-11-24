import importlib.util
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
spec = importlib.util.spec_from_file_location('review', ROOT / 'scripts' / 'review.py')
review = importlib.util.module_from_spec(spec)
spec.loader.exec_module(review)

from lfv.labels import FLAG_WEIGHT  # noqa: E402,F401


def rec(cls, flags=(), owl=None, gd=None, proposal=None):
    preds = {'owlv2': [owl] if owl else [], 'gdino': [gd] if gd else []}
    return {'cls': cls, 'size': [410, 308], 'preds': preds, 'flags': list(flags),
            'proposal': proposal if proposal is not None else [], 'agree_iou': 0.9}


@pytest.fixture
def setup(tmp_path):
    (tmp_path / 'f').mkdir()
    for n in ('a', 'b', 'c'):
        cv2.imwrite(str(tmp_path / 'f' / f'{n}.png'), np.zeros((308, 410, 3), np.uint8))
    auto = {
        'f/a.png': rec(1, owl=[10, 10, 50, 50, 0.9], gd=[12, 12, 50, 50, 0.5],
                       proposal=[[11, 11, 50, 50, 1]]),
        'f/b.png': rec(3, flags=['huge_box', 'disagree'], owl=[0, 0, 410, 308, 0.2],
                       gd=[100, 100, 200, 200, 0.4], proposal=[[100, 100, 200, 200, 3]]),
        'f/c.png': rec(0),
    }
    saved = []
    labels = {}
    rv = review.Reviewer(auto, labels, raw=tmp_path, save=lambda d: saved.append(dict(d)))
    return rv, labels, saved


def test_priority_order(setup):
    rv, _, _ = setup
    assert rv.ids[0] == 'f/b.png'          # flagged first


def test_accept_saves_proposal_and_advances(setup):
    rv, labels, saved = setup
    rv.key(ord(' '))
    assert labels['f/b.png']['boxes'] == [[100, 100, 200, 200, 3]]
    assert labels['f/b.png']['reviewed'] and not labels['f/b.png']['edited']
    assert saved and rv.image_id != 'f/b.png'


def test_draw_select_reclass_delete(setup):
    rv, labels, _ = setup
    rv.key(ord('e'))                        # clear
    assert rv.boxes == []
    rv.mouse(cv2.EVENT_LBUTTONDOWN, 20, 40)  # window px = image px * 2
    rv.mouse(cv2.EVENT_MOUSEMOVE, 120, 140)
    rv.mouse(cv2.EVENT_LBUTTONUP, 120, 140)
    assert rv.boxes == [[10, 20, 60, 70, 3]] and rv.selected == 0
    rv.key(ord('5'))
    assert rv.boxes[0][4] == 5
    rv.mouse(cv2.EVENT_LBUTTONDOWN, 300, 300)   # click outside: deselect, starts a drag
    rv.mouse(cv2.EVENT_LBUTTONUP, 300, 300)     # zero-size drag adds nothing
    assert len(rv.boxes) == 1 and rv.selected is None
    rv.mouse(cv2.EVENT_LBUTTONDOWN, 50, 50)     # inside the box: select
    rv.mouse(cv2.EVENT_LBUTTONUP, 50, 50)
    assert rv.selected == 0
    rv.key(ord('x'))
    assert rv.boxes == []
    rv.key(13)
    assert labels['f/b.png'] == {**labels['f/b.png'], 'boxes': [], 'edited': True}


def test_model_boxes_and_reset(setup):
    rv, _, _ = setup
    rv.key(ord('o'))
    assert rv.boxes == [[0, 0, 410, 308, 3]]
    rv.mouse(cv2.EVENT_RBUTTONDOWN, 10, 10)
    assert rv.boxes == []
    rv.key(ord('r'))
    assert rv.boxes == [[100, 100, 200, 200, 3]]


def test_navigation_bounds_and_unreviewed_jump(setup):
    rv, labels, _ = setup
    rv.key(63234)                            # left at the start stays
    assert rv.idx == 0
    rv.key(ord(' '))                         # accept b
    rv.go(0)
    rv.key(ord('f'))                         # next unreviewed after b
    assert rv.idx == 1
    for _ in range(5):
        rv.key(63235)
    assert rv.idx == len(rv.ids) - 1


def test_render_shape_and_help(setup):
    rv, _, _ = setup
    assert rv.render().shape == (308 * 2 + review.BAR, 410 * 2, 3)
    rv.key(ord('h'))
    assert rv.render().shape[0] == 308 * 2 + review.BAR


def test_quit(setup):
    rv, _, _ = setup
    rv.key(ord('q'))
    assert rv.done
