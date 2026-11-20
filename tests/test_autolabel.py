import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
spec = importlib.util.spec_from_file_location('autolabel', ROOT / 'scripts' / 'autolabel.py')
al = importlib.util.module_from_spec(spec)
spec.loader.exec_module(al)

W, H = 410, 308


def run(cls, owl, gd):
    return al.analyse(cls, {'owlv2': [owl] if owl else [], 'gdino': [gd] if gd else []}, W, H)


def test_agreeing_boxes_are_averaged_and_unflagged():
    r = run(1, [100, 100, 200, 160, 0.9], [102, 100, 200, 162, 0.6])
    assert r['flags'] == [] and r['proposal'] == [[101.0, 100.0, 200.0, 161.0, 1]]


def test_disagreement_prefers_owlv2():
    r = run(2, [100, 100, 200, 160, 0.8], [90, 90, 250, 190, 0.4])
    assert r['flags'] == ['disagree'] and r['proposal'][0][:4] == [100, 100, 200, 160]


def test_whole_wall_box_is_skipped_for_the_other_model():
    r = run(3, [0, 0, 410, 308, 0.5], [120, 80, 220, 200, 0.5])
    assert 'disagree' in r['flags'] and 'huge_box' not in r['flags']
    assert r['proposal'][0][:4] == [120, 80, 220, 200]


def test_both_huge_is_flagged():
    r = run(3, [0, 0, 410, 308, 0.5], [5, 5, 405, 300, 0.5])
    assert 'huge_box' in r['flags']


def test_missing_and_low_score_and_edge():
    r = run(5, None, [0, 50, 100, 150, 0.1])
    assert {'no_box', 'low_score', 'edge'} <= set(r['flags'])


def test_empty_wall():
    assert run(0, [1, 1, 50, 50, 0.1], [1, 1, 50, 50, 0.1]) == {
        'flags': [], 'proposal': [], 'agree_iou': None}
    assert run(0, [1, 1, 50, 50, 0.9], None)['flags'] == ['sign_on_empty']


def test_low_score_is_relative_to_the_class():
    auto = {f'i{k}': {'cls': 3, 'size': [W, H], 'preds': {
        'owlv2': [[100, 100, 200, 200, s]], 'gdino': [[100, 100, 200, 200, 0.5]]}}
        for k, s in enumerate([0.08, 0.09, 0.07, 0.02])}
    al.reanalyse(auto)
    assert [r['flags'] for r in auto.values()] == [[], [], [], ['low_score']]
