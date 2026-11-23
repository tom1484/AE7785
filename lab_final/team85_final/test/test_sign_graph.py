from team85_final.maze import Cell, CellPose, Heading, MazeGrid
from team85_final.planning import SignObservation, Source
from team85_final.planning.sign_graph import SignGraph
from team85_final.vision.signs import SignClass as S

E, N, W = Heading.EAST, Heading.NORTH, Heading.WEST
GRID = MazeGrid.from_dict({'rows': 3, 'cols': 6, 'cell_size': 1.0, 'walls': []})


def P(r, c, h):
    return CellPose(Cell(r, c), h)


def obs(face, sign, conf=1.0, src=Source.INSPECT):
    return SignObservation(face, sign, conf, src)


def test_chain_to_unknown_and_goal():
    g = SignGraph(GRID)
    g.add(obs(P(0, 5, E), S.LEFT))
    g.add(obs(P(2, 5, N), S.LEFT))
    ch = g.follow(P(0, 5, E))
    assert ch.end == 'unknown' and ch.faces == [P(0, 5, E), P(2, 5, N), P(2, 0, W)]
    assert g.frontier() == {P(2, 0, W)}
    g.add(obs(P(2, 0, W), S.GOAL))
    assert g.follow(P(0, 5, E)).end == 'goal'
    assert g.goal_faces() == [P(2, 0, W)]
    assert g.predecessors(P(2, 5, N)) == [P(0, 5, E)]


def test_cycle():
    g = SignGraph(GRID)
    g.add(obs(P(0, 5, E), S.STOP))     # turn around -> (0,0)W
    g.add(obs(P(0, 0, W), S.STOP))     # back -> (0,5)E
    assert g.follow(P(0, 5, E)).end == 'cycle'


def test_passive_needs_more_evidence():
    g = SignGraph(GRID, passive_weight=0.3, min_score=0.8)
    g.add(obs(P(0, 5, E), S.LEFT, 0.9, Source.PASSIVE))
    assert g.sign(P(0, 5, E)) is None
    for _ in range(3):
        g.add(obs(P(0, 5, E), S.LEFT, 0.9, Source.PASSIVE))
    assert g.sign(P(0, 5, E)) is S.LEFT


def test_conflict_is_unknown_and_non_faces_ignored():
    g = SignGraph(GRID, margin=0.5)
    g.add(obs(P(0, 5, E), S.LEFT, 0.9))
    g.add(obs(P(0, 5, E), S.RIGHT, 0.8))
    assert g.sign(P(0, 5, E)) is None
    g.add(obs(P(1, 1, E), S.LEFT))     # open side, not a wall face
    assert g.belief(P(1, 1, E)) is None


def test_passive_threshold_and_change_flag():
    g = SignGraph(GRID, passive_weight=0.3, min_score=0.8, min_confidence=0.7)
    f = P(0, 5, E)
    assert not g.add(obs(f, S.LEFT, 0.6, Source.PASSIVE))      # below threshold: dropped
    assert g.belief(f) is None
    assert not g.add(obs(f, S.LEFT, 0.9, Source.PASSIVE))      # 0.27: not known yet
    assert not g.add(obs(f, S.LEFT, 0.9, Source.PASSIVE))      # 0.54
    assert g.add(obs(f, S.LEFT, 0.9, Source.PASSIVE))          # 0.81: known -> edge appears
    assert g.successor(f) == P(2, 5, N)
    assert not g.add(obs(f, S.LEFT, 0.9, Source.PASSIVE))      # same belief: no change
    d = g.to_dict()['faces'][0]
    assert d['face'] == [0, 5, 'E'] and d['sign'] == 'left' and d['next'] == [2, 5, 'N']
