from team85_final.maze import Cell, CellPose, Heading, MazeGrid
from team85_final.planning import GoTo, Inspect, SignObservation, Source, available, make_planner
from team85_final.planning.simulate import run, sign_path
from team85_final.vision.signs import SignClass as S

E, N, W, SO = Heading.EAST, Heading.NORTH, Heading.WEST, Heading.SOUTH


def P(r, c, h):
    return CellPose(Cell(r, c), h)


GRID = MazeGrid.from_dict({'rows': 3, 'cols': 6, 'cell_size': 1.0, 'walls': [[1, 2, 'E']]})
# Start (0,0) E -> (0,5)E LEFT -> (2,5)N LEFT -> (2,0)W RIGHT -> (2,0)N GOAL
SIGNS = {P(0, 5, E): S.LEFT, P(2, 5, N): S.LEFT, P(2, 0, W): S.RIGHT, P(2, 0, N): S.GOAL}
START = P(0, 0, E)


def test_registry():
    assert {'grid', 'graph'} <= set(available())


def test_ideal_path_length():
    faces, n = sign_path(GRID, SIGNS, START)
    assert faces[-1] == P(2, 0, N) and n == 5 + 2 + 5


def test_grid_planner_follows_signs_exactly():
    res = run(make_planner('grid', GRID), GRID, SIGNS, START)
    assert res.reached, res.outcome
    assert res.distance == res.optimal == 12
    assert res.grade() == 1.0


def test_grid_planner_searches_when_no_sign_in_front():
    signs = {P(0, 0, SO): S.EMPTY, P(0, 0, W): S.GOAL}
    res = run(make_planner('grid', GRID), GRID, signs, P(0, 0, SO))
    assert res.reached and res.distance == 0


def test_grid_planner_gives_up_with_no_signs():
    res = run(make_planner('grid', GRID), GRID, {}, START)
    assert not res.reached and res.outcome.startswith('give up')


def test_graph_planner_without_shortcuts_matches_grid():
    res = run(make_planner('graph', GRID, allow_shortcuts=False), GRID, SIGNS, START)
    assert res.reached and res.distance == 12


def test_graph_planner_skips_known_chain():
    # Signs loop the long way round: (0,5)E -> (2,5)N -> (2,0)W -> goal at (0,0)S, n = 14.
    signs = {P(0, 5, E): S.LEFT, P(2, 5, N): S.LEFT, P(2, 0, W): S.LEFT, P(0, 0, SO): S.GOAL}
    # Everything seen in passing beforehand: from (0,5)E it jumps to the goal wall.
    res = run(make_planner('graph', GRID, passive_weight=1.0, max_skip=None), GRID, signs, START,
              passive=lambda pose: list(signs.items()))
    assert res.reached and res.optimal == 14
    assert res.distance == 10                  # 5 to the first wall + 5 straight back
    assert res.grade() == 0.96                 # the |r - n| trap: shortcuts are penalized too
    assert GoTo(P(0, 0, SO)) in res.actions


def test_grid_planner_inspects_on_arrival():
    pl = make_planner('grid', GRID)
    pl.reset(START)
    assert pl.next_action(START) == GoTo(P(0, 5, E))
    assert pl.next_action(P(0, 5, E)) == Inspect()
    pl.observe(SignObservation(P(0, 5, E), S.LEFT, 0.9, Source.INSPECT))
    assert pl.next_action(P(0, 5, E)) == GoTo(P(2, 5, N))


def test_explore_builds_correct_graph_from_passive_sightings():
    from team85_final.planning.explore_planner import ExplorePlanner
    from team85_final.planning.simulate import fan_sightings, run
    grid = MazeGrid.from_dict({'rows': 3, 'cols': 6, 'cell_size': 1.0,
                               'walls': [[0, 1, 'E'], [1, 3, 'N'], [2, 2, 'E']]})
    signs = {CellPose(Cell(0, 1), Heading.EAST): S.LEFT,
             CellPose(Cell(2, 1), Heading.NORTH): S.RIGHT,
             CellPose(Cell(2, 5), Heading.EAST): S.GOAL,
             CellPose(Cell(0, 5), Heading.SOUTH): S.STOP}
    p = ExplorePlanner(grid, steps=40, seed=1, min_score=0.3, margin=0.2)  # 1 sighting/stop here
    res = run(p, grid, signs, CellPose(Cell(0, 0), Heading.EAST),
              passive=fan_sightings(grid, signs))
    assert res.outcome.startswith('give up: explore')
    known = {f: p.graph.sign(f) for f in p.graph.faces() if p.graph.sign(f) is not None}
    assert known and all(signs[f] is s for f, s in known.items())   # nothing wrong
    assert len(known) >= 3
    for a, b in p.graph.edges():
        assert b == grid.drive_to_wall(a.turned(signs[a].quarter_turns))
