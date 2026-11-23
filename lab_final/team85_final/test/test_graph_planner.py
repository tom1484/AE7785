"""GraphPlanner behaviours (strategies 1-5) and a seeded Monte Carlo regression check."""

from pathlib import Path

from team85_final.maze import Cell, CellPose, Heading, MazeGrid
from team85_final.planning import Finish, GoTo, Inspect, SignObservation, Source
from team85_final.planning.graph_planner import GraphPlanner
from team85_final.planning.scenarios import ErrorModel, random_layout, random_start
from team85_final.planning.simulate import sign_path
from team85_final.vision.signs import SignClass as S

E, N, W, So = Heading.EAST, Heading.NORTH, Heading.WEST, Heading.SOUTH
GRID = MazeGrid.from_dict({'rows': 3, 'cols': 6, 'cell_size': 1.0, 'walls': []})
MAZE_SIM = Path(__file__).resolve().parents[1] / 'config' / 'maze_sim.yaml'


def P(r, c, h):
    return CellPose(Cell(r, c), h)


def passive(p, face, sign, frames=10):
    for _ in range(frames):
        p.observe(SignObservation(face, sign, 0.9, Source.PASSIVE))


def inspect(p, face, sign, conf=0.9):
    p.observe(SignObservation(face, sign, conf, Source.INSPECT))


def planner(**kw):
    p = GraphPlanner(GRID, **kw)
    p.reset(P(0, 0, E))
    return p


def test_inspects_then_obeys():
    p = planner(allow_shortcuts=False)
    assert isinstance(p.next_action(P(0, 5, E)), Inspect)
    inspect(p, P(0, 5, E), S.LEFT)
    assert p.next_action(P(0, 5, E)) == GoTo(P(2, 5, N))


def test_rereads_when_passive_disagrees_and_obeys_passive_after_miss():
    p = planner(allow_shortcuts=False)
    f = P(0, 5, E)
    passive(p, f, S.LEFT)
    p.next_action(f)
    inspect(p, f, S.EMPTY, 0.8)
    assert isinstance(p.next_action(f), Inspect)            # 1: disagrees with passive
    inspect(p, f, S.EMPTY, 0.8)
    inspect(p, f, S.EMPTY, 0.8)
    assert p.next_action(f) == GoTo(P(2, 5, N))             # miss: passive LEFT is obeyed


def test_goal_needs_inspection_unless_only_misses():
    p = planner()
    f = P(2, 5, E)
    passive(p, f, S.GOAL)
    assert isinstance(p.next_action(f), Inspect)            # passive goal alone: inspect
    inspect(p, f, S.GOAL)
    assert isinstance(p.next_action(f), Finish)
    q = planner(max_inspections=1)
    passive(q, f, S.GOAL)
    q.next_action(f)
    inspect(q, f, S.EMPTY)                                  # missed in the dark
    assert isinstance(q.next_action(f), Finish)


def test_goal_reading_overruled_by_firm_passive_stop():
    p = planner(max_inspections=1)
    f = P(1, 5, E)
    passive(p, f, S.STOP)
    p.next_action(f)
    inspect(p, f, S.GOAL)
    a = p.next_action(f)
    assert not isinstance(a, Finish) and a == GoTo(P(1, 0, W))


def test_shortcut_to_end_of_known_chain():
    p = planner(allow_shortcuts=True, max_skip=None)
    passive(p, P(2, 5, N), S.LEFT)                          # -> (2,0)W
    passive(p, P(2, 0, W), S.LEFT)                          # -> (0,0)S
    p.next_action(P(0, 5, E))
    inspect(p, P(0, 5, E), S.LEFT)                          # -> (2,5)N
    assert p.next_action(P(0, 5, E)) == GoTo(P(0, 0, So))   # skips (2,5)N and (2,0)W
    p2 = planner(allow_shortcuts=False)
    passive(p2, P(2, 5, N), S.LEFT)
    p2.next_action(P(0, 5, E))
    inspect(p2, P(0, 5, E), S.LEFT)
    assert p2.next_action(P(0, 5, E)) == GoTo(P(2, 5, N))


def test_recovers_when_graph_exposes_an_obeyed_misreading():
    p = planner(allow_shortcuts=False, max_inspections=1)
    f = P(0, 5, E)
    p.next_action(f)
    inspect(p, f, S.RIGHT, 0.7)                             # wrong: really LEFT
    assert p.next_action(f) == GoTo(P(0, 5, So))
    inspect(p, f, S.LEFT)                                   # later evidence: LEFT, twice
    inspect(p, f, S.LEFT)
    passive(p, f, S.LEFT)
    assert p.next_action(P(0, 5, So)) == GoTo(P(2, 5, N))   # back onto the true branch


def test_beats_grid_planner_on_seeded_monte_carlo():
    from team85_final.planning.montecarlo import evaluate
    grid = MazeGrid.from_yaml(MAZE_SIM)
    s = evaluate(grid, ['grid', 'graph', 'graph:allow_shortcuts=false'], n=60, seed=3,
                 errors='nominal')
    assert s['graph:allow_shortcuts=false']['mean'] >= s['grid']['mean']
    assert s['graph']['reached'] >= s['grid']['reached']


def test_random_layouts_always_lead_to_the_goal():
    import random
    grid = MazeGrid.from_yaml(MAZE_SIM)
    rng = random.Random(0)
    usable = 0
    for _ in range(40):
        signs = random_layout(grid, rng)
        assert sum(s is S.GOAL for s in signs.values()) == 1
        for face in signs:                                  # every signed face reaches the goal
            assert sign_path(grid, signs, face.turned(2).turned(2)) is not None
        usable += random_start(grid, signs, rng) is not None
    # Some goals sit in pockets few walls lead to (2-4 signs, no start with n >= 4).
    assert usable >= 20


def test_error_model_is_paired_per_face():
    a, b = ErrorModel.preset('harsh', seed=5), ErrorModel.preset('harsh', seed=5)
    faces = [P(r, c, h) for r in range(3) for c in range(6) for h in (E, N)]
    first_a = [a.reader(f, S.LEFT)[0] for f in faces]
    first_b = [b.reader(f, S.LEFT)[0] for f in reversed(faces)][::-1]
    assert first_a == first_b                               # same errors, any query order


# ---- sighting-only mode (inspect: false) ------------------------------------
def test_sighting_only_obeys_passive_without_inspecting():
    p = planner(inspect=False, allow_shortcuts=False)
    passive(p, P(0, 5, E), S.LEFT)
    assert p.next_action(P(0, 5, E)) == GoTo(P(2, 5, N))


def test_sighting_only_inspects_once_when_nothing_is_known():
    p = planner(inspect=False)
    f = P(0, 5, E)
    assert isinstance(p.next_action(f), Inspect)
    inspect(p, f, S.LEFT)
    assert p.next_action(f) == GoTo(P(2, 5, N))


def test_sighting_only_finishes_on_passive_goal():
    p = planner(inspect=False, confirm_goal=False)
    passive(p, P(2, 5, E), S.GOAL)
    assert isinstance(p.next_action(P(2, 5, E)), Finish)


def test_sighting_only_confirms_goal_up_close():
    p = planner(inspect=False)
    f = P(2, 5, E)
    passive(p, f, S.GOAL)
    assert isinstance(p.next_action(f), Inspect)
    inspect(p, f, S.STOP)                                # far STOP looked like GOAL
    assert p.next_action(f) == GoTo(P(2, 0, W))          # obey the close read


def test_sighting_only_shortcuts_any_known_chain_and_retargets_mid_drive():
    p = planner(inspect=False, max_skip=None)
    passive(p, P(0, 5, E), S.LEFT)                       # (0,5)E -> (2,5)N
    target = p.next_action(P(0, 5, E)).target
    assert target == P(2, 5, N)                          # (2,5)N unknown yet
    assert p.retarget(target) is None
    passive(p, P(2, 5, N), S.LEFT)                       # seen while driving: -> (2,0)W
    assert p.retarget(target) == P(2, 0, W)
    passive(p, P(2, 0, W), S.GOAL)
    assert p.retarget(P(2, 0, W)) is None                # goal is the end of the chain


def chain_planner(**kw):
    """(0,5)E LEFT -> (2,5)N LEFT -> (2,0)W RIGHT -> (2,0)N ... all known."""
    p = planner(inspect=False, **kw)
    passive(p, P(0, 5, E), S.LEFT)
    passive(p, P(2, 5, N), S.LEFT)
    passive(p, P(2, 0, W), S.RIGHT)
    passive(p, P(2, 0, N), S.GOAL)
    return p


def test_max_skip_limits_walls_skipped_per_shortcut():
    assert chain_planner(max_skip=None).next_action(P(0, 5, E)).target == P(2, 0, N)
    p = chain_planner(max_skip=1)
    assert p.next_action(P(0, 5, E)).target == P(2, 0, W)    # skips (2,5)N only
    assert chain_planner(max_skip=2).next_action(P(0, 5, E)).target == P(2, 0, N)


def test_retarget_only_within_one_cell_of_the_target():
    p = chain_planner(max_skip=1)
    target = P(2, 5, N)
    tx, ty, _ = GRID.pose_to_map(target)
    assert p.retarget(target, (tx, ty - 2.5)) is None           # 2.5 cells away: keep going
    assert p.retarget(target, (tx, ty - 0.8)) == P(2, 0, W)     # within 1 cell: skip one wall
    assert p.retarget(P(2, 0, W), (tx, ty - 0.8)) is None       # next one only when near it


def test_inspect_mode_max_skip_one_still_allows_single_skips():
    p = planner(max_skip=1)                                     # inspect mode, min_shortcut 2
    passive(p, P(2, 5, N), S.LEFT)
    passive(p, P(2, 0, W), S.GOAL)
    f = P(0, 5, E)
    p.next_action(f)
    inspect(p, f, S.LEFT)
    assert p.next_action(f) == GoTo(P(2, 0, W))                 # skipped (2,5)N


def test_retarget_is_off_in_inspect_mode():
    p = planner()
    passive(p, P(2, 5, N), S.LEFT)
    assert p.retarget(P(2, 5, N)) is None
