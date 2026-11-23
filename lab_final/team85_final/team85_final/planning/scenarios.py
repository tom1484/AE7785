"""Random sign layouts and a sensing-error model, for Monte Carlo planner comparisons.

Layouts: the handout guarantees that the signs lead from any start to the goal. A layout
is built backwards from a random goal face: repeatedly give an unsigned wall face a sign
whose successor (turn as it says, drive to the next wall) is already solved. That is a
tree rooted at the goal, so every signed face leads there. Faces that can't be solved
stay blank; `blank` removes extra signs to stress-test getting lost.

Errors (`ErrorModel`), calibrated loosely on the detector's held-out results
(lab_final_vision: 0 left/right swaps, far stop<->goal confusion, some misses):
  - Inspection: per run, each face independently gets a systematic misreading with the
    probabilities below (bad light, odd angle). Re-reading the same face repeats that
    error with probability `persist`, so re-inspection helps but is not free accuracy.
  - Passive sightings: the same kinds of errors, `passive_scale` times likelier (far,
    oblique, moving), drawn per view and repeated over `passive_frames` frames.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

from ..maze.geometry import Cell, CellPose, Heading
from ..maze.grid import MazeGrid
from ..vision.signs import SignClass as S

TURNS = (S.LEFT, S.RIGHT, S.STOP)


def successor(grid: MazeGrid, face: CellPose, sign: S) -> CellPose:
    return grid.drive_to_wall(face.turned(sign.quarter_turns))


def random_layout(grid: MazeGrid, rng: random.Random, goal: CellPose | None = None,
                  blank: float = 0.0) -> dict[CellPose, S]:
    faces = sorted(grid.wall_faces(), key=lambda f: (f.cell.row, f.cell.col, f.heading))
    goal = goal or rng.choice(faces)
    signs = {goal: S.GOAL}
    changed = True
    while changed:
        changed = False
        todo = [f for f in faces if f not in signs]
        rng.shuffle(todo)
        for f in todo:
            options = [s for s in TURNS if successor(grid, f, s) in signs]
            if options:
                s = rng.choice(options)
                if s is S.STOP:
                    s = rng.choice((S.STOP, S.DO_NOT_ENTER))
                signs[f] = s
                changed = True
    for f in list(signs):
        if f != goal and rng.random() < blank:
            del signs[f]
    return signs


def random_start(grid: MazeGrid, signs: dict[CellPose, S], rng: random.Random,
                 min_length: int = 4, tries: int = 500) -> CellPose | None:
    """A start pose (any cell and heading) whose sign path reaches the goal in >= min_length."""
    from .simulate import sign_path
    cells = list(grid.cells())
    for _ in range(tries):
        start = CellPose(rng.choice(cells), rng.choice(list(Heading)))
        p = sign_path(grid, signs, start)
        if p is not None and p[1] >= min_length:
            return start
    return None


@dataclass
class ErrorModel:
    p_miss: float = 0.03          # a sign read as EMPTY
    p_phantom: float = 0.01       # EMPTY read as some sign
    p_lr: float = 0.01            # left <-> right
    p_stop_goal: float = 0.02     # stop / do-not-enter read as GOAL
    p_goal_stop: float = 0.04     # GOAL read as STOP
    persist: float = 0.7          # a re-read repeats the face's systematic error
    passive_scale: float = 3.0
    passive_frames: int = 4
    seed: int = 0
    _face_error: dict = field(default_factory=dict, repr=False)

    @classmethod
    def preset(cls, name: str, seed: int = 0) -> ErrorModel:
        if name == 'none':
            return cls(0, 0, 0, 0, 0, 0, 0, seed=seed)
        if name == 'nominal':
            return cls(seed=seed)
        if name == 'harsh':
            return cls(0.08, 0.03, 0.03, 0.05, 0.10, 0.8, 3.0, seed=seed)
        raise ValueError(name)

    def __post_init__(self):
        self.rng = random.Random(self.seed)

    def _corrupt(self, true: S, scale: float, rng: random.Random | None = None) -> S | None:
        """A wrong reading for `true`, or None (read correctly)."""
        rng = rng or self.rng
        r = rng.random
        if true is S.EMPTY:
            return rng.choice(TURNS) if r() < self.p_phantom * scale else None
        if r() < self.p_miss * scale:
            return S.EMPTY
        if true in (S.LEFT, S.RIGHT) and r() < self.p_lr * scale:
            return S.RIGHT if true is S.LEFT else S.LEFT
        if true in (S.STOP, S.DO_NOT_ENTER) and r() < self.p_stop_goal * scale:
            return S.GOAL
        if true is S.GOAL and r() < self.p_goal_stop * scale:
            return S.STOP
        return None

    def reader(self, face: CellPose, true: S) -> tuple[S, float]:
        """Inspection reading (sign, vote confidence) for the sign `true` on `face`."""
        if face not in self._face_error:
            # Seeded per face: every planner meets the same systematic errors in a trial.
            frng = random.Random(hash((self.seed, face.cell.row, face.cell.col, int(face.heading))))
            self._face_error[face] = self._corrupt(true, 1.0, frng)
            first = True
        else:
            first = False
        err = self._face_error[face]
        if err is not None and (first or self.rng.random() < self.persist):
            return err, self.rng.uniform(0.4, 0.9)          # wrong readings are less sure
        return true, self.rng.uniform(0.7, 1.0)

    def passive(self, seen: list[tuple[CellPose, S]]) -> list[tuple[CellPose, S, float]]:
        """Frames of passive sightings for one view (true (face, sign) pairs in view)."""
        out = []
        for face, true in seen:
            s = self._corrupt(true, self.passive_scale) or true
            if s is S.EMPTY:
                continue                                    # a miss is just no detection
            out += [(face, s, self.rng.uniform(0.7, 1.0))] * self.passive_frames
        return out


def all_cells_reachable(grid: MazeGrid) -> bool:
    cells = list(grid.cells())
    return all(grid.path_length(cells[0], c) is not None for c in cells)


__all__ = ['ErrorModel', 'random_layout', 'random_start', 'successor', 'all_cells_reachable',
           'Cell']
