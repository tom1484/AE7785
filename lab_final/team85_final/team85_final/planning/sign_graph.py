"""SignGraph: what we know about the sign-following graph.

Nodes are wall faces (CellPose facing a wall). Because the maze is known, the
sign on a face fully determines its out-edge: turn as the sign says, then drive
to the next wall (MazeGrid.drive_to_wall). So the graph is the grid plus a
belief about which sign is on each face; edges are derived, never stored.

Evidence accumulates per face as confidence-weighted votes. Inspections weigh
more than passive sightings (`passive_weight`); sightings below `min_confidence`
are dropped. A face's sign is "known" once the leading class has at least
`min_score` and leads the runner-up by `margin`. `add` reports when that changes,
which is when the face's out-edge appears, changes or disappears.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field

from ..maze.geometry import CellPose
from ..maze.grid import MazeGrid
from ..vision.signs import SignClass
from .base import SignObservation, Source


@dataclass
class FaceBelief:
    votes: dict[SignClass, float] = field(default_factory=lambda: defaultdict(float))
    n_obs: int = 0

    def ranked(self) -> list[tuple[SignClass, float]]:
        return sorted(self.votes.items(), key=lambda kv: kv[1], reverse=True)


@dataclass(frozen=True)
class Chain:
    """Result of following known edges from a face."""

    faces: list[CellPose]        # visited faces, starting face first
    end: str                     # 'goal' | 'unknown' | 'cycle'

    @property
    def last(self) -> CellPose:
        return self.faces[-1]


class SignGraph:
    def __init__(self, grid: MazeGrid, passive_weight: float = 0.3,
                 min_score: float = 0.8, margin: float = 0.5, min_confidence: float = 0.7):
        self.grid = grid
        self.min_confidence = min_confidence
        self.passive_weight = passive_weight
        self.min_score = min_score
        self.margin = margin
        self._beliefs: dict[CellPose, FaceBelief] = {}
        self._faces = set(grid.wall_faces())

    # ---- evidence -----------------------------------------------------
    def add(self, obs: SignObservation) -> bool:
        """Add evidence. True if the face's believed sign (and so its edge) changed."""
        if obs.face not in self._faces:
            return False  # not a wall face; a mislocalized sighting
        if obs.source is Source.PASSIVE and obs.confidence < self.min_confidence:
            return False  # inspections always count: EMPTY comes with its own confidence
        before = self.sign(obs.face)
        w = obs.confidence * (1.0 if obs.source is Source.INSPECT else self.passive_weight)
        b = self._beliefs.setdefault(obs.face, FaceBelief())
        b.votes[obs.sign] += w
        b.n_obs += 1
        return self.sign(obs.face) != before

    def sign(self, face: CellPose) -> SignClass | None:
        """Believed sign on `face`, or None if unknown / ambiguous."""
        b = self._beliefs.get(face)
        if not b:
            return None
        ranked = b.ranked()
        top, score = ranked[0]
        second = ranked[1][1] if len(ranked) > 1 else 0.0
        if score >= self.min_score and score - second >= self.margin:
            return top
        return None

    def belief(self, face: CellPose) -> FaceBelief | None:
        return self._beliefs.get(face)

    # ---- structure ----------------------------------------------------
    def faces(self) -> set[CellPose]:
        return set(self._faces)

    def successor(self, face: CellPose) -> CellPose | None:
        """Next face when obeying the known sign on `face`; None if not a known turn."""
        s = self.sign(face)
        if s is None or not s.is_turn:
            return None
        return self.grid.drive_to_wall(face.turned(s.quarter_turns))

    def edges(self):
        """Known edges (face, next_face)."""
        for f in self._beliefs:
            nxt = self.successor(f)
            if nxt is not None:
                yield f, nxt

    def predecessors(self, face: CellPose) -> list[CellPose]:
        return [a for a, b in self.edges() if b == face]

    def follow(self, face: CellPose) -> Chain:
        """Follow known edges from `face` until goal, an unknown sign, or a cycle."""
        seen = [face]
        while True:
            cur = seen[-1]
            if self.sign(cur) is SignClass.GOAL:
                return Chain(seen, 'goal')
            nxt = self.successor(cur)
            if nxt is None:
                return Chain(seen, 'unknown')
            if nxt in seen:
                return Chain(seen + [nxt], 'cycle')
            seen.append(nxt)

    def goal_faces(self) -> list[CellPose]:
        return [f for f in self._beliefs if self.sign(f) is SignClass.GOAL]

    def to_dict(self) -> dict:
        """Plain-data snapshot (yaml/json): every face with evidence, its belief and edge."""
        out = []
        for f in sorted(self._beliefs, key=lambda f: (f.cell.row, f.cell.col, f.heading)):
            b, sign, nxt = self._beliefs[f], self.sign(f), self.successor(f)
            out.append({
                'face': [f.cell.row, f.cell.col, f.heading.name[0]],
                'sign': sign.name.lower() if sign is not None else None,
                'votes': {k.name.lower(): round(v, 3) for k, v in b.ranked()},
                'n_obs': b.n_obs,
                'next': [nxt.cell.row, nxt.cell.col, nxt.heading.name[0]] if nxt else None,
            })
        return {'faces': out}

    def frontier(self) -> set[CellPose]:
        """Faces we would arrive at by following known edges whose sign is still unknown."""
        return {b for _, b in self.edges() if self.sign(b) is None}
