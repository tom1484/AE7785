"""Planner 3.1: follow the signs literally, one wall at a time.

Drive straight to the wall in front, stop, read the sign, turn as it says, repeat.
Only signs read by inspection are used. With a correct reading at every wall the
path taken equals the optimal sign path, so r == n in the grading formula.

No sign in front (EMPTY): look left, right, then behind, in place (the lab's
"turn in place to check other orientations"). A sign found that way is obeyed
relative to the heading the robot is now facing.
"""

from __future__ import annotations

from ..maze.geometry import CellPose
from ..vision.signs import SignClass
from .base import Action, Finish, GiveUp, GoTo, Inspect, Planner, SignObservation, Source
from .registry import register

# Order of headings to try, relative to the original one, when no sign is visible.
_SEARCH = (1, -1, 2)


@register('grid')
class GridPlanner(Planner):
    def reset(self, start: CellPose) -> None:
        self._reading: tuple[CellPose, SignClass] | None = None
        self._search_origin: CellPose | None = None   # pose where an EMPTY started a search
        self._search_step = 0

    def observe(self, obs: SignObservation) -> None:
        if obs.source is Source.INSPECT:
            self._reading = (obs.face, obs.sign)

    def next_action(self, pose: CellPose) -> Action:
        if not self.grid.facing_wall(pose):
            # Start pose or a search turn facing an opening: go to the next wall.
            return GoTo(self.grid.drive_to_wall(pose))

        if self._reading is None or self._reading[0] != pose:
            return Inspect()
        sign = self._reading[1]

        if sign is SignClass.GOAL:
            return Finish()
        if sign.is_turn:
            self._search_origin, self._search_step = None, 0
            turned = pose.turned(sign.quarter_turns)
            return GoTo(self.grid.drive_to_wall(turned))

        # EMPTY: rotate in place to the next wall-facing heading and inspect there.
        # Signs only hang on walls, so headings facing an opening are skipped (and
        # rotating to one would make the next call drive away).
        if self._search_origin is None or self._search_origin.cell != pose.cell:
            self._search_origin, self._search_step = pose, 0
        candidates = [self._search_origin.turned(k) for k in _SEARCH]
        candidates = [p for p in candidates if self.grid.facing_wall(p)]
        if self._search_step >= len(candidates):
            return GiveUp(f'no sign on any wall at {pose.cell}')
        nxt = candidates[self._search_step]
        self._search_step += 1
        return GoTo(nxt)
