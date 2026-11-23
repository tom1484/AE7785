"""Offline simulation of a planner on a known sign layout, scored like the lab.

The world is a MazeGrid plus {wall face: SignClass}. Inspect reads the sign on the
face in front (EMPTY if none), optionally through a noisy `reader`. Passive
sightings can be fed too (`passive`: faces visible from a pose -> observed).

Grading (handout 3.1), Manhattan distances in cells:
    reached: 1 - |r - n| * 0.01 - 0.1 * c
    else:    (1 - d / n) - |r - m| * 0.01 - 0.1 * c
n = optimal sign-path length from the start, r = length driven.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

from ..maze.geometry import CellPose, Heading
from ..maze.grid import MazeGrid
from ..vision.signs import SignClass
from .base import Finish, GiveUp, GoTo, Inspect, Planner, SignObservation, Source

Reader = Callable[[CellPose, SignClass], tuple[SignClass, float]]
Passive = Callable[[CellPose], list]   # pose -> [(face, sign) | (face, sign, conf)]


@dataclass
class RunResult:
    reached: bool
    distance: int                          # r, cells driven
    optimal: int | None                    # n, None if the signs never reach the goal
    poses: list[CellPose] = field(default_factory=list)
    actions: list = field(default_factory=list)
    outcome: str = ''
    closest: int | None = None             # d: closest Manhattan distance to the goal cell
    goal_cell_visited: bool = False
    inspections: int = 0

    @property
    def c(self) -> int:
        """1 if the TA would stop the run with the robot in the goal cell, never signalling."""
        return int(not self.reached and self.goal_cell_visited)

    def grade(self) -> float | None:
        """Handout 3.1. Not reached: d = closest the robot got to the goal (Manhattan),
        m = n - d (matches the handout's example: d=3, m=6, n=9)."""
        n = self.optimal
        if n is None:
            return None
        if self.reached:
            return 1 - abs(self.distance - n) * 0.01
        d = min(self.closest if self.closest is not None else n, n)
        m = n - d
        return (1 - d / n) - abs(self.distance - m) * 0.01 - 0.1 * self.c


def sign_path(grid: MazeGrid, signs: dict[CellPose, SignClass], start: CellPose,
              max_steps: int = 200) -> tuple[list[CellPose], int] | None:
    """The intended path from `start` following signs exactly; (wall faces, length) or None."""
    pose, length, faces = start, 0, []
    for _ in range(max_steps):
        nxt = grid.drive_to_wall(pose)
        length += pose.cell.manhattan(nxt.cell)
        pose = nxt
        faces.append(pose)
        s = signs.get(pose, SignClass.EMPTY)
        if s is SignClass.GOAL:
            return faces, length
        if not s.is_turn:
            return None
        pose = pose.turned(s.quarter_turns)
    return None


def fan_sightings(grid: MazeGrid, signs: dict[CellPose, SignClass], hfov: float = 1.085,
                  max_range: float = 2.5, rays: int = 9) -> Passive:
    """A `passive` model: the signs a camera at the cell center sees across its field of view
    (rays through the known maze, like the mission node's placement, no noise)."""
    from ..maze.sign_locator import raycast

    def see(pose: CellPose):
        x, y, yaw = grid.pose_to_map(pose)
        seen = {}
        for k in range(rays):
            b = -hfov / 2 + hfov * k / (rays - 1)
            hit = raycast(grid, x, y, yaw + b, max_range)
            if hit is not None and hit.face in signs:
                seen[hit.face] = signs[hit.face]
        return list(seen.items())
    return see


def run(planner: Planner, grid: MazeGrid, signs: dict[CellPose, SignClass], start: CellPose,
        reader: Reader | None = None, passive: Passive | None = None,
        max_actions: int = 300) -> RunResult:
    """Run `planner` from `start`. `reader(face, true_sign)` -> (sign, conf) for inspections;
    `passive(pose)` -> [(face, sign) or (face, sign, conf)] seen from pose, called at
    every cell the robot drives through (facing its direction of travel) and at stops."""
    ideal = sign_path(grid, signs, start)
    res = RunResult(False, 0, ideal[1] if ideal else None, poses=[start])
    goal_cell = next((f.cell for f, s in signs.items() if s is SignClass.GOAL), None)

    def visit(pose: CellPose):
        if goal_cell is not None:
            d = pose.cell.manhattan(goal_cell)
            res.closest = d if res.closest is None else min(res.closest, d)
            res.goal_cell_visited |= d == 0
        if passive:
            for item in passive(pose):
                face, s, conf = item if len(item) == 3 else (*item, 1.0)
                planner.observe(SignObservation(face, s, conf, Source.PASSIVE))

    planner.reset(start)
    pose = start
    visit(pose)
    for _ in range(max_actions):
        a = planner.next_action(pose)
        res.actions.append(a)
        if isinstance(a, GoTo):
            # Nav2 drives a shortest route around walls (cells counted like the graders).
            route = grid.route(pose.cell, a.target.cell)
            if route is None:
                res.outcome = f'unreachable target {a.target}'
                return res
            res.distance += len(route) - 1
            for k in range(1, len(route)):
                h = next(h for h in Heading if route[k - 1].step(h) == route[k])
                visit(CellPose(route[k], h))
            pose = a.target
            visit(pose)
            res.poses.append(pose)
        elif isinstance(a, Inspect):
            res.inspections += 1
            true = signs.get(pose, SignClass.EMPTY)
            s, conf = reader(pose, true) if reader else (true, 1.0)
            planner.observe(SignObservation(pose, s, conf, Source.INSPECT))
        elif isinstance(a, Finish):
            res.reached = signs.get(pose) is SignClass.GOAL
            res.outcome = 'finish' if res.reached else 'finish at wrong wall'
            return res
        elif isinstance(a, GiveUp):
            res.outcome = f'give up: {a.reason}'
            return res
    res.outcome = 'action limit'
    return res
