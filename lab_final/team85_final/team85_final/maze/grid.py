"""MazeGrid: the static maze as cells and walls, plus maze <-> map-frame conversion.

The maze is fixed for the demo (only signs move), so it is authored once in
config/maze.yaml (geometry measured from the SLAM map; walls extracted with
`extract_maze` and checked by hand).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .geometry import Cell, CellPose, Heading


@dataclass
class MazeGrid:
    rows: int
    cols: int
    cell_size: float                       # m
    origin: tuple[float, float] = (0.0, 0.0)  # map-frame xy of cell (0,0)'s outer corner
    yaw: float = 0.0                       # maze +x axis direction in the map frame (rad)
    # Interior walls, stored from both sides: (cell, heading) is blocked.
    _blocked: set[CellPose] = field(default_factory=set, repr=False)

    # ---- construction -------------------------------------------------
    def add_wall(self, cell: Cell, heading: Heading) -> None:
        """Wall on side `heading` of `cell` (also blocks the neighbor's opposite side)."""
        self._blocked.add(CellPose(cell, heading))
        nb = cell.step(heading)
        if self.contains(nb):
            self._blocked.add(CellPose(nb, heading.back))

    @classmethod
    def from_dict(cls, d: dict) -> MazeGrid:
        grid = cls(
            rows=int(d['rows']), cols=int(d['cols']), cell_size=float(d['cell_size']),
            origin=tuple(float(v) for v in d.get('origin', (0.0, 0.0))),
            yaw=float(d.get('yaw', 0.0)),
        )
        for r, c, h in d.get('walls', []) or []:
            grid.add_wall(Cell(int(r), int(c)), Heading.parse(str(h)))
        return grid

    @classmethod
    def from_yaml(cls, path: str | Path) -> MazeGrid:
        with open(path) as f:
            return cls.from_dict(yaml.safe_load(f))

    def to_dict(self) -> dict:
        """Interior walls listed once each (EAST/NORTH sides), boundary omitted."""
        walls = []
        for p in sorted(self._blocked):
            if p.heading in (Heading.EAST, Heading.NORTH) and self.contains(p.cell.step(p.heading)):
                walls.append([p.cell.row, p.cell.col, p.heading.name[0]])
        return {
            'rows': self.rows, 'cols': self.cols, 'cell_size': self.cell_size,
            'origin': list(self.origin), 'yaw': self.yaw, 'walls': walls,
        }

    # ---- topology -----------------------------------------------------
    def contains(self, cell: Cell) -> bool:
        return 0 <= cell.row < self.rows and 0 <= cell.col < self.cols

    def cells(self):
        for r in range(self.rows):
            for c in range(self.cols):
                yield Cell(r, c)

    def blocked(self, cell: Cell, heading: Heading) -> bool:
        """True if a wall (interior or boundary) is on side `heading` of `cell`."""
        return (not self.contains(cell.step(heading))) or CellPose(cell, heading) in self._blocked

    def facing_wall(self, pose: CellPose) -> bool:
        return self.blocked(pose.cell, pose.heading)

    def drive_to_wall(self, pose: CellPose) -> CellPose:
        """Where the robot ends up driving straight from `pose` until a wall is in front."""
        cell = pose.cell
        while not self.blocked(cell, pose.heading):
            cell = cell.step(pose.heading)
        return CellPose(cell, pose.heading)

    def path_length(self, a: Cell, b: Cell) -> int | None:
        """Fewest cell steps from a to b around walls (BFS); None if unreachable."""
        frontier, dist = [a], {a: 0}
        while frontier:
            nxt = []
            for cell in frontier:
                if cell == b:
                    return dist[cell]
                for h in Heading:
                    nb = cell.step(h)
                    if not self.blocked(cell, h) and nb not in dist:
                        dist[nb] = dist[cell] + 1
                        nxt.append(nb)
            frontier = nxt
        return None

    def route(self, a: Cell, b: Cell) -> list[Cell] | None:
        """A shortest cell route a -> b around walls (BFS, both ends included); None if none."""
        prev, frontier = {a: None}, [a]
        while frontier:
            nxt = []
            for cell in frontier:
                if cell == b:
                    out = [cell]
                    while prev[out[-1]] is not None:
                        out.append(prev[out[-1]])
                    return out[::-1]
                for h in Heading:
                    nb = cell.step(h)
                    if not self.blocked(cell, h) and nb not in prev:
                        prev[nb] = cell
                        nxt.append(nb)
            frontier = nxt
        return None

    def wall_faces(self):
        """Every CellPose that faces a wall: the possible sign locations / graph nodes."""
        for cell in self.cells():
            for h in Heading:
                if self.blocked(cell, h):
                    yield CellPose(cell, h)

    # ---- maze frame <-> map frame -------------------------------------
    def cell_center(self, cell: Cell) -> tuple[float, float]:
        """Map-frame xy of a cell center."""
        return self._to_map((cell.col + 0.5) * self.cell_size, (cell.row + 0.5) * self.cell_size)

    def heading_yaw(self, heading: Heading) -> float:
        """Map-frame yaw of a heading (rad)."""
        return _wrap(self.yaw + heading.yaw)

    def pose_to_map(self, pose: CellPose) -> tuple[float, float, float]:
        x, y = self.cell_center(pose.cell)
        return x, y, self.heading_yaw(pose.heading)

    def map_to_maze(self, x: float, y: float) -> tuple[float, float]:
        """Map-frame xy -> maze-frame xy (m, from the maze origin corner)."""
        dx, dy = x - self.origin[0], y - self.origin[1]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def locate(self, x: float, y: float, yaw: float) -> CellPose | None:
        """Snap a map-frame pose to the grid; None if outside the maze."""
        mx, my = self.map_to_maze(x, y)
        cell = Cell(math.floor(my / self.cell_size), math.floor(mx / self.cell_size))
        if not self.contains(cell):
            return None
        return CellPose(cell, Heading.from_yaw(yaw - self.yaw))

    def _to_map(self, mx: float, my: float) -> tuple[float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.origin[0] + c * mx - s * my, self.origin[1] + s * mx + c * my

    # ---- debugging ----------------------------------------------------
    def render(self, marks: dict[Cell, str] | None = None) -> str:
        """ASCII drawing, north (high row) at the top. `marks` puts a char in cells."""
        marks = marks or {}
        lines = []
        for r in reversed(range(self.rows)):
            top = '+'
            mid = ''
            for c in range(self.cols):
                cell = Cell(r, c)
                top += ('---' if self.blocked(cell, Heading.NORTH) else '   ') + '+'
                mid += '|' if self.blocked(cell, Heading.WEST) else ' '
                mid += f' {marks.get(cell, " ")} '
            mid += '|' if self.blocked(Cell(r, self.cols - 1), Heading.EAST) else ' '
            lines += [top, mid]
        lines.append('+' + '---+' * self.cols)
        return '\n'.join(lines)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))
