"""Discrete maze geometry: cells, headings, and robot poses on the grid.

Conventions (shared by every module; do not change one place only):
    - Cell(row, col): col grows along the maze +x axis, row along the maze +y axis.
      Cell (0, 0) is the cell at the maze origin corner (see MazeGrid.origin).
    - Heading is the yaw quadrant in the maze frame: EAST=+x, NORTH=+y, WEST, SOUTH,
      so heading.value * pi/2 is the yaw relative to the maze frame.
    - Left turn = +1, right turn = -1, turn around = +2 (mod 4), like ROS yaw.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum


class Heading(IntEnum):
    EAST = 0
    NORTH = 1
    WEST = 2
    SOUTH = 3

    def turned(self, quarter_turns: int) -> Heading:
        """Heading after quarter_turns counter-clockwise turns (negative = right)."""
        return Heading((self.value + quarter_turns) % 4)

    @property
    def left(self) -> Heading:
        return self.turned(1)

    @property
    def right(self) -> Heading:
        return self.turned(-1)

    @property
    def back(self) -> Heading:
        return self.turned(2)

    @property
    def delta(self) -> tuple[int, int]:
        """(d_row, d_col) of one step in this heading."""
        return _DELTA[self]

    @property
    def yaw(self) -> float:
        """Yaw in the maze frame (rad)."""
        return self.value * math.pi / 2

    @staticmethod
    def from_yaw(yaw: float) -> Heading:
        """Nearest heading to a yaw in the maze frame (rad)."""
        return Heading(round(yaw / (math.pi / 2)) % 4)

    @staticmethod
    def parse(text: str) -> Heading:
        """'E'/'east'/'EAST' -> Heading.EAST, etc."""
        t = text.strip().upper()
        for h in Heading:
            if h.name == t or h.name[0] == t:
                return h
        raise ValueError(f'not a heading: {text!r}')


_DELTA = {
    Heading.EAST: (0, 1),
    Heading.NORTH: (1, 0),
    Heading.WEST: (0, -1),
    Heading.SOUTH: (-1, 0),
}


@dataclass(frozen=True, order=True)
class Cell:
    row: int
    col: int

    def step(self, heading: Heading, n: int = 1) -> Cell:
        dr, dc = heading.delta
        return Cell(self.row + n * dr, self.col + n * dc)

    def manhattan(self, other: Cell) -> int:
        return abs(self.row - other.row) + abs(self.col - other.col)


@dataclass(frozen=True, order=True)
class CellPose:
    """Robot standing in `cell` (at its center) facing `heading`.

    When a wall is in front, the pose also names that wall face: the side
    `heading` of `cell`. A sign on that face is what the robot reads there,
    so planners use CellPose as the key for wall faces / sign locations.
    """

    cell: Cell
    heading: Heading

    def turned(self, quarter_turns: int) -> CellPose:
        return CellPose(self.cell, self.heading.turned(quarter_turns))

    def __str__(self) -> str:
        return f'({self.cell.row},{self.cell.col}){self.heading.name[0]}'
