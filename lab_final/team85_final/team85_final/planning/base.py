"""Planner interface. Every planner is swappable through this contract.

The executive (nodes/mission_node.py) owns the robot: it localizes, drives with
Nav2, and reads signs. A planner only decides. The loop is:

    planner.reset(start_pose)
    loop:
        action = planner.next_action(current_pose)
        GoTo    -> executive drives to action.target (Nav2), then asks again
        Inspect -> executive reads the sign on the wall in front (stationary),
                   calls planner.observe(SignObservation(..., source=INSPECT)), asks again
        Finish  -> executive stops and signals the goal (sound) -- avoids the c penalty
        GiveUp  -> executive stops and logs the reason
    meanwhile: every sign the detector sees while moving is localized and passed
    to planner.observe(..., source=PASSIVE). Planners may ignore those.

Planners are pure Python (no rclpy) so they run in unit tests and offline sims.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Union

from ..maze.geometry import CellPose
from ..maze.grid import MazeGrid
from ..vision.signs import SignClass


class Source(Enum):
    INSPECT = 'inspect'   # robot stopped facing this wall and read it (high trust)
    PASSIVE = 'passive'   # seen in passing, placed via bearing + LIDAR (lower trust)


@dataclass(frozen=True)
class SignObservation:
    face: CellPose        # wall face the sign is on (robot at face.cell looking face.heading)
    sign: SignClass       # EMPTY = looked and saw no sign (INSPECT only)
    confidence: float
    source: Source = Source.INSPECT
    stamp: float = 0.0    # s


@dataclass(frozen=True)
class GoTo:
    target: CellPose      # drive to target.cell center and face target.heading


@dataclass(frozen=True)
class Inspect:
    pass


@dataclass(frozen=True)
class Finish:
    pass


@dataclass(frozen=True)
class GiveUp:
    reason: str


Action = Union[GoTo, Inspect, Finish, GiveUp]


class Planner(ABC):
    """Base class. Subclasses register with @register('name') in planning/registry.py."""

    name = 'base'

    def __init__(self, grid: MazeGrid, **params):
        self.grid = grid
        self.params = params

    def reset(self, start: CellPose) -> None:
        """New run from `start`. Clear all run state."""

    def observe(self, obs: SignObservation) -> bool | None:
        """New sign evidence (from an inspection or seen in passing).

        Graph-building planners return True when it changed a believed sign (an edge),
        so the executive can log it."""

    @abstractmethod
    def next_action(self, pose: CellPose) -> Action:
        """Decide what to do from the robot's current (snapped) pose."""
