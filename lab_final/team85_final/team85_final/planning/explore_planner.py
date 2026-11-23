"""Planner 'explore': random traversal that only builds the sign graph (a test tool).

Drives to `steps` random wall faces (cell center + heading, Nav2 picks the route),
never inspects and never follows signs. Every detection on the way arrives as a
PASSIVE observation and goes into a SignGraph, so a run shows how well the graph is
built from sightings alone. Compare the result with the Gazebo truth:

    sim/sign_truth.py --compare /tmp/graph.yaml     (mission param graph_dump)

Targets: uniformly random wall faces, preferring ones whose sign is still unknown
(`prefer_unknown`) and never the face the robot is at. Reproducible with `seed`.
"""

from __future__ import annotations

import random

from ..maze.geometry import CellPose
from .base import Action, GiveUp, GoTo, Planner, SignObservation
from .registry import register
from .sign_graph import SignGraph

GRAPH_PARAMS = ('passive_weight', 'min_score', 'margin', 'min_confidence')


@register('explore')
class ExplorePlanner(Planner):
    def __init__(self, grid, steps: int = 12, seed: int | None = None,
                 prefer_unknown: bool = True, **params):
        super().__init__(grid, **params)
        self.steps, self.seed, self.prefer_unknown = int(steps), seed, bool(prefer_unknown)
        self.graph_params = {k: params[k] for k in GRAPH_PARAMS if k in params}
        self.graph = SignGraph(grid, **self.graph_params)

    def reset(self, start: CellPose) -> None:
        self.graph = SignGraph(self.grid, **self.graph_params)
        self.rng = random.Random(self.seed)
        self.done = 0
        self.targets: list[CellPose] = []

    def observe(self, obs: SignObservation) -> bool:
        return self.graph.add(obs)

    def next_action(self, pose: CellPose) -> Action:
        if self.done >= self.steps:
            return GiveUp(f'explore: {self.steps} targets visited')
        faces = sorted(f for f in self.graph.faces() if f != pose)
        unknown = [f for f in faces if self.graph.sign(f) is None]
        pool = unknown if self.prefer_unknown and unknown else faces
        target = self.rng.choice(pool)
        self.done += 1
        self.targets.append(target)
        return GoTo(target)
