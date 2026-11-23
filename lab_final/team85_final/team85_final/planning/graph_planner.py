"""Planner 3.2: follow the signs, and use every sign seen so far to avoid and fix mistakes.

The grade has no time term and penalizes |r - n| both ways, so the sign graph is used for
reliability (getting lost or misreading costs far more than a detour). Every rule below
was kept only if it improved the Monte Carlo comparison (planning/montecarlo.py; results
in the workspace CLAUDE.md).

1. Verify readings. Every wall is inspected; a reading that is weak or disagrees with the
   passive sightings of that face is re-read (up to `max_inspections` per visit). The
   decision fuses inspections with passive evidence capped at `passive_cap` (= one
   inspection), so a sign misread repeatedly from afar can't outvote close-up readings.
   A sign read as EMPTY that passive sightings firmly know is obeyed (a miss in a dark
   corner is far likelier than a phantom seen from many places).
2. Look ahead. If the sign about to be obeyed leads, through known signs, into a cycle that
   never reaches the goal, some belief is wrong: re-read this face before moving.
3. Goal. Finish on an inspected GOAL, except when passive firmly knows another sign here
   and (`distrust_goal`) - stop read as goal is the detector's weak spot. Also finish when
   inspections only ever saw EMPTY and passive firmly knows GOAL, or when the signs lead
   back to a face where GOAL was seen (signs never loop, so that face is the goal).
4. Lost (no sign in front). Turn straight to a wall of this cell whose known chain reaches
   the goal; else check the cell's other walls (like the grid planner); else drive to the
   nearest wall whose known chain reaches the goal; else to the nearest unexplored wall.
   Back at a face already obeyed: re-read it, then try its next-best sign. Give up after
   `max_visits` visits to one face (wandering costs 1 per cell).
5. Recover. If the graph later shows that a sign we obeyed was misread, drive to where the
   correct sign leads instead of continuing on the wrong branch.

Shortcuts (`allow_shortcuts`, on by the team's choice - for fun, not score, and the TA is
told): when the signs after the next wall are already known, drive straight to the end of
that known chain and inspect there. Skipped cells make r < n (-1 per cell).

Sighting-only mode (`inspect: false`, for show - fast and smooth, not graded-safe): never
stop to read a wall. The sign in front is what passive sightings firmly know (`passive_sign`);
only if nothing is known there does it fall back to one Inspect. `confirm_goal` (default on)
reads a sighted GOAL once up close before finishing: far stop vs goal is the detector's weak
spot (Monte Carlo: most of this mode's lost points were wrong-wall finishes). Shortcuts then take any
known chain (no `min_shortcut`), and `retarget` lets the executive move the current Nav2
goal mid-drive when new sightings extend the known chain ahead of it - one wall at a time
(`max_skip`, default 1) and only within `retarget_within` cells (default 1) of the current
target, so the robot rolls through the sign route instead of re-planning a different one. Less reliable: one
wrong far sighting is obeyed without a close-up check (recovery and the visit cap remain).

Rejected after measuring: finishing at a wall the signs lead to twice that still reads
EMPTY (turned give-ups into wrong-wall finishes, no better on average).
"""

from __future__ import annotations

from collections import defaultdict

from ..maze.geometry import CellPose, Heading
from ..vision.signs import SignClass as S
from .base import Action, Finish, GiveUp, GoTo, Inspect, Planner, SignObservation, Source
from .registry import register
from .sign_graph import SignGraph

GRAPH_PARAMS = ('passive_weight', 'min_score', 'margin', 'min_confidence')


@register('graph')
class GraphPlanner(Planner):
    def __init__(self, grid, allow_shortcuts: bool = True, max_inspections: int = 3,
                 reinspect_conf: float = 0.6, passive_cap: float = 1.0,
                 min_shortcut: int = 2, max_visits: int = 3, distrust_goal: bool = True,
                 inspect: bool = True, confirm_goal: bool = True, max_skip: int | None = 1,
                 retarget_within: float = 1.0, **params):
        super().__init__(grid, **params)
        self.allow_shortcuts = bool(allow_shortcuts)
        self.max_inspections = int(max_inspections)
        self.reinspect_conf = float(reinspect_conf)
        self.passive_cap = float(passive_cap)
        self.min_shortcut = int(min_shortcut)
        self.max_visits = int(max_visits)
        self.distrust_goal = bool(distrust_goal)
        self.inspect = bool(inspect)       # False: sighting-only mode (see docstring)
        self.confirm_goal = bool(confirm_goal)  # sighting-only: one close read before finishing
        # Shortcuts skip at most max_skip walls at once (None/0 = any), so the route stays
        # the sign route; retarget only within retarget_within cells of the current target.
        self.max_skip = int(max_skip) if max_skip else None
        self.retarget_within = float(retarget_within)
        self.graph_params = {k: params[k] for k in GRAPH_PARAMS if k in params}
        self.graph = SignGraph(grid, **self.graph_params)

    # ---- run state ----------------------------------------------------
    def reset(self, start: CellPose) -> None:
        self.graph = SignGraph(self.grid, **self.graph_params)
        self.readings: dict[CellPose, list[tuple[S, float]]] = defaultdict(list)
        self.passive: dict[CellPose, dict[S, float]] = defaultdict(lambda: defaultdict(float))
        self.committed: list[tuple[CellPose, S]] = []      # (face, sign obeyed) in order
        self.searched: set[CellPose] = set()               # faces turned to while lost
        self.arrived_at: CellPose | None = None            # face of the current visit
        self.visit_readings = 0                            # inspections in this visit
        self.visits: dict[CellPose, int] = defaultdict(int)
        self.used: dict[CellPose, set[S]] = defaultdict(set)   # signs obeyed per face
        self.log: list[str] = []                           # why each decision was made

    def observe(self, obs: SignObservation) -> bool:
        if obs.source is Source.INSPECT:
            self.readings[obs.face].append((obs.sign, obs.confidence))
            if obs.face == self.arrived_at:
                self.visit_readings += 1
        elif obs.confidence >= self.graph.min_confidence:
            self.passive[obs.face][obs.sign] += obs.confidence
        return self.graph.add(obs)

    # ---- evidence ------------------------------------------------------
    def fused(self, face: CellPose) -> tuple[S | None, float]:
        """Decision for the face in front: inspections + passive capped at passive_cap.
        Returns (sign, share of the fused weight) or (None, 0)."""
        votes: dict[S, float] = defaultdict(float)
        for s, c in self.readings.get(face, ()):
            votes[s] += c
        pv = self.passive.get(face)
        if pv:
            total = sum(pv.values())
            for s, w in pv.items():
                votes[s] += self.passive_cap * w / total
        if not votes:
            return None, 0.0
        top = max(votes, key=votes.get)
        return top, votes[top] / sum(votes.values())

    def passive_sign(self, face: CellPose) -> S | None:
        """Passive-only belief with SignGraph's known-rule (min_score, margin), or None."""
        pv = self.passive.get(face)
        if not pv:
            return None
        ranked = sorted(pv.items(), key=lambda kv: kv[1], reverse=True)
        w = self.graph.passive_weight
        top, score = ranked[0][0], ranked[0][1] * w
        second = ranked[1][1] * w if len(ranked) > 1 else 0.0
        return top if score >= self.graph.min_score and score - second >= self.graph.margin else None

    def passive_top(self, face: CellPose) -> S | None:
        pv = self.passive.get(face)
        return max(pv, key=pv.get) if pv else None

    def chain_to_goal(self, face: CellPose) -> bool:
        return self.graph.follow(face).end == 'goal'

    # ---- decisions -----------------------------------------------------
    def next_action(self, pose: CellPose) -> Action:
        if not self.grid.facing_wall(pose):
            return GoTo(self.grid.drive_to_wall(pose))
        if pose != self.arrived_at:                         # a new visit to this face
            self.arrived_at, self.visit_readings = pose, 0
            self.visits[pose] += 1
            if self.visits[pose] > self.max_visits:
                return GiveUp(f'{pose} visited {self.visits[pose]} times: stuck in a loop')

        # 5. Recover from an earlier misreading the graph has since exposed.
        fix = self.recovery(pose)
        if fix is not None:
            return fix

        if not self.inspect:
            return self.sighting_only(pose)
        if self.visit_readings == 0:
            return Inspect()
        sign, share = self.fused(pose)
        last_sign, last_conf = self.readings[pose][-1]
        can_reread = self.visit_readings < self.max_inspections      # per visit

        # 1. Verify: weak reading, or passive sightings disagree -> read again.
        pt = self.passive_top(pose)
        if can_reread and (last_conf < self.reinspect_conf or (pt is not None and pt != last_sign)):
            self.log.append(f'{pose}: re-read ({last_sign.name} {last_conf:.2f}, passive {pt})')
            return Inspect()

        known = self.passive_sign(pose)                      # what passive alone firmly says
        passive_goal = self.passive_top(pose) is S.GOAL
        # 3. Finish only on an inspected goal ...
        if sign is S.GOAL:
            other_goal = [f for f in self.graph.goal_faces() if f != pose]
            pt = self.passive_top(pose)
            firm = self.passive_sign(pose)               # passive alone, with margin
            if (other_goal or (self.distrust_goal and firm is not None)) and \
                    pt is not None and pt is not S.GOAL:
                # ... unless the goal is firmly known elsewhere and passive says otherwise here.
                self.log.append(f'{pose}: read GOAL but passive says {pt.name} (goal elsewhere: {other_goal}); obey it')
                sign = pt
            elif any(s is S.GOAL for s, _ in self.readings[pose]):
                return Finish()
            else:
                sign = S.EMPTY
        if sign is None:
            sign = S.EMPTY
        # A missed sign (dark, oblique) is far likelier than a phantom seen repeatedly from
        # afar: if inspections only saw EMPTY but passive sightings know the sign, use it.
        # For a goal that means finishing without an inspected goal (nothing contradicts it).
        if sign is S.EMPTY and known is not None:
            if known is S.GOAL and all(s is S.EMPTY for s, _ in self.readings[pose]):
                self.log.append(f'{pose}: inspected empty, passive knows GOAL; finish')
                return Finish()
            if known.is_turn:
                self.log.append(f'{pose}: inspected empty, passive knows {known.name}; obey it')
                sign = known
        # Back at a face we already obeyed: the signs can't loop, so that reading was wrong.
        if sign in self.used[pose] or (sign in (S.STOP, S.DO_NOT_ENTER)
                                       and self.used[pose] & {S.STOP, S.DO_NOT_ENTER}):
            # Led back here: most likely this face is the goal, misread. Read it afresh
            # (errors don't always repeat) and finish if it, or passive, ever said GOAL.
            if passive_goal or any(s is S.GOAL for s, _ in self.readings[pose]):
                self.log.append(f'{pose}: led back here after {sign.name}; GOAL seen here; finish')
                return Finish()
            if can_reread:
                self.log.append(f'{pose}: led back here after {sign.name}; re-read')
                return Inspect()
            alt = self.alternative(pose)
            self.log.append(f'{pose}: revisited after obeying {sign.name}; try {alt}')
            sign = alt if alt is not None else S.EMPTY

        if sign.is_turn:
            nxt = self.grid.drive_to_wall(pose.turned(sign.quarter_turns))
            # 2. Look ahead: known signs from nxt loop without the goal -> re-read here.
            chain = self.graph.follow(nxt)
            if chain.end == 'cycle' and can_reread:
                self.log.append(f'{pose}: {sign.name} leads into a known cycle; re-read')
                return Inspect()
            self.committed.append((pose, sign))
            self.used[pose].add(sign)
            self.searched.clear()
            return GoTo(self.shortcut(nxt))

        return self.lost(pose)                               # 4.

    def sighting_only(self, pose: CellPose) -> Action:
        """next_action at a wall without inspections (inspect: false)."""
        sign = self.passive_sign(pose)
        if sign is None:
            if self.visit_readings == 0:
                self.log.append(f'{pose}: no firm sighting; inspect')
                return Inspect()
            sign = self.fused(pose)[0] or S.EMPTY
        if sign is S.GOAL:
            if not self.confirm_goal:
                return Finish()
            if self.visit_readings == 0:
                self.log.append(f'{pose}: sighted GOAL; confirm with one close read')
                return Inspect()
            read = self.readings[pose][-1][0]
            if read in (S.GOAL, S.EMPTY):                # EMPTY: a miss, trust the sightings
                return Finish()
            self.log.append(f'{pose}: sighted GOAL but read {read.name} up close; obey the read')
            sign = read
        tried = sign in self.used[pose] or (sign in (S.STOP, S.DO_NOT_ENTER)
                                            and self.used[pose] & {S.STOP, S.DO_NOT_ENTER})
        if tried:                                        # led back here: a belief is wrong
            if self.visit_readings == 0:
                self.log.append(f'{pose}: led back here after {sign.name}; inspect')
                return Inspect()
            alt = self.alternative(pose)
            self.log.append(f'{pose}: revisited after obeying {sign.name}; try {alt}')
            sign = alt if alt is not None else S.EMPTY
        if sign.is_turn:
            nxt = self.grid.drive_to_wall(pose.turned(sign.quarter_turns))
            self.committed.append((pose, sign))
            self.used[pose].add(sign)
            self.searched.clear()
            return GoTo(self.shortcut(nxt))
        return self.lost(pose)

    def retarget(self, target: CellPose, robot_xy: tuple[float, float] | None = None
                 ) -> CellPose | None:
        """Mid-drive (sighting-only mode): a farther target if the known chain from
        `target` now goes on; None to keep driving to `target`. Only once the robot is
        within `retarget_within` cells of `target` (if robot_xy is given), so the drive from
        there to the new target follows the sign route instead of a different one."""
        if self.inspect or not self.allow_shortcuts:
            return None
        if robot_xy is not None and self.retarget_within > 0:
            tx, ty, _ = self.grid.pose_to_map(target)
            if ((robot_xy[0] - tx) ** 2 + (robot_xy[1] - ty) ** 2) ** 0.5 > \
                    self.retarget_within * self.grid.cell_size:
                return None
        new = self.shortcut(target)
        return new if new != target else None

    def alternative(self, face: CellPose) -> S | None:
        """Best turn sign for `face` not obeyed yet, by fused votes (any evidence)."""
        votes: dict[S, float] = defaultdict(float)
        for s, c in self.readings.get(face, ()):
            votes[s] += c
        for s, w in self.passive.get(face, {}).items():
            votes[s] += w
        tried = self.used[face] | ({S.STOP, S.DO_NOT_ENTER}
                                   if self.used[face] & {S.STOP, S.DO_NOT_ENTER} else set())
        options = [s for s in sorted(votes, key=votes.get, reverse=True)
                   if s.is_turn and s not in tried]
        return options[0] if options else None

    def shortcut(self, nxt: CellPose) -> CellPose:
        """Where to drive after obeying a sign whose next wall is nxt."""
        if not self.allow_shortcuts:
            return nxt
        chain = self.graph.follow(nxt)
        if chain.end not in ('goal', 'unknown'):
            return nxt
        skip = len(chain.faces) - 1                        # walls we could drive past
        if self.max_skip:
            skip = min(skip, self.max_skip)
        # Worth it? Sighting-only: any skip (no stops to save). Inspecting: min_shortcut,
        # but never more than max_skip allows (else max_skip 1 would disable shortcuts).
        need = 1 if not self.inspect else min(self.min_shortcut, self.max_skip or self.min_shortcut)
        if skip < max(1, need):
            return nxt
        # Faces in between are taken on belief; they count as obeyed.
        for f in chain.faces[:skip]:
            self.committed.append((f, self.graph.sign(f)))
        target = chain.faces[skip]
        self.log.append(f'shortcut {nxt} -> {target} ({skip} walls)')
        return target

    def recovery(self, pose: CellPose) -> Action | None:
        """Earliest obeyed sign that the graph now says was wrong -> go where it really leads."""
        for i, (face, used) in enumerate(self.committed):
            if face not in self.readings and self.graph.sign(face) is not None:
                believed = self.graph.sign(face)          # shortcut face: belief only
            else:
                believed, share = self.fused(face)
                if share < 0.75:                           # not convincingly wrong
                    continue
            if believed is None or believed == used or (
                    believed in (S.STOP, S.DO_NOT_ENTER) and used in (S.STOP, S.DO_NOT_ENTER)):
                continue
            del self.committed[i:]
            if believed is S.GOAL:
                target = face
            elif believed.is_turn:
                target = self.shortcut(self.grid.drive_to_wall(face.turned(believed.quarter_turns)))
            else:
                continue
            if target == pose:
                return None
            self.log.append(f'recover: {face} was {believed.name}, not {used.name}; go {target}')
            return GoTo(target)
        return None

    def lost(self, pose: CellPose) -> Action:
        cell_faces = [CellPose(pose.cell, h) for h in Heading
                      if self.grid.blocked(pose.cell, h) and h != pose.heading]
        self.searched.add(pose)
        # a) A wall of this cell already known to lead to the goal.
        for f in cell_faces:
            if f not in self.searched and self.visits[f] < 2 and self.chain_to_goal(f):
                self.log.append(f'{pose}: empty; turn to {f} (known chain to goal)')
                return GoTo(f)
        # b) Check this cell's other walls in place (left, right, back), skipping known blanks.
        for k in (1, -1, 2):
            f = pose.turned(k)
            if f in cell_faces and f not in self.searched and f not in self.readings:
                return GoTo(f)
        # c) Nearest wall anywhere whose known chain reaches the goal; d) nearest unexplored.
        known = [f for f in self.graph.faces() if f not in self.searched and self.visits[f] < 2
                 and self.graph.sign(f) is not None and self.chain_to_goal(f)]
        unexplored = [f for f in self.graph.faces() if f not in self.searched
                      and f not in self.readings and self.graph.sign(f) is None]
        for pool, why in ((known, 'known chain to goal'), (unexplored, 'unexplored')):
            if pool:
                f = min(pool, key=lambda f: (self.grid.path_length(pose.cell, f.cell),
                                             f.cell.row, f.cell.col, f.heading))
                self.log.append(f'{pose}: lost; go to {f} ({why})')
                self.searched.add(f) if why == 'unexplored' else None
                return GoTo(f)
        return GiveUp(f'lost at {pose.cell}: no known or unexplored wall left')
