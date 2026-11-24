# Graph planner: strategies and test results

The graph planner (`planner:=graph`, `team85_final/planning/graph_planner.py`) follows the
signs like the basic grid planner but remembers every sign it has seen, including
signs seen in passing while driving. It uses that memory to catch and fix misreads.
This note explains each strategy, why it exists, and what the Monte Carlo
simulation measured for it.

All numbers below were re-run on 2026-11-20 against the current code
(1000 trials, seed 1, `config/maze_sim.yaml`). See [Reproduce](#reproduce).

---

## 1. Why reliability, not speed

Grade when the goal is reached: `1 − |r − n|·0.01 − 0.1·c`. Here `n` is the sign-path
length in cells, `r` is the distance actually driven, and `c = 1` for a collision or no
goal signal.

- There is **no time term**, so extra inspections and re-reads cost nothing.
- `|r − n|` penalizes a shortcut exactly like a detour: 1 point per cell either way.
- The big losses are **misreads** (finishing at the wrong wall, getting lost and
  giving up). A wrong-wall finish or give-up typically costs 10–60 points, while a
  one-cell detour costs 1.

So every rule in the planner aims to avoid misreads and recover from them. A rule
was kept only if the Monte Carlo grade went up.

---

## 2. Concepts

| Concept | Meaning |
|---|---|
| **Wall face** | A cell plus a heading that faces a wall (`CellPose`). Signs hang on wall faces. |
| **Edge** | Because the maze walls are known, a face's sign fully determines where it leads: turn as the sign says, then drive to the next wall (`drive_to_wall`). The graph stores only a *belief per face*; edges are derived, never stored. |
| **Inspection** | The robot stops squarely in front of a wall and reads it (5 frames). Strong evidence. |
| **Passive sighting** | Any sign the camera sees while driving or turning. It is placed on a face by casting the camera ray through the known maze, with a LIDAR range check. Weak but plentiful evidence. |
| **SignGraph** (`sign_graph.py`) | Confidence-weighted votes per face. Inspections weigh 1.0 and passive sightings 0.3 (`passive_weight`). Passive sightings below 0.7 confidence are dropped (`min_confidence`). A face is **known** once the leading class has score ≥ 0.8 (`min_score`) and leads the runner-up by ≥ 0.5 (`margin`). |
| **Chain** (`follow`) | Start at a face and follow known edges until the chain ends in `goal`, `unknown` (a face whose sign isn't known) or `cycle`. |

Valid sign layouts never loop: every sign leads toward the goal. That fact powers
several strategies below. If the planner is ever led back to a face it already
obeyed, or a known chain forms a cycle, then some belief is wrong.

---

## 3. The decision at each wall, in order

`next_action(pose)` runs these checks each time the robot stands at a wall:

```
not facing a wall?            → drive to the wall ahead
new visit to this face        → count it; > max_visits (3)  → GIVE UP        (§4.7)
graph exposes an old misread  → drive to where the right sign leads          (§4.6 Recover)
not read yet this visit       → INSPECT
weak / passive disagrees      → INSPECT again (≤ max_inspections = 3)         (§4.1 Verify)
reads GOAL                    → goal rules                                   (§4.3)
reads EMPTY, passive knows it → obey passive                                 (§4.2)
led back to an obeyed face    → finish if GOAL ever seen here, else re-read,
                                else try the next-best sign                  (§4.5)
reads a turn                  → look ahead: known cycle? re-read             (§4.4)
                                else obey (+ shortcut if allowed)            (§4.8)
nothing usable                → lost search                                  (§4.7)
```

---

## 4. Strategies

### 4.1 Verify readings (re-read)
Every wall is inspected. The planner reads the same face again (up to
`max_inspections = 3` per visit) when:
- the last reading's confidence is below `reinspect_conf = 0.6`, or
- the passive sightings of that face mostly say something else.

The decision fuses all inspections with passive evidence. The passive evidence is
**capped at the weight of one inspection** (`passive_cap = 1.0`). Without the cap,
hundreds of far-away frames would outvote close-up reads. Telling stop from goal at a
distance is the detector's weak spot.

### 4.2 Trust passive over an EMPTY inspection
If inspections only saw EMPTY but passive sightings *firmly* know the face's sign, the
planner obeys the passive sign. If that sign is GOAL, it finishes there.
Reasoning: missing a sign in a dark corner or at an angle is far likelier than seeing
the same phantom sign from many places.

### 4.3 Goal rules (`distrust_goal = true`)
- Finish on an inspected GOAL, **unless** passive firmly says another sign here, or the
  goal is already known to be on another face. In that case the planner obeys the
  passive sign, since stop read as goal is the known weak spot.
- Also finish when inspections saw only EMPTY and passive firmly knows GOAL (§4.2).
- Also finish when the signs lead back to a face where GOAL was ever seen. Signs never
  loop, so that face must be the goal.

### 4.4 Look ahead
Before obeying a turn, the planner follows the known chain from the wall it would drive
to. If that chain is a **cycle that never reaches the goal**, a belief is wrong, so it
re-reads the current face before moving.

### 4.5 Revisited face
If the robot is led back to a face where it already obeyed the sign it now reads, that
reading was wrong (signs don't loop). The planner then tries, in order:
1. Finish, if GOAL was ever read or passively seen here (most likely a misread goal).
2. Re-read the face, since errors don't always repeat.
3. Obey the next-best turn sign not tried yet, ranked by all evidence.

### 4.6 Recover from an earlier misread
At every wall the planner rechecks the signs it has already obeyed. If the graph now
says, convincingly (≥ 75 % of the fused vote), that one of them was something else, it
drops the wrong branch and drives straight to where the correct sign leads. If the
correct sign is GOAL, it drives back to that face.

### 4.7 Lost (no sign in front)
When the face in front has no usable sign, the planner tries these in order:
1. Turn to a wall of **this cell** whose known chain reaches the goal.
2. Check this cell's other walls in place (left, right, back), like the grid planner,
   skipping walls already read blank.
3. Drive to the **nearest wall anywhere** whose known chain reaches the goal.
4. Drive to the nearest unexplored wall.

**Visit cap** (`max_visits = 3`): give up after the 4th visit to one face. Wandering
costs 1 point per cell and can go on indefinitely.

### 4.8 Shortcuts (`allow_shortcuts = true`, the team's choice)
After obeying a sign, if the signs after the next wall are already known (chain longer
than `min_shortcut = 2` walls, ending in `goal` or `unknown`), the robot drives straight
to the end of that known chain and inspects there. Skipped cells make `r < n`, which
costs 1 point per cell. This is enabled for fun, not score, and the TA will be told.
(The tests below show it is roughly neutral to slightly positive anyway.)

In practice (real run 2026-11-23) it rarely fires: the camera mostly sees walls *ahead*,
so the wall after a turn is usually unknown until the robot gets there, and known chains
stay short. Sighting-only mode (§4.10) fixes this by retargeting while driving.

### 4.10 Sighting-only mode (`inspect = false`, for show)
No stops to read walls; it is fast and smooth, and slightly less reliable.
- **At a wall,** the sign is the firm passive belief. It inspects once only if nothing is
  known there.
- **At a believed goal,** with `confirm_goal = true` (the default), it reads the sign once up
  close before finishing. If the read says something else, it obeys the read; an EMPTY read
  is treated as a miss.
- **Shortcuts** take any known chain (no `min_shortcut`). While Nav2 drives, every graph
  change calls `retarget(target)`: if the known chain now continues past the current target,
  the mission sends Nav2 the farther goal. Results of the replaced goal are ignored by
  sequence id.
- **Rolling shortcuts (`max_skip = 1`, `retarget_within = 1` cell, 2026-11-23).** A shortcut
  skips at most `max_skip` walls, and a mid-drive retarget fires only once the robot is
  within `retarget_within` cells of its current target. The mission re-checks this at 10 Hz,
  not only on new sightings. The robot then rolls through the sign route instead of
  re-planning a different one (unlimited jumps let NavFn pick a different equal-length route
  on the real maze). In inspect mode the skip threshold is min(min_shortcut, max_skip).
  Monte Carlo (real maze, 500): graph 0.9972 / 0.9869 (nominal / harsh) vs 0.9962 / 0.9827
  unlimited. Sighting-only is within noise. Caveat: at a U-turn sign the 1-cell gate turns
  the robot about 1 cell early (r < n by up to 2).
- **No ALIGN on arrival,** except right before an inspection.

Monte Carlo on the real maze layout, 500 trials, seed 1. Mid-drive retargeting is not
simulated, only the decisions at each wall.

| errors | graph | inspect=false | inspect=false, confirm_goal=false |
|---|---|---|---|
| nominal | 0.996 (10.1 reads) | 0.993 (1.4 reads) | 0.985 (0.4 reads; 21 wrong-wall finishes) |
| harsh | 0.983 | 0.975 | 0.960 (50 wrong-wall finishes) |

### 4.11 Rejected after measuring
- **Finish at a wall the signs lead to twice that still reads EMPTY.** This turned
  give-ups into wrong-wall finishes and was no better on average.

### Parameters (all graph-planner defaults; `config/final.yaml` sets only `min_confidence: 0.7`)
| Parameter | Default | Strategy |
|---|---|---|
| `max_inspections` | 3 | re-reads per visit (§4.1, §4.4, §4.5) |
| `reinspect_conf` | 0.6 | re-read below this confidence (§4.1) |
| `passive_cap` | 1.0 | passive weight cap, in inspections (§4.1) |
| `distrust_goal` | true | let firm passive override an inspected GOAL (§4.3) |
| `max_visits` | 3 | visits per face before giving up (§4.7) |
| `allow_shortcuts` / `min_shortcut` | true / 2 | §4.8 |
| `inspect` / `confirm_goal` | true / true | §4.10 sighting-only mode (`inspect: false`) |
| `max_skip` / `retarget_within` | 1 / 1.0 cell | §4.10 rolling shortcuts (`max_skip: 0` = unlimited) |
| `passive_weight`, `min_score`, `margin`, `min_confidence` | 0.3, 0.8, 0.5, 0.7 | SignGraph "known" rule |

---

## 5. How it is tested: paired Monte Carlo

`python -m team85_final.planning.montecarlo` (`montecarlo.py`, `scenarios.py`, `simulate.py`)

- **Maze:** the real sim maze geometry (`maze_sim.yaml`, 3×6).
- **Layouts:** `random_layout` picks a random goal face and grows a valid sign layout
  backwards from it, so every signed face leads to the goal and signs never loop. With
  `--blank 0.1`, 10 % of the signs are then removed.
- **Starts:** random cell and heading whose sign path is ≥ 4 cells long.
- **Paired:** every planner gets the same layouts, starts and per-face errors, so the
  differences between planners are real and not noise from different trials.
- **Inspection error model** (seeded per face, so systematic errors repeat on re-reads
  with probability `persist`). Wrong reads have confidence 0.4–0.9; right reads 0.7–1.0.

  | preset | miss (→EMPTY) | phantom | L↔R | stop→goal | goal→stop | persist |
  |---|---|---|---|---|---|---|
  | nominal | 3 % | 1 % | 1 % | 2 % | 4 % | 0.7 |
  | harsh | 8 % | 3 % | 3 % | 5 % | 10 % | 0.8 |

- **Passive model** (`fan_sightings`): 9 rays across the 62° field of view, range 2.5 m,
  cast from the cell center at every cell driven through and at every stop. Error rates
  are 3× the inspection rates, and each sighting arrives as 4 frames.
- **Grade:** the handout formula, including partial credit when the goal isn't reached.

---

## 6. Results

### 6.1 Planner comparison
Mean grade over 1000 trials:

| scenario | grid | graph | graph, no shortcuts |
|---|---|---|---|
| perfect sensing | 1.000 | 1.000 | 1.000 |
| nominal errors | 0.979 | **0.996** | 0.995 |
| harsh errors | 0.942 | **0.984** | 0.978 |
| nominal + 10 % signs missing | 0.984 | **0.999** | 0.997 |
| nominal, **no passive sightings** | **0.979** | 0.976 | 0.975 |

Detail (reached = finished at the true goal; perfect = grade 1.0; |r−n| over reached runs):

| scenario | planner | reached | perfect | p5 | worst | mean \|r−n\| | wrong-wall finish | give up |
|---|---|---|---|---|---|---|---|---|
| nominal | grid | 94.8 % | 89.6 % | 0.86 | 0.00 | 0.53 | 25 | 27 |
| nominal | graph | 99.1 % | 95.3 % | 1.00 | 0.39 | 0.15 | 3 | 6 |
| harsh | grid | 85.5 % | 72.5 % | 0.64 | −0.34 | 2.32 | 56 | 89 |
| harsh | graph | 96.2 % | 81.6 % | 0.90 | 0.27 | 0.67 | 18 | 20 |
| 10 % missing | grid | 95.5 % | 90.9 % | 0.90 | 0.00 | 0.46 | 16 | 28 |
| 10 % missing | graph | 100 % | 95.6 % | 1.00 | 0.90 | 0.15 | 0 | 0 |
| no passive | grid | 94.8 % | 89.6 % | 0.83 | 0.00 | 0.55 | 25 | 27 |
| no passive | graph | 93.4 % | 86.3 % | 0.82 | 0.00 | 0.33 | 23 | 43 |

What this shows:
- With passive sightings, the graph planner cuts wrong-wall finishes and give-ups by
  3–8× and lifts the worst cases the most (harsh p5: 0.64 → 0.90).
- **Without passive sightings it is slightly worse than the grid planner** (more
  give-ups). Its advantage comes entirely from passive sightings. In Gazebo they were
  36/36 correct, but on the real robot they are still untested.
- With perfect sensing, all three planners score 1.000. Shortcuts cost points in only 1
  of 1000 trials (worst 0.96).

### 6.2 Ablations: switch off one strategy at a time
Mean grade (Δ vs full graph planner):

| variant | what it disables | nominal | harsh | harsh: perfect / wrong-wall / give-up |
|---|---|---|---|---|
| **full graph planner** | - | **0.9961** | **0.9835** | 81.6 % / 18 / 20 |
| `max_inspections=1` | all re-reads: verify §4.1, look-ahead §4.4, revisit re-read §4.5 | 0.9951 (−0.001) | 0.9810 (−0.003) | 77.0 % / 16 / 23 |
| `passive_cap=1000` | cap on passive evidence §4.1 | 0.9953 (−0.001) | 0.9797 (−0.004) | 73.9 % / 13 / 28 |
| `distrust_goal=false` | passive veto of a read GOAL §4.3 | 0.9930 (−0.003) | 0.9799 (−0.004) | 84.8 % / **35** / 18 |
| `max_visits=100` | visit cap §4.7 | 0.9867 (−0.009) | 0.9628 (**−0.021**) | 81.6 % / 21 / 4 (+6 endless, worst −3.42) |
| `allow_shortcuts=false` | shortcuts §4.8 | 0.9952 (−0.001) | 0.9778 (−0.006) | 80.5 % / 20 / 33 |
| `reinspect_conf=0` | low-confidence re-read trigger §4.1 | 0.9961 (0) | 0.9836 (0) | 81.7 % / 19 / 19 |

What this shows:
- **Visit cap** matters most: without it, a confused robot wanders until the action limit
  and the grade goes deeply negative.
- **Distrusting GOAL** halves wrong-wall finishes (35 → 18 in harsh). It costs a few
  perfect runs, but a wrong-wall finish costs far more.
- **Passive cap** and **re-reads** each add a few tenths of a point, mostly by turning
  imperfect runs into perfect ones (smaller |r − n|).
- **Shortcuts** help slightly under harsh errors: every wall skipped is a wall that
  can't be misread.
- **The low-confidence trigger alone does nothing measurable** in this error model. Re-reads
  are driven by passive disagreement. It is harmless, but its value is unproven.
- **Not ablated** (no switch in the code): recovery §4.6, obeying passive over EMPTY §4.2,
  the lost-search order §4.7.

---

## 7. Caveats

- The error model is **assumed**, not measured on the real robot. The nominal and harsh
  presets bracket a guess, and the passive model has no placement noise (sightings always
  land on the right face).
- Gazebo (from the project notes, not re-run here): grid and graph planners both reached
  the goal reading every wall, 5/5; passive graph 36/36 correct.
- **Real robot: untested.** Before choosing `planner:=graph` on demo day, check that
  passive sightings are mostly correct on the real maze. Run with `graph_dump` and compare
  against the real signs. If they are poor, the grid planner is the safer choice (§6.1, no
  passive row).

---

## Reproduce

```zsh
cd ~/Workspace/turtlebot3_ws/src/team85_final
MC="python -m team85_final.planning.montecarlo --maze config/maze_sim.yaml -n 1000 --seed 1"
$MC --planners grid graph "graph:allow_shortcuts=false" --errors nominal   # also: harsh | none
$MC --planners grid graph "graph:allow_shortcuts=false" --errors nominal --blank 0.1
$MC --planners grid graph "graph:allow_shortcuts=false" --errors nominal --no-passive
$MC --errors harsh --planners graph graph:max_inspections=1 graph:passive_cap=1000 \
    graph:distrust_goal=false graph:max_visits=100 graph:reinspect_conf=0
```
Each run takes about 10 s on the mini.
