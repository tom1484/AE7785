"""Compare planners over random sign layouts with sensing errors (paired trials).

    python -m team85_final.planning.montecarlo --maze config/maze_sim.yaml -n 2000 \\
        --planners grid graph "graph:allow_shortcuts=false" --errors nominal

Every planner runs on the same layouts, starts and per-face inspection errors, so the
differences are the planners'. Grades follow the handout (simulate.RunResult.grade).
Planner spec: name[:key=value,key=value] (values parsed as yaml: true/false/numbers).
"""

from __future__ import annotations

import argparse
import random
import statistics
from collections import Counter

import yaml

from ..maze.grid import MazeGrid
from .registry import make_planner
from .scenarios import ErrorModel, random_layout, random_start
from .simulate import fan_sightings, run


def parse_spec(spec: str) -> tuple[str, dict]:
    name, _, rest = spec.partition(':')
    params = {}
    for kv in filter(None, rest.split(',')):
        k, _, v = kv.partition('=')
        params[k.strip()] = yaml.safe_load(v)
    return name, params


def trials(grid: MazeGrid, n: int, seed: int, blank: float, min_length: int):
    rng = random.Random(seed)
    made = 0
    while made < n:
        signs = random_layout(grid, rng, blank=blank)
        start = random_start(grid, signs, rng, min_length=min_length)
        if start is not None:
            made += 1
            yield made, signs, start


def evaluate(grid: MazeGrid, specs: list[str], n: int = 500, seed: int = 0,
             errors: str = 'nominal', blank: float = 0.0, min_length: int = 4,
             passive: bool = True) -> dict[str, dict]:
    results = {s: [] for s in specs}
    for k, signs, start in trials(grid, n, seed, blank, min_length):
        for spec in specs:
            name, params = parse_spec(spec)
            em = ErrorModel.preset(errors, seed=seed * 100003 + k)
            see = fan_sightings(grid, signs) if passive else None
            p_fn = (lambda pose, see=see, em=em: em.passive(see(pose))) if see else None
            res = run(make_planner(name, grid, **params), grid, signs, start,
                      reader=em.reader, passive=p_fn)
            results[spec].append(res)
    return {spec: summarize(rs) for spec, rs in results.items()}


def summarize(rs) -> dict:
    grades = sorted(r.grade() for r in rs)
    outcomes = Counter(r.outcome.split(':')[0] for r in rs)
    return {
        'n': len(rs),
        'mean': statistics.fmean(grades),
        'p5': grades[max(0, int(0.05 * len(grades)) - 1)],
        'min': grades[0],
        'reached': sum(r.reached for r in rs) / len(rs),
        'perfect': sum(g >= 0.9999 for g in grades) / len(rs),
        'mean_abs_r_n': statistics.fmean(abs(r.distance - r.optimal) for r in rs if r.reached)
        if any(r.reached for r in rs) else float('nan'),
        'inspections': statistics.fmean(r.inspections for r in rs),
        'outcomes': dict(outcomes),
    }


def table(summary: dict[str, dict]) -> str:
    lines = ['| planner | mean grade | p5 | min | reached | perfect | mean abs(r-n) | inspections | outcomes |',
             '|---|---|---|---|---|---|---|---|---|']
    for spec, s in summary.items():
        lines.append(f"| {spec} | {s['mean']:.4f} | {s['p5']:.2f} | {s['min']:.2f} | "
                     f"{s['reached']:.1%} | {s['perfect']:.1%} | {s['mean_abs_r_n']:.2f} | "
                     f"{s['inspections']:.1f} | {s['outcomes']} |")
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--maze', required=True)
    ap.add_argument('-n', type=int, default=500)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--planners', nargs='+', default=['grid', 'graph'])
    ap.add_argument('--errors', default='nominal', choices=['none', 'nominal', 'harsh'])
    ap.add_argument('--blank', type=float, default=0.0, help='fraction of signs removed')
    ap.add_argument('--min-length', type=int, default=4)
    ap.add_argument('--no-passive', action='store_true')
    a = ap.parse_args()
    grid = MazeGrid.from_yaml(a.maze)
    s = evaluate(grid, a.planners, a.n, a.seed, a.errors, a.blank, a.min_length, not a.no_passive)
    print(f'{a.n} trials, errors={a.errors}, blank={a.blank}, maze={a.maze}\n')
    print(table(s))


if __name__ == '__main__':
    main()
