"""
plot_graph: draw a graph dump (mission param graph_dump) on the saved map, to check by eye.

    python -m team85_final.nodes.plot_graph --map maps/map.yaml --maze config/maze.yaml \\
        --graph /tmp/graph.yaml --out /tmp/graph.png

Each wall face with evidence gets a label just in front of its wall: the believed sign
(L, R, STOP, DNE, GOAL; grey "x?" = seen but not yet certain) and the number of sightings.
Turn signs also get an arrow pointing where the robot would go next (turn + drive).
Cells are labelled row,col (row 0 = bottom, col 0 = left, as in maze.yaml).
"""

import argparse
import math

import cv2
import numpy as np
import yaml

from ..maze.geometry import Cell, CellPose, Heading
from ..maze.grid import MazeGrid

SHORT = {'left': 'L', 'right': 'R', 'stop': 'STOP', 'do_not_enter': 'DNE', 'goal': 'GOAL',
         'empty': '-'}
COLOR = {'left': (200, 120, 0), 'right': (200, 120, 0), 'stop': (30, 30, 220),
         'do_not_enter': (30, 30, 220), 'goal': (0, 170, 230), 'empty': (130, 130, 130)}
TURNS = {'left': 1, 'right': -1, 'stop': 2, 'do_not_enter': 2}


def render(map_yaml: str, maze_yaml: str, graph_yaml: str, scale: int = 8) -> np.ndarray:
    meta = yaml.safe_load(open(map_yaml))
    import os
    img = cv2.imread(os.path.join(os.path.dirname(map_yaml), meta['image']), cv2.IMREAD_GRAYSCALE)
    res, (ox, oy) = float(meta['resolution']), meta['origin'][:2]
    h = img.shape[0]
    out = cv2.cvtColor(cv2.resize(img, None, fx=scale, fy=scale,
                                  interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
    out = cv2.addWeighted(out, 0.55, np.full_like(out, 255), 0.45, 0)       # fade the map

    def px(x, y):
        return int((x - ox) / res * scale), int((h - (y - oy) / res) * scale)

    g = MazeGrid.from_yaml(maze_yaml)
    s = g.cell_size
    for r in range(g.rows + 1):                                        # grid lines
        cv2.line(out, px(*g._to_map(0, r * s)), px(*g._to_map(g.cols * s, r * s)), (210, 210, 210), 1)
    for c in range(g.cols + 1):
        cv2.line(out, px(*g._to_map(c * s, 0)), px(*g._to_map(c * s, g.rows * s)), (210, 210, 210), 1)
    for cell in g.cells():                                             # maze walls (from maze.yaml)
        for hd in (Heading.EAST, Heading.NORTH, Heading.WEST, Heading.SOUTH):
            if g.blocked(cell, hd):
                x0, y0 = cell.col * s, cell.row * s
                a, b = {Heading.EAST: ((x0 + s, y0), (x0 + s, y0 + s)),
                        Heading.WEST: ((x0, y0), (x0, y0 + s)),
                        Heading.NORTH: ((x0, y0 + s), (x0 + s, y0 + s)),
                        Heading.SOUTH: ((x0, y0), (x0 + s, y0))}[hd]
                cv2.line(out, px(*g._to_map(*a)), px(*g._to_map(*b)), (60, 60, 60), 3)
        cx, cy = g.cell_center(cell)
        cv2.putText(out, f'{cell.row},{cell.col}', px(cx - 0.08, cy - 0.03),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (170, 170, 170), 1, cv2.LINE_AA)

    graph = yaml.safe_load(open(graph_yaml))
    font = cv2.FONT_HERSHEY_SIMPLEX
    for f in graph['faces']:
        r, c, hd = f['face']
        face = CellPose(Cell(int(r), int(c)), Heading.parse(hd))
        x, y, yaw = g.pose_to_map(face)
        d = 0.3 * s                                                    # just in front of the wall
        fx, fy = x + d * math.cos(yaw), y + d * math.sin(yaw)
        # Faces on the two sides of one E/W wall sit 0.6 cell apart: stagger them so the
        # labels don't overlap (E face above the centre line, W face below).
        if face.heading in (Heading.EAST, Heading.WEST):
            k = 0.12 * s * (1 if face.heading == Heading.EAST else -1)
            fx, fy = fx - k * math.sin(g.yaw), fy + k * math.cos(g.yaw)
        sign = f['sign']
        top = next(iter(f['votes']), 'empty')
        label = SHORT.get(sign, '?') if sign else f'{SHORT.get(top, top)}?'
        color = COLOR.get(sign, (0, 0, 0)) if sign else (150, 150, 150)
        text = f"{label} {f['n_obs']}"
        (tw, th), _ = cv2.getTextSize(text, font, 0.5, 2)
        p = px(fx, fy)
        cv2.rectangle(out, (p[0] - tw // 2 - 3, p[1] - th - 4), (p[0] + tw // 2 + 3, p[1] + 4),
                      (255, 255, 255), -1)
        cv2.putText(out, text, (p[0] - tw // 2, p[1]), font, 0.5, color, 2, cv2.LINE_AA)
        if sign in TURNS:                                              # where it sends the robot
            nyaw = yaw + TURNS[sign] * math.pi / 2
            q0 = px(fx - 0.12 * math.cos(yaw), fy - 0.12 * math.sin(yaw))
            q1 = px(fx - 0.12 * math.cos(yaw) + 0.2 * math.cos(nyaw),
                    fy - 0.12 * math.sin(yaw) + 0.2 * math.sin(nyaw))
            cv2.arrowedLine(out, q0, q1, color, 2, cv2.LINE_AA, tipLength=0.35)
    n_known = sum(f['sign'] is not None for f in graph['faces'])
    cv2.putText(out, f"{n_known} known / {len(graph['faces'])} faces with sightings; label = sign "
                f"+ sightings; arrow = where a turn sign sends the robot", (10, 20), font, 0.5,
                (0, 0, 0), 1, cv2.LINE_AA)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--map', required=True)
    ap.add_argument('--maze', required=True)
    ap.add_argument('--graph', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--scale', type=int, default=8)
    a = ap.parse_args(argv)
    cv2.imwrite(a.out, render(a.map, a.maze, a.graph, a.scale))
    print(a.out)


if __name__ == '__main__':
    main()
