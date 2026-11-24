"""
extract_maze: find interior walls of the maze in a saved map and print maze yaml.

    ros2 run team85_final extract_maze --map maps/map.yaml --maze config/maze.yaml --fit --render
    python -m team85_final.nodes.extract_maze --map ... --maze ... [--fit] [--threshold 0.5] [--write]

Geometry (rows, cols, cell_size, origin, yaw): with --fit it is fitted to the map's outer
walls (maze/extract.py fit_grid; the maze must be roughly axis-aligned in the map, true
when mapping starts square to the maze; check the printed aspect error, ~0 is good);
without --fit it is read from --maze. Walls are always re-extracted. Prints each edge's
occupancy score so borderline edges can be checked by hand.
"""

import argparse

import yaml

from ..maze.extract import extract_walls, fit_grid
from ..maze.grid import MazeGrid


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--map', required=True, help='map.yaml from map_saver')
    ap.add_argument('--maze', required=True,
                    help='maze yaml: geometry source (without --fit) and --write target')
    ap.add_argument('--fit', action='store_true', help='fit the grid to the map instead')
    ap.add_argument('--rows', type=int, default=3)
    ap.add_argument('--cols', type=int, default=6)
    ap.add_argument('--threshold', type=float, default=0.5)
    ap.add_argument('--render', action='store_true', help='print an ASCII drawing')
    ap.add_argument('--write', action='store_true', help='overwrite --maze with the result')
    a = ap.parse_args(argv)

    if a.fit:
        d = fit_grid(a.map, a.rows, a.cols)
        fit = d.pop('fit')
        print(f"fit: cell {d['cell_size']} m (x {fit['cell_x']}, y {fit['cell_y']}), "
              f"aspect error {fit['aspect_error']:.1%}, origin {d['origin']}")
    else:
        d = yaml.safe_load(open(a.maze))
    d['walls'] = []
    grid = MazeGrid.from_dict(d)
    scores = extract_walls(grid, a.map, a.threshold)
    for (cell, h), s in sorted(scores.items()):
        flag = 'WALL' if s >= a.threshold else '    '
        near = '  <- check' if abs(s - a.threshold) < 0.2 else ''
        print(f'{flag} ({cell.row},{cell.col}) {h.name[0]}: {s:.2f}{near}')
    if a.render:
        print(grid.render())
    out = yaml.safe_dump(grid.to_dict(), default_flow_style=None, sort_keys=False)
    print(out)
    if a.write:
        with open(a.maze, 'w') as f:
            f.write(out)


if __name__ == '__main__':
    main()
