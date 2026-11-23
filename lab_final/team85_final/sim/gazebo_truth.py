"""Ground-truth maze walls from the Gazebo maze mesh, to check a sim map (dev tool).

Renders the wall faces of turtlebot3_gazebo's 7785_maze mesh top-down into an occupancy
image (+ map.yaml) in the WORLD frame, fits the grid and extracts walls with the same code
as for SLAM maps. Compare a sim map's WALLS and CELL SIZE with it, not its origin:
maze_truth.yaml shifts the origin by the spawn pose assuming the map starts at the spawn,
but Cartographer's map_sim (2026-11-20) put the start ~0.37 m away from its origin.
Verified: mesh walls = Lab 5 map_sim walls = final map_sim walls; true cell 0.9163 m.

    python -I sim/gazebo_truth.py --out /tmp/truth     # writes truth_map.{pgm,yaml}, maze_truth.yaml

Constants come from sim/7785_maze_signs.world (<state>: maze model and robot poses).
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import yaml

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
from team85_final.maze.extract import extract_walls, fit_grid  # noqa: E402
from team85_final.maze.grid import MazeGrid  # noqa: E402

MESH = (PKG.parent / 'turtlebot3_simulations' / 'turtlebot3_gazebo' / 'models' / '7785_maze'
        / 'meshes' / '7785_maze.dae')
MAZE_POSE = (-1.38825, -1.40559)        # <state> pose of model 7785_maze
SPAWN = (-0.059578, -0.050137)          # <state> pose of model burger


def wall_faces(mesh: Path) -> np.ndarray:
    """All mesh triangles in world coordinates, shape (n, 3, 3)."""
    r = ET.parse(mesh).getroot()
    ns = {'c': r.tag.split('}')[0][1:]}
    geo = {}
    for g in r.findall('.//c:geometry', ns):
        m = g.find('c:mesh', ns)
        vin = m.find('c:vertices/c:input[@semantic="POSITION"]', ns)
        V = np.array(m.find(f"c:source[@id='{vin.get('source')[1:]}']/c:float_array", ns)
                     .text.split(), float).reshape(-1, 3)
        T = []
        for prim in list(m.findall('c:triangles', ns)) + list(m.findall('c:polylist', ns)):
            inputs = prim.findall('c:input', ns)
            stride = max(int(i.get('offset')) for i in inputs) + 1
            voff = int(next(i for i in inputs if i.get('semantic') == 'VERTEX').get('offset'))
            idx = np.array(prim.find('c:p', ns).text.split(), int).reshape(-1, stride)[:, voff]
            if prim.tag.endswith('polylist'):
                k = 0
                for n in np.array(prim.find('c:vcount', ns).text.split(), int):
                    poly = idx[k:k + n]
                    k += n
                    T += [[poly[0], poly[j], poly[j + 1]] for j in range(1, n - 1)]
            else:
                T += idx.reshape(-1, 3).tolist()
        geo['#' + g.get('id')] = (V, np.array(T, int))

    faces = []

    def walk(node, M):
        mat = node.find('c:matrix', ns)
        if mat is not None:
            M = M @ np.array(mat.text.split(), float).reshape(4, 4)
        for ig in node.findall('c:instance_geometry', ns):
            V, T = geo[ig.get('url')]
            W = (M @ np.c_[V, np.ones(len(V))].T).T[:, :3]
            if len(T):
                faces.append(W[T])
        for ch in node.findall('c:node', ns):
            walk(ch, M)

    for n in r.find('.//c:visual_scene', ns).findall('c:node', ns):
        walk(n, np.eye(4))
    F = np.vstack(faces)
    F[:, :, 0] += MAZE_POSE[0]
    F[:, :, 1] += MAZE_POSE[1]
    return F


def truth_image(res: float = 0.01, x0: float = -0.9, y0: float = -0.9):
    """Top-down wall image in the world frame: (uint8 img, 0 = wall, row 0 = top; res, x0, y0)."""
    F = wall_faces(MESH)
    F = F[(F[:, :, 2].max(1) > 0.05) & (F[:, :, 2].max(1) < 1.0)]   # walls only
    H, W = int(3.6 / res), int(6.4 / res)
    img = np.full((H, W), 255, np.uint8)
    for f in F:
        cv2.fillPoly(img, [np.c_[(f[:, 0] - x0) / res, H - 1 - (f[:, 1] - y0) / res]
                           .astype(np.int32)], 0)
    return img, res, x0, y0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    img, res, x0, y0 = truth_image()
    cv2.imwrite(str(out / 'truth_map.pgm'), img)
    (out / 'truth_map.yaml').write_text(
        f'image: truth_map.pgm\nresolution: {res}\norigin: [{x0}, {y0}, 0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.25\n')

    geom = fit_grid(out / 'truth_map.yaml', 3, 6)
    fit = geom.pop('fit')
    grid = MazeGrid.from_dict(geom)
    extract_walls(grid, out / 'truth_map.yaml')
    maze = grid.to_dict()
    maze['origin'] = [round(maze['origin'][0] - SPAWN[0], 4), round(maze['origin'][1] - SPAWN[1], 4)]
    (out / 'maze_truth.yaml').write_text(yaml.safe_dump(maze, default_flow_style=None,
                                                        sort_keys=False))
    print(f'world-frame fit {geom} {fit}\nmap-frame origin {maze["origin"]}\n{grid.render()}')


if __name__ == '__main__':
    os.environ.setdefault('PYTHONDONTWRITEBYTECODE', '1')
    main()
