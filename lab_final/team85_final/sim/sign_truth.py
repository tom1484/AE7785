"""Ground-truth signs of the Gazebo world as wall faces, and a graph check (dev tool).

Each sign model in sim/7785_maze_signs.world (<state> poses) is snapped to the maze grid
fitted to the maze mesh (gazebo_truth.py, world frame). Faces are in grid terms
(row, col, heading), so they compare directly with maze_sim.yaml and the mission
node's graph, whatever the map frame.

- Side of the wall: walls are 6.4 mm thick in the mesh and the sign meshes are flat
  (local xy plane, 3 mm thick along local z); the side is where the sign's middle lies
  relative to the wall's centerline. (Front/back of a sign says nothing: some signs face
  the wall, and the closed 3 mm meshes render from both sides.)
- Class: Star = goal, Stop = stop, UTurn = do_not_enter. Arrows point along local +x
  (C*Arrow, curved) or -x (S*Arrow, straight) in the mesh, so in the world along
  +-(cos Y, sin Y) for any roll; left/right is that direction against the viewer's right.
  Checked against the grid planner's sim run ((0,0)W right, (2,0)N right, ...).

    python -I sim/sign_truth.py                          # print the truth table
    python -I sim/sign_truth.py --compare /tmp/graph.yaml   # graph_dump of a mission run
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

SIM = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(SIM.parent))
from gazebo_truth import MESH, wall_faces  # noqa: E402
from team85_final.maze.geometry import Cell, CellPose, Heading  # noqa: E402

WORLD = SIM / '7785_maze_signs.world'
GRID_ORIGIN = (-0.53, -0.50)      # world-frame fit of the maze mesh (gazebo_truth.py)
CELL = 0.9163
CLASS = {'Star': 'goal', 'Stop': 'stop', 'UTurn': 'do_not_enter'}


def sign_poses(world: Path = WORLD):
    state = ET.parse(world).getroot().find('world').find('state')
    for m in state.findall('model'):
        name = m.get('name')
        kind = name.split('_')[0]
        if kind in CLASS or kind.endswith('Arrow'):
            yield name, kind, [float(v) for v in m.find('pose').text.split()]


def wall_planes():
    """Axis-aligned wall surfaces of the maze mesh: (x of walls along y, y of walls along x)."""
    F = wall_faces(MESH)
    F = F[(F[:, :, 2].max(1) > 0.05) & (F[:, :, 2].max(1) < 1.0)]
    xs = F[np.ptp(F[:, :, 0], axis=1) < 1e-4][:, 0, 0]
    ys = F[np.ptp(F[:, :, 1], axis=1) < 1e-4][:, 0, 1]
    return np.unique(xs.round(4)), np.unique(ys.round(4))


def truth(world: Path = WORLD) -> dict[CellPose, tuple[str, str]]:
    """{face: (class name, model name)}."""
    planes_x, planes_y = wall_planes()
    out = {}
    for name, kind, (x, y, _, roll, _, yaw) in sign_poses(world):
        mx, my = x - GRID_ORIGIN[0], y - GRID_ORIGIN[1]
        fx, fy = mx / CELL, my / CELL
        vertical = abs(fx - round(fx)) < abs(fy - round(fy))      # on a wall along y?
        # The mesh is 3 mm thick along its local z: the sign's middle is 1.5 mm off the pose.
        zx, zy = ((math.sin(yaw), -math.cos(yaw)) if roll > 0 else
                  (-math.sin(yaw), math.cos(yaw)))
        cx, cy = x + 0.0015 * zx, y + 0.0015 * zy
        c, planes = (cx, planes_x) if vertical else (cy, planes_y)
        near = planes[np.abs(planes - c) < 0.02]
        if len(near) == 0:
            raise ValueError(f'{name}: no wall surface near {c:.4f}')
        side = 1 if c > (near.min() + near.max()) / 2 else -1    # + = east / north side
        if vertical:
            line = round(fx)
            cell = Cell(math.floor(fy), line if side > 0 else line - 1)
            heading = Heading.WEST if side > 0 else Heading.EAST
        else:
            line = round(fy)
            cell = Cell(line if side > 0 else line - 1, math.floor(fx))
            heading = Heading.SOUTH if side > 0 else Heading.NORTH
        face = CellPose(cell, heading)
        if kind in CLASS:
            cls = CLASS[kind]
        else:
            d = 1.0 if kind.startswith(('BlueC', 'GreenC')) else -1.0
            ax, ay = d * math.cos(yaw), d * math.sin(yaw)
            right = heading.yaw - math.pi / 2              # viewer's right
            cls = 'right' if ax * math.cos(right) + ay * math.sin(right) > 0 else 'left'
        if face in out:
            raise ValueError(f'two signs on {face}: {out[face][1]}, {name}')
        out[face] = (cls, name)
    return out


def face_key(f) -> CellPose:
    r, c, h = f
    return CellPose(Cell(int(r), int(c)), Heading.parse(str(h)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--compare', help='graph yaml written by the mission node (graph_dump)')
    a = ap.parse_args()
    t = truth()
    if not a.compare:
        for f in sorted(t, key=lambda f: (f.cell.row, f.cell.col, f.heading)):
            print(f'{str(f):8s} {t[f][0]:13s} {t[f][1]}')
        print(f'{len(t)} signs')
        return

    g = yaml.safe_load(Path(a.compare).read_text())
    rows, right, wrong, unsure, phantom = [], 0, 0, 0, 0
    for e in g['faces']:
        f = face_key(e['face'])
        true = t.get(f, ('empty', ''))[0]
        top = next(iter(e['votes']), None)
        if e['sign'] is None:
            verdict, unsure = 'unsure', unsure + 1
        elif e['sign'] == true:
            verdict, right = 'ok', right + 1
        elif true == 'empty':
            verdict, phantom = 'PHANTOM', phantom + 1
        else:
            verdict, wrong = 'WRONG', wrong + 1
        rows.append(f'{str(f):8s} truth {true:13s} graph {str(e["sign"]):13s} '
                    f'(top {top}, {e["n_obs"]} obs) {verdict}   votes {e["votes"]}')
    seen = {face_key(e['face']) for e in g['faces']}
    print('\n'.join(rows))
    print(f'\nknown signs: {right} correct, {wrong} wrong class, {phantom} on a face with no sign;'
          f' {unsure} faces with evidence but not known; '
          f'{len(seen & set(t))}/{len(t)} true signs got any evidence')
    if g.get('targets'):
        print('targets:', ' '.join(g['targets']))


if __name__ == '__main__':
    main()
