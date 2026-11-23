from pathlib import Path

import cv2
import numpy as np

from team85_final.maze import Cell, CellPose, Heading, MazeGrid
from team85_final.maze.extract import extract_walls
import math

from team85_final.maze.sign_locator import camera_pose, lidar_agrees, raycast


def test_extract_finds_drawn_wall(tmp_path):
    # 3x6 maze of 1 m cells at 0.05 m/px, map origin 0.5 m outside the maze corner.
    res, pad = 0.05, 10
    h, w = 3 * 20 + 2 * pad, 6 * 20 + 2 * pad
    img = np.full((h, w), 254, np.uint8)
    cv2.rectangle(img, (pad, pad), (w - pad - 1, h - pad - 1), 0, 1)   # boundary
    x = pad + 2 * 20                                                  # wall east of col 1
    row0_top = h - pad - 20                                           # image rows flip y
    cv2.line(img, (x, row0_top), (x, h - pad), 0, 1)                  # only beside row 0
    cv2.imwrite(str(tmp_path / 'm.pgm'), img)
    (tmp_path / 'm.yaml').write_text(
        'image: m.pgm\nresolution: 0.05\norigin: [-0.5, -0.5, 0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    g = MazeGrid.from_dict({'rows': 3, 'cols': 6, 'cell_size': 1.0})
    extract_walls(g, tmp_path / 'm.yaml')
    assert g.to_dict()['walls'] == [[0, 1, 'E']]


def test_raycast_hits_first_wall_on_the_cameras_side():
    g = MazeGrid.from_dict({'rows': 3, 'cols': 6, 'cell_size': 1.0, 'walls': [[0, 1, 'E']],
                            'origin': [-1.0, 2.0]})
    ox, oy = -1.0, 2.0
    # Camera at (0,0) center looking E: wall E of (0,1), 1.5 m ahead.
    hit = raycast(g, ox + 0.5, oy + 0.5, 0.0)
    assert hit.face == CellPose(Cell(0, 1), Heading.EAST)
    assert abs(hit.distance - 1.5) < 1e-9 and abs(hit.lateral) < 1e-9
    # Looking W from (0,3): same wall, other side.
    assert raycast(g, ox + 3.5, oy + 0.5, math.pi).face == CellPose(Cell(0, 2), Heading.WEST)
    # Diagonal NE from (0,0): passes (1,0)/(1,1)... ends on the north boundary.
    hit = raycast(g, ox + 0.5, oy + 0.5, math.atan2(1.0, 0.8))
    assert hit.face.heading == Heading.NORTH and hit.face.cell.row == 2
    # Right of the robot (south boundary), and out of range / outside the maze.
    assert raycast(g, ox + 0.5, oy + 0.5, -math.pi / 2).face == CellPose(Cell(0, 0), Heading.SOUTH)
    assert raycast(g, ox + 0.5, oy + 0.5, 0.0, max_range=1.0) is None
    assert raycast(g, ox - 0.5, oy + 0.5, 0.0) is None


def test_camera_offset_and_lidar_check():
    x, y, yaw = camera_pose(1.0, 1.0, math.pi / 2, 0.07, 0.0)      # facing N
    assert abs(x - 1.0) < 1e-9 and abs(y - 1.07) < 1e-9 and yaw == math.pi / 2
    g = MazeGrid.from_dict({'rows': 3, 'cols': 6, 'cell_size': 1.0})
    hit = raycast(g, 0.57, 0.5, 0.0)                                 # E boundary, 5.43 m
    assert lidar_agrees(hit, 0.47, 0.5, 0.0, lambda b: 5.53 if abs(b) < 1e-6 else None)
    assert not lidar_agrees(hit, 0.47, 0.5, 0.0, lambda b: 1.0)      # something in the way
    assert lidar_agrees(hit, 0.47, 0.5, 0.0, lambda b: None)


def test_fit_grid_on_synthetic_map(tmp_path):
    res, pad = 0.05, 10
    h, w = 3 * 20 + 2 * pad, 6 * 20 + 2 * pad
    img = np.full((h, w), 254, np.uint8)
    cv2.rectangle(img, (pad, pad), (w - pad - 1, h - pad - 1), 0, 1)
    cv2.imwrite(str(tmp_path / 'm.pgm'), img)
    (tmp_path / 'm.yaml').write_text(
        'image: m.pgm\nresolution: 0.05\norigin: [-0.5, -0.5, 0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    from team85_final.maze.extract import fit_grid
    g = fit_grid(tmp_path / 'm.yaml', 3, 6)
    assert abs(g['cell_size'] - (129 - 10) * 0.05 / 6) < 0.01     # walls at px 10 and 129
    assert abs(g['origin'][0] - (-0.5 + 10.5 * 0.05)) < 1e-6
    assert g['fit']['aspect_error'] < 0.05


def test_dfs_walk_covers_all_cells_through_open_edges():
    # dfs_walk lives in a node module that imports rclpy; load just the function.
    import ast
    src = (Path(__file__).parents[1] / 'team85_final' / 'nodes' / 'drive_cells.py').read_text()
    ns = {}
    tree = ast.parse(src)
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'dfs_walk')
    exec(compile(ast.Module([fn], []), 'drive_cells', 'exec'),
         {'MazeGrid': MazeGrid, 'Cell': Cell, 'Heading': Heading}, ns)
    g = MazeGrid.from_dict({'rows': 3, 'cols': 6, 'cell_size': 1.0,
                            'walls': [[0, 1, 'E'], [1, 1, 'E'], [1, 3, 'N'], [0, 4, 'N']]})
    walk = ns['dfs_walk'](g, Cell(0, 0))
    assert set(walk) == set(g.cells()) and walk[0] == Cell(0, 0)
    for a, b in zip(walk, walk[1:]):
        h = next(h for h in Heading if a.step(h) == b)
        assert not g.blocked(a, h)
    assert walk[-1] not in walk[:-1]                       # no walking back at the end


def test_calibrated_bearing_matches_pinhole_and_handles_distortion():
    from team85_final.vision.detector import Detection, calibrated_bearing
    hfov, w = 1.0856, 320
    f = (w / 2) / math.tan(hfov / 2)
    K = [f, 0, 160, 0, f, 120, 0, 0, 1]
    for u in (10, 100, 160, 250, 310):
        d = Detection(None, 'x', 1.0, u, 120, 10, 10)
        assert abs(d.bearing_calibrated(K, [0, 0, 0, 0, 0]) - d.bearing(w, hfov)) < 1e-6
    robot_K = [250.81522, 0, 166.29687, 0, 249.06849, 121.61568, 0, 0, 1]
    robot_D = [0.256329, -0.375874, -0.001012, 0.021296, 0.0]
    assert abs(calibrated_bearing(166.29687, 121.61568, robot_K, robot_D)) < 1e-6   # principal point
    assert calibrated_bearing(20, 120, robot_K, robot_D) > 0                          # left = +
