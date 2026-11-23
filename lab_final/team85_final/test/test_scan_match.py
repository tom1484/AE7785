import math
import os

import numpy as np
import pytest

from team85_final.maze.geometry import Cell, CellPose, Heading
from team85_final.maze.grid import MazeGrid
from team85_final.maze.scan_match import MapMatcher, icp, localize, scan_points

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP = os.path.join(PKG, 'maps', 'map.yaml')


def fake_scan(m: MapMatcher, pose, n=360, rmax=3.5, noise=0.01, seed=0):
    """Ray-march the map's occupied pixels from pose (robot frame = laser frame here)."""
    rng = np.random.default_rng(seed)
    occ = np.zeros((m.h, m.w), bool)
    occ[m.nearest_px[..., 1].astype(int), m.nearest_px[..., 0].astype(int)] = True
    ranges = np.full(n, np.inf)
    for i in range(n):
        a = pose[2] + 2 * math.pi * i / n
        for r in np.arange(0.12, rmax, m.res / 2):
            x, y = pose[0] + r * math.cos(a), pose[1] + r * math.sin(a)
            col = int((x - m.origin[0]) / m.res)
            row = m.h - 1 - int((y - m.origin[1]) / m.res)
            if 0 <= col < m.w and 0 <= row < m.h and occ[row, col]:
                ranges[i] = r + rng.normal(0, noise)
                break
    return scan_points(ranges, 0.0, 2 * math.pi / n)


@pytest.fixture(scope='module')
def matcher():
    return MapMatcher.from_yaml(MAP)


@pytest.mark.parametrize('cell,heading,err', [
    ((0, 5), 'N', (0.12, -0.08, math.radians(12))),
    ((0, 0), 'E', (-0.10, 0.10, math.radians(-15))),
    ((2, 3), 'W', (0.05, 0.15, math.radians(8))),
])
def test_localize_recovers_hand_placement(matcher, cell, heading, err):
    grid = MazeGrid.from_yaml(os.path.join(PKG, 'config', 'maze.yaml'))
    guess = grid.pose_to_map(CellPose(Cell(*cell), Heading.parse(heading)))
    truth = (guess[0] + err[0], guess[1] + err[1], guess[2] + err[2])
    scan = fake_scan(matcher, truth)
    f = localize(matcher, scan, guess)
    assert math.hypot(f.x - truth[0], f.y - truth[1]) < 0.03
    assert abs(math.remainder(f.yaw - truth[2], math.tau)) < math.radians(1.5)
    assert f.inliers > 0.8


def test_icp_exact_guess_stays(matcher):
    pose = (1.0, -1.2, 0.3)
    f = icp(matcher, fake_scan(matcher, pose, noise=0.0), pose)
    assert math.hypot(f.x - pose[0], f.y - pose[1]) < 0.02
