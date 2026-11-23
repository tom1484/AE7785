"""Find the maze's interior walls in a saved SLAM map (map.yaml + .pgm).

The grid geometry (rows, cols, cell_size, origin, yaw) is given; this only decides,
for every edge between two neighbouring cells, whether the map shows a wall there.
Output is a starting point: always check the result against the real maze
(`extract_maze --render`) before using it.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

from .geometry import Cell, Heading
from .grid import MazeGrid


def load_occupancy(map_yaml: str | Path) -> tuple[np.ndarray, float, tuple[float, float]]:
    """(occupied bool image with row 0 = lowest y, resolution, origin xy) from a map.yaml."""
    import cv2

    map_yaml = Path(map_yaml)
    meta = yaml.safe_load(map_yaml.read_text())
    img = cv2.imread(str(map_yaml.parent / meta['image']), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(meta['image'])
    p = img.astype(np.float32) / 255.0
    occ_prob = p if meta.get('negate', 0) else 1.0 - p
    occupied = occ_prob > float(meta.get('occupied_thresh', 0.65))
    return occupied[::-1], float(meta['resolution']), tuple(meta['origin'][:2])


def fit_grid(map_yaml: str | Path, rows: int, cols: int, min_frac: float = 0.5) -> dict:
    """Grid geometry from the outer walls of an axis-aligned maze map.

    Occupied-pixel profiles along x and y peak at walls; the outermost peaks that reach
    `min_frac` of the strongest one are the outer walls (centerlines). Returns
    {rows, cols, cell_size, origin, yaw: 0, fit: {...}} with origin = the corner of cell
    (0, 0) at the low-x, low-y outer wall. The maze must be roughly aligned with the map
    axes (true when mapping starts square to the maze); check `fit.aspect_error`.
    """
    occupied, res, origin = load_occupancy(map_yaml)
    occupied = largest_structure(occupied)

    def outer(profile: np.ndarray) -> tuple[float, float]:
        idx = np.flatnonzero(profile >= min_frac * profile.max())
        lo = _peak_center(profile, idx[0])
        hi = _peak_center(profile, idx[-1])
        return lo, hi

    x0, x1 = outer(occupied.sum(axis=0).astype(float))     # columns -> x
    y0, y1 = outer(occupied.sum(axis=1).astype(float))     # rows (y up) -> y
    sx, sy = (x1 - x0) * res / cols, (y1 - y0) * res / rows
    cell = (sx + sy) / 2
    return {
        'rows': rows, 'cols': cols, 'cell_size': round(cell, 4),
        'origin': [round(origin[0] + (x0 + 0.5) * res, 4), round(origin[1] + (y0 + 0.5) * res, 4)],
        'yaw': 0.0,
        'fit': {'cell_x': round(sx, 4), 'cell_y': round(sy, 4),
                'aspect_error': round(abs(sx - sy) / cell, 4)},
    }


def largest_structure(occupied: np.ndarray, bridge_px: int = 2) -> np.ndarray:
    """Occupied pixels of the largest connected wall structure (the maze itself).

    Walls seen outside the maze (e.g. the room, through a gap) are separate structures and
    would otherwise be taken for outer walls. Small gaps in the maze walls are bridged by a
    dilation of `bridge_px` before labeling.
    """
    import cv2

    k = np.ones((2 * bridge_px + 1, 2 * bridge_px + 1), np.uint8)
    grown = cv2.dilate(occupied.astype(np.uint8), k)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(grown, connectivity=8)
    if n <= 1:
        return occupied
    big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return occupied & (lab == big)


def _peak_center(profile: np.ndarray, i: int) -> float:
    """Center (fractional index) of the contiguous high run containing index i."""
    thr = 0.5 * profile[i]
    lo = hi = i
    while lo > 0 and profile[lo - 1] >= thr:
        lo -= 1
    while hi < len(profile) - 1 and profile[hi + 1] >= thr:
        hi += 1
    w = profile[lo:hi + 1]
    return float((np.arange(lo, hi + 1) * w).sum() / w.sum())


def edge_occupancy(grid: MazeGrid, occupied: np.ndarray, resolution: float,
                   map_origin: tuple[float, float], cell: Cell, heading: Heading,
                   margin: float = 0.2, band: float = 0.08) -> float:
    """Fraction of samples along the edge on side `heading` of `cell` that hit occupied pixels.

    Samples the middle (1 - 2*margin) of the edge; each sample counts as a hit if any
    pixel within `band` m across the edge is occupied (tolerates small map misalignment).
    """
    s = grid.cell_size
    # Edge endpoints in maze coordinates.
    x0, y0 = cell.col * s, cell.row * s
    if heading == Heading.EAST:
        a, b = (x0 + s, y0), (x0 + s, y0 + s)
    elif heading == Heading.WEST:
        a, b = (x0, y0), (x0, y0 + s)
    elif heading == Heading.NORTH:
        a, b = (x0, y0 + s), (x0 + s, y0 + s)
    else:
        a, b = (x0, y0), (x0 + s, y0)
    nx, ny = (1.0, 0.0) if heading in (Heading.EAST, Heading.WEST) else (0.0, 1.0)

    n = max(4, int(s * (1 - 2 * margin) / resolution))
    hits = 0
    for t in np.linspace(margin, 1 - margin, n):
        mx, my = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
        hit = False
        for off in np.arange(-band, band + 1e-9, resolution):
            x, y = grid._to_map(mx + off * nx, my + off * ny)
            i = math.floor((y - map_origin[1]) / resolution)
            j = math.floor((x - map_origin[0]) / resolution)
            if 0 <= i < occupied.shape[0] and 0 <= j < occupied.shape[1] and occupied[i, j]:
                hit = True
                break
        hits += hit
    return hits / n


def extract_walls(grid: MazeGrid, map_yaml: str | Path, threshold: float = 0.5,
                  **kwargs) -> dict[tuple[Cell, Heading], float]:
    """Add every interior edge with occupancy >= threshold as a wall; return all scores."""
    occupied, res, origin = load_occupancy(map_yaml)
    scores = {}
    for cell in grid.cells():
        for h in (Heading.EAST, Heading.NORTH):
            if not grid.contains(cell.step(h)):
                continue
            score = edge_occupancy(grid, occupied, res, origin, cell, h, **kwargs)
            scores[(cell, h)] = score
            if score >= threshold:
                grid.add_wall(cell, h)
    return scores
