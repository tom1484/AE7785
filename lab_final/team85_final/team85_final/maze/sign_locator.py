"""Place a detected sign on a wall face of the grid.

A detection gives a bearing from the camera (bbox center). The maze is known, so the
sign is on the first wall the camera ray hits: cast the ray through the grid from the
camera position (robot pose + camera offset) and take that wall face. The face is named
by the CellPose that looks at it (cell on the camera's side), which is the planner's key.

LIDAR is only a check (`lidar_agrees`): the scan should see the same wall at the same
distance, otherwise something is in the way or the pose is off and the sighting is dropped.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .geometry import Cell, CellPose, Heading
from .grid import MazeGrid


@dataclass(frozen=True)
class RayHit:
    face: CellPose                  # wall face hit, seen from face.cell
    distance: float                 # m from the ray origin
    point: tuple[float, float]      # map-frame xy of the hit
    lateral: float                  # m from the face's center along the wall (signed)


def camera_pose(robot_x: float, robot_y: float, robot_yaw: float,
                offset_x: float, offset_y: float) -> tuple[float, float, float]:
    """Map-frame camera pose from the robot pose and the camera offset in base_link."""
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    return robot_x + c * offset_x - s * offset_y, robot_y + s * offset_x + c * offset_y, robot_yaw


def raycast(grid: MazeGrid, x: float, y: float, angle: float,
            max_range: float = math.inf) -> RayHit | None:
    """First wall face hit by the map-frame ray from (x, y) along `angle`.

    None if the origin is outside the maze or nothing is hit within max_range.
    Grid traversal (Amanatides-Woo) in maze coordinates; walls are cell sides.
    """
    s = grid.cell_size
    mx, my = grid.map_to_maze(x, y)
    a = angle - grid.yaw
    dx, dy = math.cos(a), math.sin(a)
    cell = Cell(math.floor(my / s), math.floor(mx / s))
    if not grid.contains(cell):
        return None

    step_c = 1 if dx > 0 else -1
    step_r = 1 if dy > 0 else -1
    # Ray parameter t (= distance, |d| = 1) of the next vertical / horizontal grid line.
    next_x = (cell.col + (step_c > 0)) * s
    next_y = (cell.row + (step_r > 0)) * s
    t_x = (next_x - mx) / dx if abs(dx) > 1e-12 else math.inf
    t_y = (next_y - my) / dy if abs(dy) > 1e-12 else math.inf
    dt_x = s / abs(dx) if abs(dx) > 1e-12 else math.inf
    dt_y = s / abs(dy) if abs(dy) > 1e-12 else math.inf

    while True:
        if t_x < t_y:
            t, h = t_x, (Heading.EAST if step_c > 0 else Heading.WEST)
        else:
            t, h = t_y, (Heading.NORTH if step_r > 0 else Heading.SOUTH)
        if t > max_range:
            return None
        if grid.blocked(cell, h):            # includes the outer walls
            hx, hy = mx + t * dx, my + t * dy
            if h in (Heading.EAST, Heading.WEST):
                lateral = hy - (cell.row + 0.5) * s
            else:
                lateral = hx - (cell.col + 0.5) * s
            return RayHit(CellPose(cell, h), t, grid._to_map(hx, hy), lateral)
        cell = cell.step(h)
        if h in (Heading.EAST, Heading.WEST):
            t_x += dt_x
        else:
            t_y += dt_y


def lidar_agrees(hit: RayHit, scan_x: float, scan_y: float, scan_yaw: float,
                 range_at, tolerance: float = 0.25) -> bool:
    """Does the LIDAR see a wall at the hit point? `range_at(bearing)` -> m or None.

    The scanner is not at the camera, so the expected range/bearing are recomputed from
    the scanner pose. A missing reading (None) counts as agreement (no evidence against).
    """
    px, py = hit.point
    bearing = math.atan2(py - scan_y, px - scan_x) - scan_yaw
    bearing = math.atan2(math.sin(bearing), math.cos(bearing))
    r = range_at(bearing)
    if r is None:
        return True
    return abs(r - math.hypot(px - scan_x, py - scan_y)) <= tolerance
