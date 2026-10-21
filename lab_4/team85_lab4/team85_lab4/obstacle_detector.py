from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from team85_lab4.path_zone import HEADING, in_path_zone
from team85_lab4.scan_segmentation import Segment


@dataclass
class Obstacle:
    """
    Closest obstacle point, in the scan frame (x: forward, y: left).

    Attributes:
        point: [x, y] vector from the LiDAR to the closest point (meters).
        normal: Unit [x, y] normal of the segment at that point, pointing
                towards the LiDAR side of the surface.
        distance: Distance from the LiDAR to the point (meters).
        segment: The segment containing the point.
        index: Index of the point within the segment.
        at_end: Whether the point is the first or last point of the segment,
                so the normal was fitted from one side of the surface only.
    """
    point: np.ndarray
    normal: np.ndarray
    distance: float
    segment: Segment
    index: int
    at_end: bool


class ObstacleDetector:
    """
    Find the closest point of all segments within the path zone, and
    approximate the surface normal there.

    The path zone is the strip ahead along the path direction (towards the
    goal; the robot's heading by default), within path_radius of the path
    line through the robot, and short of the goal (path_length): what the
    robot would hit on its way to the goal.

    The normal is the direction of least variance (PCA) of the segment's
    points within normal_radius of the closest point, using at least
    min_normal_points points. Neighbors outside the path zone are still
    used, since they describe the same surface.

    If the closest point is at an end of its segment (e.g. a surface split by
    the segmentation), its neighbors only exist on one side. The fit then
    takes them all from that side, which still gives the surface direction.
    """

    def __init__(self, path_radius=np.inf, normal_radius=0.1, min_normal_points=5):
        """
        Args:
            path_radius: Half width (meters) of the path zone. Points behind the
                         robot along the path, or farther than this from the
                         path line, are ignored.
            normal_radius: Radius (meters) of the neighborhood used to fit the normal.
            min_normal_points: Minimum points (including the closest point) used
                               to fit the normal, taken along the segment when
                               normal_radius holds too few, e.g. far away or at
                               a segment end.
        """
        self.path_radius = path_radius
        self.normal_radius = normal_radius
        self.min_normal_points = min_normal_points

    def detect(
        self,
        segments: List[Segment],
        direction: np.ndarray = HEADING,
        path_length: float = np.inf,
    ) -> Optional[Obstacle]:
        """
        Args:
            segments: Segments of the scan.
            direction: Unit path direction in the scan frame.
            path_length: Length of the path zone along direction (the goal distance).

        Returns:
            The closest obstacle, or None if no segment point is in the path zone.
        """
        best = None
        for segment in segments:
            in_zone = in_path_zone(segment.points, direction, self.path_radius, path_length)
            if not in_zone.any():
                continue

            ranges = np.where(in_zone, segment.ranges, np.inf)
            index = int(np.argmin(ranges))
            if best is None or ranges[index] < best[0]:
                best = (ranges[index], segment, index)

        if best is None:
            return None

        distance, segment, index = best
        point = segment.points[index]
        return Obstacle(
            point=point,
            normal=self.estimate_normal(segment.points, index),
            distance=float(distance),
            segment=segment,
            index=index,
            at_end=index in (0, len(segment) - 1),
        )

    def estimate_normal(self, points: np.ndarray, index: int) -> np.ndarray:
        point = points[index]

        # Facing the LiDAR; also the fallback when the surface can't be fitted
        towards_lidar = -point / np.linalg.norm(point)

        # Segment points ordered by how far along the segment they are from
        # the closest point; at a segment end they all come from one side
        order = np.argsort(np.abs(np.arange(len(points)) - index), kind="stable")
        within = np.linalg.norm(points[order] - point, axis=1) <= self.normal_radius
        # Keep only the leading run within the radius, then enforce the minimum
        run = len(within) if within.all() else int(np.argmin(within))
        count = max(run, self.min_normal_points)
        neighbors = points[order[:count]]
        if len(neighbors) < 2:
            return towards_lidar

        # Eigenvector of the smallest eigenvalue is perpendicular to the fitted line
        centered = neighbors - neighbors.mean(axis=0)
        _, eigenvectors = np.linalg.eigh(centered.T @ centered)
        normal = eigenvectors[:, 0]

        # Point the normal towards the LiDAR
        if normal @ towards_lidar < 0:
            normal = -normal
        return normal
