from dataclasses import dataclass, field
from typing import List

import numpy as np


@dataclass
class Segment:
    """
    One segment of a LiDAR scan.

    Attributes:
        indices: Indices of the segment's points in the original scan,
                 ordered counterclockwise (may wrap from the end to the start).
        angles: Scan angles (radians) of the points.
        ranges: Distances (meters) of the points.
        points: Mx2 array of [x, y] in the scan frame (x: forward, y: left).
    """
    indices: np.ndarray
    angles: np.ndarray
    ranges: np.ndarray
    points: np.ndarray = field(init=False)

    def __post_init__(self):
        self.points = np.stack(
            [self.ranges * np.cos(self.angles), self.ranges * np.sin(self.angles)],
            axis=1,
        )

    def __len__(self):
        return len(self.indices)


class ScanSegmenter:
    """
    Segment an ordered 2D LiDAR scan.

    Two neighboring points belong to the same segment if both are valid and
    their Euclidean distance is within an adaptive threshold:

        base_threshold + k * min(r, prev_r) * |delta_theta|
    """

    def __init__(self, min_points=3, base_threshold=0.05, k=2.0, wrap_around=True):
        """
        Args:
            min_points: Minimum points required for a segment.
            base_threshold: Baseline distance threshold (meters).
            k: Adaptive threshold scaling factor.
            wrap_around: Join the last and first points of a full 360 degree
                         scan, so an object crossing the scan's 0 angle is
                         kept as one segment.
        """
        self.min_points = min_points
        self.base_threshold = base_threshold
        self.k = k
        self.wrap_around = wrap_around

    def segment(self, scan) -> List[Segment]:
        """
        Segment a sensor_msgs/msg/LaserScan message.

        Returns:
            List of segments, ordered by their first index in the scan.
        """
        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(ranges.shape[0]) * scan.angle_increment

        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        coverage = ranges.shape[0] * abs(scan.angle_increment)
        full_circle = coverage >= 2 * np.pi - 1.5 * abs(scan.angle_increment)

        return self.segment_polar(angles, ranges, valid, full_circle)

    def segment_polar(self, angles, ranges, valid=None, full_circle=False) -> List[Segment]:
        """
        Segment a scan given as ordered polar points.

        Args:
            angles: N scan angles (radians), ordered.
            ranges: N distances (meters).
            valid: Optional N boolean mask; defaults to finite, positive ranges.
            full_circle: Whether the scan covers 360 degrees (enables wrap-around).

        Returns:
            List of segments, ordered by their first index in the scan.
        """
        angles = np.asarray(angles, dtype=float)
        ranges = np.asarray(ranges, dtype=float)
        n = ranges.shape[0]
        if valid is None:
            valid = np.isfinite(ranges) & (ranges > 0)
        if n == 0:
            return []

        # connected[i]: point i continues the segment of point i - 1
        connected = np.zeros(n, dtype=bool)
        connected[1:] = self._close(
            ranges[:-1], ranges[1:], np.diff(angles)
        ) & valid[:-1] & valid[1:]

        # Label runs of connected valid points
        starts = valid & ~connected
        labels = np.where(valid, np.cumsum(starts) - 1, -1)
        num_runs = int(starts.sum())
        if num_runs == 0:
            return []

        runs = [np.flatnonzero(labels == i) for i in range(num_runs)]

        # Join the last run with the first one across the scan's end
        if self.wrap_around and full_circle and num_runs > 1:
            delta = angles[0] + 2 * np.pi - angles[-1]
            if (valid[0] and valid[-1]
                    and self._close(ranges[-1], ranges[0], delta)):
                runs[0] = np.concatenate([runs[-1], runs[0]])
                runs.pop()

        return [
            Segment(indices=run, angles=angles[run], ranges=ranges[run])
            for run in runs if len(run) >= self.min_points
        ]

    def _close(self, prev_r, r, delta_theta):
        # Euclidean distance in polar coordinates
        # (invalid ranges give nan/inf here; they are masked out by the caller)
        with np.errstate(invalid="ignore"):
            d = np.sqrt((r - prev_r) ** 2 + 2 * r * prev_r * (1 - np.cos(delta_theta)))

        # Adaptive threshold
        threshold = self.base_threshold + self.k * np.minimum(r, prev_r) * np.abs(delta_theta)

        return d <= threshold
