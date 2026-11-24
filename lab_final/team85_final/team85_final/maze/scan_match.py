"""
Scan-to-map matching for the initial pose: the robot is placed by hand in a known cell
facing a known heading (a few cm / degrees off), and ICP against the saved map's
occupied pixels refines that guess before AMCL gets it.

Nearest neighbours come from a distance transform of the map (cv2, O(1) per point),
so there is no scipy/kd-tree. Several starts around the guess (+-yaw, +-xy) guard
against ICP's local minima; the best fit wins only if it is clearly good.

Frames: map points and poses in the map frame; scan points in the robot (base) frame.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

import cv2
import numpy as np
import yaml


@dataclass
class Fit:
    x: float
    y: float
    yaw: float
    rms: float        # m, over inliers
    inliers: float    # fraction of scan points within max_dist of a map wall

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw


class MapMatcher:
    """Occupied pixels of a map_server map (yaml + image) with a nearest-wall lookup."""

    def __init__(self, occupied: np.ndarray, resolution: float, origin: tuple[float, float]):
        # occupied: bool image, row 0 = top (as map_server images are stored)
        self.res = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.h, self.w = occupied.shape
        free = np.where(occupied, 0, 255).astype(np.uint8)
        _, labels = cv2.distanceTransformWithLabels(free, cv2.DIST_L2, 5,
                                                    labelType=cv2.DIST_LABEL_PIXEL)
        # label -> pixel coordinates of that occupied pixel
        ys, xs = np.nonzero(free == 0)
        lut = np.zeros((labels.max() + 1, 2), np.float64)
        lut[labels[ys, xs]] = np.stack([xs, ys], 1)
        self.nearest_px = lut[labels]              # (h, w, 2): nearest occupied (col, row)
        self.n_occupied = len(xs)

    @classmethod
    def from_yaml(cls, map_yaml: str) -> MapMatcher:
        meta = yaml.safe_load(open(map_yaml))
        img = cv2.imread(os.path.join(os.path.dirname(map_yaml), meta['image']),
                         cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(meta['image'])
        p = img.astype(np.float64) / 255.0
        occ = (p if meta.get('negate', 0) else 1.0 - p) > float(meta.get('occupied_thresh', 0.65))
        return cls(occ, meta['resolution'], meta['origin'][:2])

    def nearest(self, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Map-frame points (N,2) -> (nearest wall points (N,2), valid mask)."""
        col = np.floor((pts[:, 0] - self.origin[0]) / self.res).astype(int)
        row = self.h - 1 - np.floor((pts[:, 1] - self.origin[1]) / self.res).astype(int)
        ok = (col >= 0) & (col < self.w) & (row >= 0) & (row < self.h)
        out = np.zeros_like(pts)
        px = self.nearest_px[row[ok], col[ok]]
        out[ok, 0] = self.origin[0] + (px[:, 0] + 0.5) * self.res
        out[ok, 1] = self.origin[1] + (self.h - 1 - px[:, 1] + 0.5) * self.res
        return out, ok


def scan_points(ranges, angle_min: float, angle_increment: float, range_min: float = 0.12,
                range_max: float = 3.5, laser_pose=(0.0, 0.0, 0.0)) -> np.ndarray:
    """LaserScan ranges -> (N,2) points in the robot frame (laser_pose = laser in robot)."""
    r = np.asarray(ranges, np.float64)
    a = angle_min + angle_increment * np.arange(len(r)) + laser_pose[2]
    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    r, a = r[ok], a[ok]
    return np.stack([laser_pose[0] + r * np.cos(a), laser_pose[1] + r * np.sin(a)], 1)


def transform(pts: np.ndarray, pose) -> np.ndarray:
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return pts @ np.array([[c, s], [-s, c]]) + np.array([x, y])


def icp(m: MapMatcher, scan: np.ndarray, guess, iterations: int = 40,
        max_dist: float = 0.25) -> Fit:
    """Point-to-point ICP of robot-frame scan points onto the map, from `guess`."""
    x, y, yaw = guess
    d = max_dist
    for _ in range(iterations):
        world = transform(scan, (x, y, yaw))
        near, ok = m.nearest(world)
        ok &= np.linalg.norm(world - near, axis=1) < d
        if ok.sum() < 10:
            break
        p, q = world[ok], near[ok]
        pc, qc = p.mean(0), q.mean(0)
        H = (p - pc).T @ (q - qc)
        dyaw = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        c, s = math.cos(dyaw), math.sin(dyaw)
        R = np.array([[c, -s], [s, c]])
        t = qc - R @ pc
        # compose: new = R * (old pose) + t
        x, y = R @ np.array([x, y]) + t
        yaw = math.atan2(math.sin(yaw + dyaw), math.cos(yaw + dyaw))
        d = max(0.08, d * 0.85)                 # tighten the gate as it converges
        if abs(dyaw) < 1e-4 and np.hypot(*t) < 1e-4:
            break
    world = transform(scan, (x, y, yaw))
    near, ok = m.nearest(world)
    err = np.linalg.norm(world - near, axis=1)
    inl = ok & (err < 0.08)
    rms = float(np.sqrt(np.mean(err[inl] ** 2))) if inl.any() else float('inf')
    return Fit(float(x), float(y), float(yaw), rms, float(inl.sum()) / max(1, len(scan)))


def localize(m: MapMatcher, scan: np.ndarray, guess, yaw_span: float = math.radians(20),
             xy_span: float = 0.2) -> Fit:
    """Best ICP fit over starts around `guess` (yaw +-yaw_span, xy +-xy_span)."""
    best = None
    for dyaw in np.linspace(-yaw_span, yaw_span, 5):
        for dx in (-xy_span, 0.0, xy_span):
            for dy in (-xy_span, 0.0, xy_span):
                f = icp(m, scan, (guess[0] + dx, guess[1] + dy, guess[2] + dyaw))
                if best is None or (f.inliers, -f.rms) > (best.inliers, -best.rms):
                    best = f
    return best
