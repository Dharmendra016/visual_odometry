"""
Rolling local evidence grid in the odom frame, with decay.

Each cell holds evidence e in [-1, 1]: obstacle observations push it up (two frames needed to
become occupied), free-ground
observations push it down, and everything decays towards 0 (unknown) with time constant
`tau`. Decay is what removes "ghost trails" left by moving people/carts, and stale
obstacles that are no longer in view are slowly forgotten (they re-appear when seen again).
"""
import math

import cv2
import numpy as np


class LocalMap:

    # hit 0.35 / threshold 0.5: a cell needs obstacle evidence from >= 2 frames. With one
    # hit enough, single-frame pebble mis-detections piled up into walls in the gravel.
    def __init__(self, size_m=10.0, res=0.05, tau=20.0, hit=0.35, miss=0.25, occ_thresh=0.5):
        self.res = res
        self.n = int(round(size_m / res))
        self.e = np.zeros((self.n, self.n), np.float32)     # [iy, ix]
        self.ox = self.oy = None                             # odom coords of cell (0, 0)
        self.tau, self.hit, self.miss, self.occ = tau, hit, miss, occ_thresh
        self.t_last = None
        self.dist = None

    def _idx(self, x, y):
        return (np.floor((x - self.ox) / self.res).astype(int),
                np.floor((y - self.oy) / self.res).astype(int))

    def recenter(self, x, y):
        half = self.n * self.res / 2
        if self.ox is None:
            self.ox, self.oy = x - half, y - half
            return
        cx, cy = self.ox + half, self.oy + half
        if abs(x - cx) < 1.0 and abs(y - cy) < 1.0:
            return
        sx = int(round((x - cx) / self.res))
        sy = int(round((y - cy) / self.res))
        e = np.roll(self.e, (-sy, -sx), axis=(0, 1))
        if sy > 0:
            e[-sy:, :] = 0
        elif sy < 0:
            e[:-sy, :] = 0
        if sx > 0:
            e[:, -sx:] = 0
        elif sx < 0:
            e[:, :-sx] = 0
        self.e = e
        self.ox += sx * self.res
        self.oy += sy * self.res

    def decay(self, t):
        if self.t_last is not None and t > self.t_last:
            self.e *= math.exp(-(t - self.t_last) / self.tau)
        self.t_last = t

    def insert(self, obstacles_xy, free_xy):
        for pts, delta in ((free_xy, -self.miss), (obstacles_xy, self.hit)):
            if len(pts) == 0:
                continue
            ix, iy = self._idx(pts[:, 0], pts[:, 1])
            ok = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
            cells = np.unique(iy[ok] * self.n + ix[ok])
            flat = self.e.reshape(-1)
            flat[cells] = np.clip(flat[cells] + delta, -1.0, 1.0)

    def update_distance(self):
        """Distance (m) from every cell to the nearest occupied cell."""
        free = (self.e <= self.occ).astype(np.uint8)
        self.dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.res
        return self.dist

    def clearance(self, x, y):
        """Distance to nearest obstacle at odom points (outside the grid: 'far')."""
        ix, iy = self._idx(x, y)
        ok = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        out = np.full(np.shape(x), 5.0)
        out[ok] = self.dist[iy[ok], ix[ok]]
        return out

    def occupancy_grid_data(self):
        """int8 data for nav_msgs/OccupancyGrid: -1 unknown, 0..100 evidence."""
        d = np.full(self.e.shape, -1, np.int8)
        known = np.abs(self.e) > 0.05
        d[known] = np.clip((self.e[known] + 1) * 50, 0, 100).astype(np.int8)
        return d
