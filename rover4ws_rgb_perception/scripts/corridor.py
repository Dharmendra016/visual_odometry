"""
Corridor and vanishing-point estimation from the free-space result.

Corridor: a polar histogram of free range per azimuth (from the column scan). A min-filter
as wide as the robot at ~2.5 m turns it into "how far can the robot go in this direction";
the corridor heading is the most open direction (ties broken towards straight ahead).

Vanishing point: straight lines fitted to the left and right edges of the drivable region
(path / sidewalk borders); their intersection is the vanishing point. In open fields the
drivable region touches the image borders, no edges are found and confidence is 0.
"""
import math

import cv2
import numpy as np


def corridor(col_range, col_az, fov_deg=50.0, bin_deg=2.0, robot_halfwidth=0.35):
    edges = np.arange(-fov_deg, fov_deg + bin_deg, bin_deg)
    centers = np.radians(0.5 * (edges[:-1] + edges[1:]))
    hist = np.full(len(centers), np.nan)
    ok = np.isfinite(col_range) & np.isfinite(col_az)
    idx = np.digitize(np.degrees(col_az[ok]), edges) - 1
    for i, r in zip(idx, col_range[ok]):
        if 0 <= i < len(hist):
            hist[i] = r if np.isnan(hist[i]) else min(hist[i], r)
    if np.all(np.isnan(hist)):
        return None
    h = np.nan_to_num(hist, nan=0.0)
    half = max(1, int(round(math.degrees(math.atan2(robot_halfwidth, 2.5)) / bin_deg)))
    clear = np.array([h[max(0, i - half):i + half + 1].min() for i in range(len(h))])
    best = clear.max()
    if best <= 0.0:
        return {'heading': 0.0, 'range': 0.0, 'width': 0.0, 'clear': clear, 'centers': centers}
    cand = np.nonzero(clear >= best - 1e-3)[0]
    i = cand[np.argmin(np.abs(centers[cand]))]
    good = clear >= 0.8 * best
    lo = hi = i
    while lo > 0 and good[lo - 1]:
        lo -= 1
    while hi < len(good) - 1 and good[hi + 1]:
        hi += 1
    return {'heading': float(centers[i]), 'range': float(best),
            'width': float(centers[hi] - centers[lo] + math.radians(bin_deg)),
            'clear': clear, 'centers': centers}


def vanishing_point(drivable, horizon_row, min_rows=15):
    H, W = drivable.shape
    left, right = [], []
    for r in range(max(horizon_row + 3, 0), H):
        xs = np.flatnonzero(drivable[r])
        if len(xs) < 5:
            continue
        if xs[0] > 2:
            left.append((xs[0], r))
        if xs[-1] < W - 3:
            right.append((xs[-1], r))
    n_rows = max(1, H - horizon_row)
    if len(left) < min_rows or len(right) < min_rows:
        return None, 0.0
    lines = []
    for pts in (left, right):
        vx, vy, x0, y0 = cv2.fitLine(np.array(pts, np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
        lines.append((vx, vy, x0, y0))
    (ax, ay, ax0, ay0), (bx, by, bx0, by0) = lines
    den = ax * by - ay * bx
    if abs(den) < 1e-6:
        return None, 0.0
    t = ((bx0 - ax0) * by - (by0 - ay0) * bx) / den
    u, v = ax0 + t * ax, ay0 + t * ay
    if not (-W <= u <= 2 * W and -H <= v <= H):
        return None, 0.0
    conf = min(len(left), len(right)) / n_rows
    return (float(u), float(v)), float(min(1.0, conf))
