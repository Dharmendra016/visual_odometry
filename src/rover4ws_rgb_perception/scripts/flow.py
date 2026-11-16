"""
Optical flow motion cues.

1. Ground-plane visual odometry (no depth needed):
   corners on *drivable ground* are tracked with pyramidal Lucas-Kanade; both ends of each
   track are projected onto the flat ground (GroundGeometry). The previous and current
   ground points of a static ground feature are related by a 2D rigid transform - the
   robot's motion. RANSAC (2-point) + least squares gives (dx, dy, dyaw).

2. Independent-motion cue per detected object:
   if an object standing on the ground were static, its ground-contact pixel would move
   exactly as predicted by the ego-motion. The residual between the measured flow inside
   the object's box and that prediction (pixels/frame) says whether it moves by itself.
"""
import math

import cv2
import numpy as np

LK = dict(winSize=(21, 21), maxLevel=3,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))


def rigid_2d(A, B):
    """Least-squares R, t with A ~ R @ B + t (2D Kabsch)."""
    ca, cb = A.mean(0), B.mean(0)
    H = (B - cb).T @ (A - ca)
    th = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    return R, ca - R @ cb, th


class GroundFlow:

    def __init__(self, max_features=400, vo_range=4.0, inlier_tol=0.04, min_inliers=25,
                 min_ratio=0.4):
        self.max_features = max_features
        self.vo_range = vo_range
        self.tol = inlier_tol
        self.min_inliers = min_inliers
        self.min_ratio = min_ratio
        self.rng = np.random.default_rng(0)
        self.last = None   # (p0, p1) tracked pairs of the last update
        self.motion = None  # (R, t) prev_base <- curr_base

    def update(self, g0, g1, ground_mask0, box_mask0, geom):
        """g0, g1: previous / current grey images. Returns VO dict (ok, dx, dy, dyaw, ...)."""
        self.last, self.motion = None, None
        feat_mask = ((ground_mask0 & (geom.cam_dist <= self.vo_range)) | box_mask0).astype(np.uint8) * 255
        p0 = cv2.goodFeaturesToTrack(g0, self.max_features, 0.01, 5, mask=feat_mask, blockSize=5)
        res = {'ok': False, 'tracked': 0, 'inliers': 0}
        if p0 is None or len(p0) < self.min_inliers:
            return res
        p1, st, _ = cv2.calcOpticalFlowPyrLK(g0, g1, p0, None, **LK)
        p0r, st2, _ = cv2.calcOpticalFlowPyrLK(g1, g0, p1, None, **LK)
        fb = np.linalg.norm((p0 - p0r).reshape(-1, 2), axis=1)
        good = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < 1.0)
        p0, p1 = p0.reshape(-1, 2)[good], p1.reshape(-1, 2)[good]
        self.last = (p0, p1)
        # VO on ground features only
        u0 = np.clip(p0[:, 0].round().astype(int), 0, geom.W - 1)
        v0 = np.clip(p0[:, 1].round().astype(int), 0, geom.H - 1)
        on_ground = ground_mask0[v0, u0] & (geom.cam_dist[v0, u0] <= self.vo_range)
        q0, q1 = p0[on_ground], p1[on_ground]
        res['tracked'] = len(q0)
        if len(q0) < self.min_inliers:
            return res
        A, ok0 = geom.ground_points(q0[:, 0], q0[:, 1])
        B, ok1 = geom.ground_points(q1[:, 0], q1[:, 1])
        ok = ok0 & ok1
        A, B = A[ok], B[ok]
        if len(A) < self.min_inliers:
            return res
        best, best_in = 0, None
        for _ in range(80):
            i, j = self.rng.choice(len(A), 2, replace=False)
            if np.linalg.norm(B[i] - B[j]) < 0.1:
                continue
            R, t, _ = rigid_2d(A[[i, j]], B[[i, j]])
            inl = np.linalg.norm(A - (B @ R.T + t), axis=1) < self.tol
            n = int(inl.sum())
            if n > best:
                best, best_in = n, inl
        if best_in is None or best < self.min_inliers or best / len(A) < self.min_ratio:
            res['inliers'] = best
            return res
        R, t, th = rigid_2d(A[best_in], B[best_in])
        self.motion = (R, t)
        res.update(ok=True, dx=float(t[0]), dy=float(t[1]), dyaw=float(th), inliers=best)
        return res

    def object_residual(self, box, geom):
        """Flow residual (px/frame) of a box vs. what a static object would show; None if unknown."""
        if self.last is None or self.motion is None:
            return None
        x1, y1, x2, y2 = box
        p0, p1 = self.last
        inside = (p1[:, 0] >= x1) & (p1[:, 0] <= x2) & (p1[:, 1] >= y1 + 0.3 * (y2 - y1)) & (p1[:, 1] <= y2)
        if inside.sum() < 4:
            return None
        measured = np.median(p1[inside] - p0[inside], axis=0)
        # predicted flow of the ground-contact point if the object were static
        cu, cv = 0.5 * (x1 + x2), min(y2, geom.H - 1)
        Gc, ok = geom.ground_points(np.array([cu]), np.array([cv]))
        if not ok[0]:
            return None
        R, t = self.motion
        Gp = R @ Gc[0] + t                   # same world point in the previous base frame
        px, front = geom.project(np.array([[Gp[0], Gp[1], 0.0]]))
        if not front[0]:
            return None
        predicted = np.array([cu, cv]) - px[0]
        return float(np.linalg.norm(measured - predicted))
