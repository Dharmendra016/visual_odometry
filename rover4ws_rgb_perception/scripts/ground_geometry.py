"""
Flat-ground camera geometry ("inverse perspective mapping").

With a calibrated camera at a known pose above flat ground, every pixel below the horizon
maps to exactly one ground point - no depth sensor or depth network needed. This is the
metric backbone of the RGB-only pipeline: free-space boundaries, obstacle heights,
object positions and visual odometry are all expressed through it.

Frames: camera_optical_frame (z forward, x right, y down) -> base_footprint (on the ground).
"""
import numpy as np


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class GroundGeometry:
    """Pixel <-> ground-plane mapping at one image resolution."""

    def __init__(self, K, R, t, width, height):
        self.K = np.asarray(K, float)
        self.R = np.asarray(R, float)       # optical -> base rotation
        self.C = np.asarray(t, float)       # camera centre in base frame
        self.W, self.H = width, height
        u, v = np.meshgrid(np.arange(width) + 0.0, np.arange(height) + 0.0)
        self.ray = self.rays(u.ravel(), v.ravel()).reshape(height, width, 3)
        rz = self.ray[..., 2]
        self.valid = rz < -1e-6
        s = np.where(self.valid, -self.C[2] / np.where(self.valid, rz, -1.0), np.nan)
        self.GX = self.C[0] + s * self.ray[..., 0]
        self.GY = self.C[1] + s * self.ray[..., 1]
        self.dist = np.hypot(self.GX, self.GY)                       # from base origin
        self.cam_dist = np.hypot(self.GX - self.C[0], self.GY - self.C[1])
        self.azimuth = np.arctan2(self.GY, self.GX)

    @property
    def camera_height(self):
        return self.C[2]

    def rays(self, u, v):
        """Base-frame ray directions (not normalised) for pixels (u, v)."""
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        d = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], -1)
        return d @ self.R.T

    def ground_points(self, u, v):
        """Ground (z = 0) points for sub-pixel coordinates; returns (N, 2) and valid mask."""
        r = self.rays(np.asarray(u, float), np.asarray(v, float))
        ok = r[:, 2] < -1e-6
        s = np.where(ok, -self.C[2] / np.where(ok, r[:, 2], -1.0), np.nan)
        return np.stack([self.C[0] + s * r[:, 0], self.C[1] + s * r[:, 1]], 1), ok

    def project(self, P):
        """Project base-frame 3D points (N, 3) to pixels; returns (N, 2) and in-front mask."""
        Pc = (np.asarray(P, float) - self.C) @ self.R          # base -> optical
        ok = Pc[:, 2] > 1e-3
        z = np.where(ok, Pc[:, 2], 1.0)
        u = self.K[0, 0] * Pc[:, 0] / z + self.K[0, 2]
        v = self.K[1, 1] * Pc[:, 1] / z + self.K[1, 2]
        return np.stack([u, v], 1), ok

    def height_at(self, rows_top, cols, contact_cam_dist):
        """
        Height of the point seen at (rows_top, cols), assuming it lies on a vertical surface
        standing at horizontal distance `contact_cam_dist` from the camera.
        """
        r = self.ray[rows_top, cols]
        horiz = np.hypot(r[..., 0], r[..., 1])
        s = contact_cam_dist / np.maximum(horiz, 1e-9)
        return self.C[2] + s * r[..., 2]
