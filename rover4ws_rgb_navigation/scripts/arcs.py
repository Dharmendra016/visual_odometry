"""
Steering-aware trajectory sampling for the 4WS rover (counter-phase steering).

A candidate is (target curvature, speed). The rollout respects
  * the curvature limit |k| <= 1 / R_min (R_min ~ 0.64 m at +-35 deg, planned with margin),
  * the steering rate: curvature moves from the current value towards the target at
    `k_rate` (1/m per s), like the real steering servos - so a hard turn is not instant,
  * the footprint, approximated by 3 circles along the body.
"""
import numpy as np

# footprint: 3 circles along x (base_footprint), radius covers the 0.58 m wide body
CIRCLE_X = np.array([-0.22, 0.0, 0.22])
CIRCLE_R = 0.33   # 0.31 left the deck corners (0.38, 0.29) 2 cm outside the circles


def rollout(x0, y0, th0, k0, k_targets, speeds, horizon, dt, k_rate):
    """Returns poses (C, N, 3) and curvature profile (C, N) for C = len(k_targets) candidates."""
    C, N = len(k_targets), int(round(horizon / dt))
    x = np.full(C, x0, float)
    y = np.full(C, y0, float)
    th = np.full(C, th0, float)
    k = np.full(C, k0, float)
    poses = np.zeros((C, N, 3))
    ks = np.zeros((C, N))
    step = k_rate * dt
    for i in range(N):
        k = k + np.clip(k_targets - k, -step, step)
        dth = speeds * k * dt
        mid = th + 0.5 * dth
        x = x + speeds * np.cos(mid) * dt
        y = y + speeds * np.sin(mid) * dt
        th = th + dth
        poses[:, i] = np.stack([x, y, th], 1)
        ks[:, i] = k
    return poses, ks


def circle_centers(poses):
    """(C, N, 3) poses -> (C, N, 3 circles, 2) centres."""
    c, s = np.cos(poses[..., 2]), np.sin(poses[..., 2])
    cx = poses[..., 0, None] + c[..., None] * CIRCLE_X
    cy = poses[..., 1, None] + s[..., None] * CIRCLE_X
    return np.stack([cx, cy], -1)
