"""
Free-space column scan: segmentation mask + flat-ground geometry -> obstacles and free space.

For every image column, scan upward from the bottom:
  * drivable pixels are free ground,
  * a vertical run of non-drivable pixels is a candidate obstacle. Its bottom row gives the
    ground contact distance (flat-ground projection) and its top row gives its height
    (assuming a vertical front face at that distance),
  * runs lower than `min_obstacle_height` (pebbles, segmentation speckle) are drivable -
    the scan continues above them,
  * the first run that is tall enough is the obstacle boundary for that column.
Everything is vectorised across columns (one numpy pass per image row).
"""
import cv2
import numpy as np


def vertical_runs(mask):
    """For each True pixel: row index of the bottom and of the top of its vertical run."""
    H, W = mask.shape
    bottom = np.full((H, W), -1, np.int32)
    top = np.full((H, W), -1, np.int32)
    cur = np.full(W, -1, np.int32)
    for r in range(H - 1, -1, -1):              # bottom-up: run bottom
        cur = np.where(mask[r], np.where(cur >= 0, cur, r), -1)
        bottom[r] = cur
    cur[:] = -1
    for r in range(H):                          # top-down: run top
        cur = np.where(mask[r], np.where(cur >= 0, cur, r), -1)
        top[r] = cur
    return bottom, top


def scan(labels, drivable_lut, sky_lut, geom, max_range, min_height, dyn_boxes=(), object_depth=0.1):
    """
    labels: HxW class ids at the geometry's resolution.
    object_depth: assumed front-to-top depth of an object (m). The top pixel of a low rounded
               object (pebble) lies behind its ground contact; assuming a purely vertical face
               overestimated a 3 cm pebble at 0.3 m as 7 cm. 0.1 m fixes that while tall
               objects (top above the camera) stay tall.
    dyn_boxes: [(x1, y1, x2, y2)] of moving-object classes (same resolution) - columns whose
               boundary falls inside one are reported as dynamic, not static.
    Returns a dict of results (see end of function).
    """
    H, W = labels.shape
    drivable = drivable_lut[labels]
    sky = sky_lut[labels]
    # remove speckle in the drivable mask
    k = np.ones((3, 3), np.uint8)
    drivable = cv2.morphologyEx(drivable.astype(np.uint8), cv2.MORPH_CLOSE, k).astype(bool)

    in_range = geom.valid & (geom.cam_dist <= max_range)
    candidate = ~drivable & ~sky
    bottom, top = vertical_runs(candidate)

    rows = np.arange(H)[:, None].repeat(W, 1)
    cols = np.arange(W)[None, :].repeat(H, 0)
    is_bottom = candidate & (rows == bottom) & in_range
    heights = np.full((H, W), np.nan)
    br, bc = np.nonzero(is_bottom)
    if len(br):
        heights[br, bc] = geom.height_at(top[br, bc], bc, geom.cam_dist[br, bc] + object_depth)
    tall = is_bottom & (heights >= min_height)
    small = is_bottom & (heights < min_height)

    # first (nearest = lowest in image) tall run per column
    obs_row = np.where(tall, rows, -1).max(axis=0)
    has_obs = obs_row >= 0
    oc = np.nonzero(has_obs)[0]
    orow = obs_row[oc]

    dynamic_col = np.zeros(W, bool)
    for (x1, y1, x2, y2) in dyn_boxes:
        sel = (oc >= x1) & (oc <= x2) & (orow >= y1 - 4) & (orow <= y2 + 6)
        dynamic_col[oc[sel]] = True
    static_c = oc[~dynamic_col[oc]]
    static_r = obs_row[static_c]

    free = in_range & (rows > obs_row[None, :])
    # range per column: to the obstacle, or to the farthest visible ground within range
    far_row = np.where(in_range, rows, H).min(axis=0)
    rng_row = np.where(has_obs, obs_row, far_row)
    vis = rng_row < H
    col_range = np.full(W, np.nan)
    col_az = np.full(W, np.nan)
    vc = np.nonzero(vis)[0]
    col_range[vc] = geom.dist[rng_row[vc], vc]
    col_az[vc] = geom.azimuth[rng_row[vc], vc]
    col_range[vc[~has_obs[vc]]] = max_range   # nothing seen up to max range

    return {
        'drivable': drivable,
        'static_pts': np.stack([geom.GX[static_r, static_c], geom.GY[static_r, static_c],
                                heights[static_r, static_c]], 1),
        'static_px': np.stack([static_c, static_r], 1),
        'free_mask': free,
        'small_px': np.stack(np.nonzero(small)[::-1], 1),
        'col_range': col_range,
        'col_az': col_az,
        'obs_row': obs_row,
    }
