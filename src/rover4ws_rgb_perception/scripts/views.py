"""
Visualisation views of the RGB-only perception (all at work resolution, RGB uint8).

  semantic        SegFormer classes, coloured, with a legend of the most frequent classes
  traversability  reachable free ground / drivable-but-blocked / small stones / obstacles /
                  people & vehicles / sky
  obstacles       obstacle instances (connected regions), each with its own colour and distance
  distance        distance heatmap from flat-ground geometry (near = red, far = blue) -
                  no depth network: ground pixels by projection, obstacles by their ground contact
  vanishing       path-edge lines (green left, red right), vanishing point, corridor centre dots,
                  horizon
  detections      YOLO boxes with track id, class, distance and moving flag; horizon; free space
  flow            optical-flow vectors (ground = green, objects = red) and the visual-odometry speed
  dashboard       3 x 2 mosaic of six views
"""
import math

import cv2
import numpy as np

RNG = np.random.default_rng(7)
PALETTE = RNG.integers(40, 255, (256, 3)).astype(np.uint8)

C_FREE = (0, 190, 170)        # teal: reachable free ground
C_DRIVABLE = (60, 120, 60)    # dark green: drivable but behind an obstacle / out of range
C_SMALL = (255, 150, 0)       # orange: small stones (drivable)
C_OBST = (240, 210, 0)        # yellow: obstacles
C_DYN = (230, 0, 200)         # magenta: people / vehicles / animals
C_SKY = (150, 190, 255)
C_OTHER = (230, 120, 120)     # salmon: other non-drivable


def title(img, text):
    cv2.rectangle(img, (0, 0), (img.shape[1], 14), (0, 0, 0), -1)
    cv2.putText(img, text, (3, 11), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def blend(rgb, color_img, mask, alpha=0.55):
    out = rgb.copy()
    out[mask] = (alpha * color_img[mask] + (1 - alpha) * rgb[mask]).astype(np.uint8)
    return out


def class_split_bottom(candidate, labels):
    """
    Bottom row of each pixel's vertical run, with runs also broken where the class changes
    (rock -> wall behind it). Visualisation only: in the navigation scan a tall obstacle whose
    lowest rows carry a different label would otherwise be placed too far away.
    """
    H, W = candidate.shape
    bottom = np.full((H, W), -1, np.int32)
    cur = np.full(W, -1, np.int32)
    same = np.zeros(W, bool)
    for r in range(H - 1, -1, -1):
        if r < H - 1:
            same = labels[r] == labels[r + 1]
        cur = np.where(candidate[r], np.where((cur >= 0) & same, cur, r), -1)
        bottom[r] = cur
    return bottom


def contact_distance(fs, geom, labels):
    """Per-pixel distance of the ground contact of the (class-split) run the pixel belongs to."""
    H, W = labels.shape
    b = class_split_bottom(fs['candidate'], labels)
    cols = np.broadcast_to(np.arange(W), (H, W))
    bc = np.clip(b, 0, H - 1)
    ok = (b >= 0) & geom.valid[bc, cols]
    return np.where(ok, geom.dist[bc, cols], np.nan)


def obstacle_pixels(fs):
    """Pixels belonging to tall obstacle runs (vertical runs whose bottom was judged tall)."""
    cand, bottom, tall = fs['candidate'], fs['run_bottom'], fs['tall_bottom']
    H, W = cand.shape
    cols = np.broadcast_to(np.arange(W), (H, W))
    b = np.clip(bottom, 0, H - 1)
    return cand & (bottom >= 0) & tall[b, cols]


def small_pixels(fs, geom):
    cand, bottom, tall = fs['candidate'], fs['run_bottom'], fs['tall_bottom']
    H, W = cand.shape
    cols = np.broadcast_to(np.arange(W), (H, W))
    b = np.clip(bottom, 0, H - 1)
    in_range = geom.valid[b, cols]
    return cand & (bottom >= 0) & ~tall[b, cols] & in_range


def semantic(rgb, labels, names, top_k=6):
    col = PALETTE[labels % 256]
    out = (0.55 * col + 0.45 * rgb).astype(np.uint8)
    ids, counts = np.unique(labels, return_counts=True)
    order = np.argsort(-counts)[:top_k]
    y = 26
    for i in order:
        c = tuple(int(v) for v in PALETTE[ids[i] % 256])
        cv2.rectangle(out, (4, y - 8), (12, y), c, -1)
        cv2.putText(out, f'{names.get(int(ids[i]), "?")} {100 * counts[i] / labels.size:.0f}%',
                    (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1, cv2.LINE_AA)
        y += 11
    return title(out, 'SEMANTIC SEGMENTATION (SegFormer, ADE20K)')


def traversability(rgb, fs, geom, dyn_boxes):
    H, W = rgb.shape[:2]
    lay = np.zeros_like(rgb)
    mask = np.zeros((H, W), bool)
    obst = obstacle_pixels(fs)
    small = small_pixels(fs, geom)
    for m, c in ((fs['candidate'] & ~obst & ~small, C_OTHER), (fs['drivable'], C_DRIVABLE),
                 (fs['free_mask'], C_FREE), (small, C_SMALL), (obst, C_OBST), (fs['sky'], C_SKY)):
        lay[m] = c
        mask |= m
    for (x1, y1, x2, y2) in dyn_boxes:
        x1, y1, x2, y2 = int(max(x1, 0)), int(max(y1, 0)), int(min(x2, W)), int(min(y2, H))
        sub = fs['candidate'][y1:y2, x1:x2]
        lay[y1:y2, x1:x2][sub] = C_DYN
    out = blend(rgb, lay, mask)
    legend = [('free', C_FREE), ('drivable', C_DRIVABLE), ('stones<6cm', C_SMALL),
              ('obstacle', C_OBST), ('person/vehicle', C_DYN)]
    x = 4
    for name, c in legend:
        cv2.rectangle(out, (x, H - 11), (x + 7, H - 4), c, -1)
        cv2.putText(out, name, (x + 9, H - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1, cv2.LINE_AA)
        x += 12 + 6 * len(name)
    return title(out, 'TRAVERSABILITY (free / drivable / stones / obstacles)')


def distance_map(fs, geom, labels, max_d=10.0):
    """Metric distance per pixel (nan where unknown) from flat-ground geometry."""
    d = np.where(geom.valid & fs['drivable'], geom.dist, np.nan)
    obj = fs['candidate'] & ~fs['sky']
    d = np.where(obj, contact_distance(fs, geom, labels), d)
    d[d > max_d] = np.nan
    return d


def distance(rgb, fs, geom, labels, max_d=10.0):
    d = distance_map(fs, geom, labels, max_d)
    ok = np.isfinite(d)
    norm = np.zeros(d.shape, np.uint8)
    norm[ok] = (255 * (1 - np.clip(d[ok] / max_d, 0, 1))).astype(np.uint8)   # near = high
    heat = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)[..., ::-1]
    out = (0.25 * rgb).astype(np.uint8)
    out[ok] = heat[ok]
    H, W = out.shape[:2]
    bar = cv2.applyColorMap(np.linspace(255, 0, 100).astype(np.uint8)[None, :].repeat(6, 0),
                            cv2.COLORMAP_TURBO)[..., ::-1]
    out[H - 12:H - 6, W - 110:W - 10] = bar
    cv2.putText(out, '0', (W - 114, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
    cv2.putText(out, f'{max_d:.0f}m', (W - 26, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
    return title(out, 'DISTANCE HEATMAP (flat-ground geometry, no depth net)')


def obstacles(rgb, fs, geom, labels):
    """Instances = connected regions of obstacle pixels, separated where the class changes."""
    obst = obstacle_pixels(fs)
    contact = contact_distance(fs, geom, labels)
    out = (0.6 * rgb).astype(np.uint8)
    shown, k = 0, 0
    for cls in np.unique(labels[obst]):
        n, lab, stats, _ = cv2.connectedComponentsWithStats((obst & (labels == cls)).astype(np.uint8),
                                                            connectivity=8)
        for i in range(1, n):
            x, y, w, h, area = stats[i]
            if area < 15:
                continue
            m = lab == i
            dmin = np.nanmin(np.where(m, contact, np.nan)) if np.isfinite(contact[m]).any() else np.nan
            if not np.isfinite(dmin):
                continue                      # no ground contact visible (background, sky line)
            k += 1
            c = np.array(PALETTE[(k * 37) % 256], float)
            out[m] = (0.3 * out[m] + 0.7 * c).astype(np.uint8)
            cv2.putText(out, f'{dmin:.1f}m', (int(x), max(int(y) - 2, 22)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.33, (255, 255, 255), 1, cv2.LINE_AA)
            shown += 1
    return title(out, f'OBSTACLE INSTANCES ({shown})')


def vanishing(rgb, vpinfo, cor, geom, horizon_row):
    out = rgb.copy()
    H, W = out.shape[:2]
    cv2.line(out, (0, horizon_row), (W, horizon_row), (255, 255, 0), 1)
    for pts, c in ((vpinfo.get('left', []), (0, 255, 0)), (vpinfo.get('right', []), (255, 60, 60))):
        for (x, y) in pts[::3]:
            cv2.circle(out, (int(x), int(y)), 1, c, -1)
    vp = vpinfo.get('vp')
    if vp is not None and vpinfo.get('lines'):
        for (vx, vy, x0, y0), c in zip(vpinfo['lines'], ((0, 255, 0), (255, 60, 60))):
            t = (H - 1 - y0) / vy if abs(vy) > 1e-6 else 0
            cv2.line(out, (int(x0 + t * vx), H - 1), (int(vp[0]), int(vp[1])), c, 2, cv2.LINE_AA)
        cv2.circle(out, (int(vp[0]), int(vp[1])), 5, (255, 0, 0), -1)
    if cor:
        pts = np.array([[r * math.cos(cor['heading']), r * math.sin(cor['heading']), 0.0]
                        for r in np.arange(0.6, min(cor['range'], 5.0) + 0.01, 0.5)])
        if len(pts):
            px, ok = geom.project(pts)
            for (u, v), o in zip(px, ok):
                if o:
                    cv2.circle(out, (int(u), int(v)), 3, (0, 255, 0), -1)
        cv2.putText(out, f'corridor {math.degrees(cor["heading"]):+.0f}deg  free {cor["range"]:.1f}m  '
                          f'width {math.degrees(cor["width"]):.0f}deg', (4, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.33, (0, 255, 0), 1, cv2.LINE_AA)
    state = f'VP conf {vpinfo.get("conf", 0):.2f}' if vp else 'no path edges (open ground)'
    cv2.putText(out, state, (4, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (255, 255, 255), 1, cv2.LINE_AA)
    return title(out, 'VANISHING POINT / CORRIDOR')


def detections(rgb, fs, dets, horizon_row):
    H, W = rgb.shape[:2]
    lay = np.zeros_like(rgb)
    lay[fs['free_mask']] = C_FREE
    out = blend(rgb, lay, fs['free_mask'], 0.35)
    cv2.line(out, (0, horizon_row), (W, horizon_row), (255, 230, 0), 1)
    for d in dets:
        x1, y1, x2, y2 = d['wbox'].astype(int)
        moving = d.get('moving', False)
        c = (255, 40, 40) if moving else (C_DYN if d['dynamic'] else (255, 200, 0))
        cv2.rectangle(out, (x1, y1), (x2, y2), c, 2)
        label = (f'#{d["track_id"]} ' if d.get('track_id') else '') + d['cls']
        if d.get('ground') is not None:
            label += f' {math.hypot(*d["ground"]):.1f}m'
        if moving:
            label += ' MOVING'
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.33, 1)
        cv2.rectangle(out, (x1, max(y1 - th - 3, 14)), (x1 + tw + 2, max(y1, th + 17)), c, -1)
        cv2.putText(out, label, (x1 + 1, max(y1 - 2, th + 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.33,
                    (0, 0, 0), 1, cv2.LINE_AA)
    return title(out, f'DETECTIONS + TRACKS (YOLO11n) - {len(dets)} objects')


def flow(rgb, flow_last, box_mask, vo, dt):
    out = (0.7 * rgb).astype(np.uint8)
    if flow_last is not None:
        p0, p1 = flow_last
        for a, b in zip(p0, p1):
            u, v = int(round(a[0])), int(round(a[1]))
            on_obj = 0 <= v < box_mask.shape[0] and 0 <= u < box_mask.shape[1] and box_mask[v, u]
            c = (255, 60, 60) if on_obj else (60, 255, 60)
            e = a + 3 * (b - a)                         # exaggerate x3
            cv2.arrowedLine(out, (u, v), (int(e[0]), int(e[1])), c, 1, tipLength=0.3)
    if vo.get('ok') and dt > 0:
        txt = (f'VO  vx {vo["dx"] / dt:+.2f} m/s  wz {vo["dyaw"] / dt:+.2f} rad/s  '
               f'inliers {vo["inliers"]}')
    else:
        txt = f'VO  not tracking (tracked {vo.get("tracked", 0)})'
    cv2.putText(out, txt, (4, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (255, 255, 255), 1, cv2.LINE_AA)
    return title(out, 'OPTICAL FLOW (green ground -> odometry, red objects)')


def dashboard(tiles):
    """tiles: list of 6 equally sized RGB images -> 3 x 2 mosaic with thin separators."""
    H, W = tiles[0].shape[:2]
    rows = []
    for r in range(2):
        row = []
        for c in range(3):
            t = tiles[r * 3 + c].copy()
            t[:, -1] = 255
            t[-1, :] = 255
            row.append(t)
        rows.append(np.hstack(row))
    return np.vstack(rows)
