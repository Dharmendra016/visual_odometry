"""Minimal multi-object tracker in the odom frame (nearest neighbour + smoothed velocity)."""
import math


class Track:
    def __init__(self, tid, cls, x, y, radius, t):
        self.id, self.cls, self.radius = tid, cls, radius
        self.x, self.y, self.vx, self.vy = x, y, 0.0, 0.0
        self.t_last, self.hits = t, 1
        self.flow_resid = 0.0

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)


class Tracker:

    def __init__(self, gate=1.0, max_age=1.0, alpha=0.4, moving_speed=0.25, moving_flow=1.2):
        self.gate, self.max_age, self.alpha = gate, max_age, alpha
        self.moving_speed, self.moving_flow = moving_speed, moving_flow
        self.tracks = []
        self.next_id = 1

    def update(self, dets, t):
        """dets: [{'cls', 'x', 'y', 'radius', 'flow_resid'}] in odom. Returns live tracks."""
        unmatched = list(range(len(dets)))
        for tr in sorted(self.tracks, key=lambda k: -k.hits):
            best, bd = None, self.gate
            dt = max(t - tr.t_last, 1e-3)
            px, py = tr.x + tr.vx * dt, tr.y + tr.vy * dt
            for i in unmatched:
                d = dets[i]
                if d['cls'] != tr.cls:
                    continue
                dist = math.hypot(d['x'] - px, d['y'] - py)
                if dist < bd:
                    best, bd = i, dist
            if best is None:
                continue
            d = dets[best]
            unmatched.remove(best)
            if dt < 1.0:
                a = self.alpha
                tr.vx = (1 - a) * tr.vx + a * (d['x'] - tr.x) / dt
                tr.vy = (1 - a) * tr.vy + a * (d['y'] - tr.y) / dt
            tr.x, tr.y, tr.t_last, tr.hits = d['x'], d['y'], t, tr.hits + 1
            if d.get('flow_resid') is not None:
                tr.flow_resid = 0.5 * tr.flow_resid + 0.5 * d['flow_resid']
        for i in unmatched:
            d = dets[i]
            tr = Track(self.next_id, d['cls'], d['x'], d['y'], d['radius'], t)
            if d.get('flow_resid') is not None:
                tr.flow_resid = d['flow_resid']
            self.tracks.append(tr)
            self.next_id += 1
        self.tracks = [k for k in self.tracks if t - k.t_last <= self.max_age]
        return self.tracks

    def is_moving(self, tr):
        return tr.hits >= 3 and (tr.speed > self.moving_speed or tr.flow_resid > self.moving_flow)
