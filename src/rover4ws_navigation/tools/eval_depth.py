#!/usr/bin/env python3
"""
Compare /perception/depth (Depth Anything V2 + ground-plane scale) with the simulator's
ground-truth depth camera. Requires depth_source:=model and gt_depth:=true (default).

  python3 eval_depth.py --frames 60 [--drive]

Metrics over pixels with true depth 0.3..max m:
  AbsRel = mean(|Z - Z*| / Z*)      delta1 = fraction with max(Z/Z*, Z*/Z) < 1.25
Reported separately for ground pixels and obstacle pixels (true point > 6 cm above ground).
"""
import argparse

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def to_np(msg):
    return np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=60)
    ap.add_argument('--max', type=float, default=4.0)
    ap.add_argument('--drive', action='store_true', help='drive slowly in a circle meanwhile')
    args = ap.parse_args()

    rclpy.init()
    n = Node('eval_depth')
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    gt, est = {}, {}
    key = lambda m: (m.header.stamp.sec, m.header.stamp.nanosec)  # noqa: E731
    n.create_subscription(Image, '/camera_gt/depth/image_raw', lambda m: gt.__setitem__(key(m), to_np(m)), qos)
    n.create_subscription(Image, '/perception/depth', lambda m: est.__setitem__(key(m), to_np(m)), qos)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)

    rows = []
    used = set()
    while len(rows) < args.frames:
        rclpy.spin_once(n, timeout_sec=0.05)
        if args.drive:
            t = Twist()
            t.linear.x, t.angular.z = 0.3, 0.25
            pub.publish(t)
        # gt and RGB cameras render on separate ticks; pair each estimate with the
        # nearest gt frame within 40 ms
        for k, Z in list(est.items()):
            if k in used:
                continue
            ts = k[0] + k[1] * 1e-9
            best = min(gt, key=lambda g: abs(g[0] + g[1] * 1e-9 - ts), default=None)
            if best is None or abs(best[0] + best[1] * 1e-9 - ts) > 0.04:
                continue
            used.add(k)
            G = gt[best]
            if Z.shape != G.shape:
                Z = cv2.resize(Z, (G.shape[1], G.shape[0]), interpolation=cv2.INTER_NEAREST)
            valid = np.isfinite(G) & (G > 0.3) & (G < args.max) & (Z > 0)
            if valid.sum() < 500:
                continue
            # ground pixels: lower part of the image whose gt depth matches a flat floor is
            # approximated by "row below horizon and smooth" - simpler: use vertical gradient
            gy = np.abs(np.gradient(G, axis=0))
            ground = valid & (np.arange(G.shape[0])[:, None] > G.shape[0] * 0.42) & (gy < 0.05 * G)
            obstacle = valid & ~ground
            r = np.abs(Z - G) / G
            d = np.maximum(Z / G, G / Z)
            rows.append([r[valid].mean(), (d[valid] < 1.25).mean(),
                         r[ground].mean() if ground.any() else np.nan,
                         r[obstacle].mean() if obstacle.sum() > 100 else np.nan])
            print(f'frame {len(rows):3d}: AbsRel {rows[-1][0]:.3f}  delta1 {rows[-1][1]:.3f}', flush=True)
    pub.publish(Twist())
    a = np.array(rows)
    print('\n==== Depth Anything V2 (ground-plane scaled) vs ground truth, '
          f'0.3-{args.max:.0f} m, {len(a)} frames ====')
    print(f'AbsRel all       : {np.nanmean(a[:, 0]):.3f}')
    print(f'delta<1.25 all   : {np.nanmean(a[:, 1]):.3f}')
    print(f'AbsRel ground    : {np.nanmean(a[:, 2]):.3f}')
    print(f'AbsRel obstacles : {np.nanmean(a[:, 3]):.3f}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
