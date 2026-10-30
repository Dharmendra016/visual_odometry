#!/usr/bin/env python3
"""
Drive a scripted pattern with /cmd_vel and compare odometry estimates with ground truth.

  python3 eval_odometry.py --pattern loop      # normal grass (spawn at 0,0)
  python3 eval_odometry.py --pattern ice       # low-friction patch (spawn at 1.5,7)

Each estimate is compared in the frame of the robot's starting pose, so all start at 0.
Prints final position error and error as % of distance travelled for:
  wheel odometry, visual odometry, and the EKF output (/odometry/filtered).
"""
import argparse
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

PATTERNS = {
    # (seconds, vx, wz)
    'loop': [(5, 0.4, 0.0), (6, 0.4, 0.45), (5, 0.4, 0.0), (6, 0.4, -0.45), (4, 0.4, 0.0), (2, 0, 0)],
    # on ice: hard accelerations and turns - the wheels spin and skid
    'ice': [(3, 0.6, 0.0), (1, 0.0, 0.0), (4, 0.5, 0.6), (1, 0.0, 0.0), (3, 0.6, -0.6), (2, 0, 0)],
}


def pose2d(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def relative(start, cur):
    dx, dy = cur[0] - start[0], cur[1] - start[1]
    c, s = math.cos(-start[2]), math.sin(-start[2])
    return dx * c - dy * s, dx * s + dy * c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pattern', default='loop', choices=PATTERNS)
    args = ap.parse_args()

    rclpy.init()
    n = Node('eval_odometry')
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    topics = {'truth': '/ground_truth/odom', 'wheel': '/wheel/odom',
              'visual': '/vo/odom', 'ekf': '/odometry/filtered'}
    latest = {k: None for k in topics}
    for k, t in topics.items():
        n.create_subscription(Odometry, t, lambda m, k=k: latest.__setitem__(k, pose2d(m)), 20)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)

    def spin_for(sec, vx=0.0, wz=0.0, track=None):
        t0 = n.get_clock().now()
        while (n.get_clock().now() - t0).nanoseconds < sec * 1e9:
            m = Twist()
            m.linear.x, m.angular.z = float(vx), float(wz)
            pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.02)
            if track is not None and latest['truth'] is not None:
                track.append(latest['truth'][:2])

    print('waiting for topics ...', flush=True)
    while latest['truth'] is None or latest['wheel'] is None or latest['ekf'] is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    spin_for(2.0)  # let VO initialise
    start = dict(latest)
    track = []
    for sec, vx, wz in PATTERNS[args.pattern]:
        spin_for(sec, vx, wz, track)
    end = dict(latest)

    dist = sum(math.dist(track[i], track[i + 1]) for i in range(len(track) - 1))
    truth = relative(start['truth'], end['truth'])
    print(f'\n==== odometry vs ground truth, pattern "{args.pattern}", '
          f'distance travelled {dist:.2f} m ====')
    print(f'{"source":8s} {"final x":>8s} {"final y":>8s} {"error m":>8s} {"% dist":>7s}')
    print(f'{"truth":8s} {truth[0]:8.3f} {truth[1]:8.3f}')
    for k in ('wheel', 'visual', 'ekf'):
        if start[k] is None or end[k] is None:
            print(f'{k:8s}   (no data)')
            continue
        est = relative(start[k], end[k])
        err = math.dist(est, truth)
        print(f'{k:8s} {est[0]:8.3f} {est[1]:8.3f} {err:8.3f} {100 * err / max(dist, 1e-6):6.1f}%')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
