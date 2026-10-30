#!/usr/bin/env python3
"""
Send a NavigateToPose goal and judge the run from simulator ground truth.

  python3 nav_test.py --x 12 --y 0 [--yaw 0] [--timeout 240]     (world coordinates)

  python3 nav_test.py --world stone_yard --x 14 --y 0

Reports: Nav2 result, time, final distance to goal (true position), and the minimum
clearance between the robot's footprint and every obstacle in the course - including the
moving actors in stone_yard, whose positions are computed from their scripted trajectories.
Clearance <= 0 means a collision.
"""
import argparse
import json
import math
import os

import numpy as np
import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

# (name, cx, cy, yaw, half_x, half_y) from test_yard.world; cylinders as squares
OBSTACLES = [
    ('crate_a', 3.0, 0.0, 0.2, 0.30, 0.30),
    ('crate_b', 5.5, 1.0, 0.0, 0.25, 0.70),
    ('crate_c', 5.5, -1.6, 0.0, 0.25, 0.70),
    ('wall', 8.0, 0.4, 0.0, 0.10, 1.30),
    ('person', 10.0, -0.9, 0.0, 0.20, 0.20),
    ('fence_right', 7.0, -4.5, 0.0, 7.0, 0.05),
    ('shed_far', 16.0, 0.0, 0.0, 0.5, 4.0),
]
FOOT_HALF = (0.36, 0.28)  # deck / wheel extents


def footprint_points(x, y, yaw):
    hx, hy = FOOT_HALF
    pts = []
    for t in np.linspace(-1, 1, 15):
        pts += [(hx, t * hy), (-hx, t * hy), (t * hx, hy), (t * hx, -hy)]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + px * c - py * s, y + px * s + py * c) for px, py in pts]


def box_distance(px, py, cx, cy, yaw, hx, hy):
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = (px - cx) * c - (py - cy) * s, (px - cx) * s + (py - cy) * c
    dx, dy = abs(lx) - hx, abs(ly) - hy
    if dx < 0 and dy < 0:
        return max(dx, dy)
    return math.hypot(max(dx, 0), max(dy, 0))


def actor_position(waypoints, t):
    """Linear interpolation of a looping Gazebo actor trajectory [(t, x, y, yaw), ...]."""
    t = t % waypoints[-1][0]
    for (t0, x0, y0, _), (t1, x1, y1, _) in zip(waypoints, waypoints[1:]):
        if t0 <= t <= t1:
            a = (t - t0) / max(t1 - t0, 1e-9)
            return x0 + a * (x1 - x0), y0 + a * (y1 - y0)
    return waypoints[-1][1], waypoints[-1][2]


def load_obstacles(world):
    """List of (name, distance_fn(px, py, sim_time), position_fn(sim_time) or None)."""
    if world == 'test_yard':
        return [(o[0], lambda px, py, t, b=o[1:]: box_distance(px, py, *b), None) for o in OBSTACLES]
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                        'rover4ws_gazebo', 'worlds', f'{world}.json')
    info = json.load(open(path))
    obs = []
    for s in info['static']:
        obs.append((s['name'], lambda px, py, t, c=s['circles']:
                    min(math.hypot(px - cx, py - cy) - r for cx, cy, r in c), None))
    for name, *b in info['boxes']:
        obs.append((name, lambda px, py, t, b=b: box_distance(px, py, *b), None))
    for m in info['moving']:
        def dist(px, py, t, m=m):
            ax, ay = actor_position(m['waypoints'], t)
            return math.hypot(px - ax, py - ay) - m['radius']
        obs.append((m['name'] + ' (moving)', dist,
                    lambda t, w=m['waypoints']: actor_position(w, t)))
    return obs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='test_yard', help='test_yard | stone_yard')
    ap.add_argument('--x', type=float, default=12.0)
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--yaw', type=float, default=None,
                    help='final heading (world, rad); default = direction of travel')
    ap.add_argument('--timeout', type=float, default=240.0)
    args = ap.parse_args()

    rclpy.init()
    n = Node('nav_test')
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    obstacles = load_obstacles(args.world)
    state = {'pose': None, 'min': {o[0]: 99.0 for o in obstacles}, 'track': []}

    def on_gt(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        speed = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        state['pose'] = (p.x, p.y, yaw)
        state['track'].append((p.x, p.y))
        fp = footprint_points(p.x, p.y, yaw)
        for name, dist, pos in obstacles:
            d = min(dist(px, py, t) for px, py in fp)
            if d < state['min'][name]:
                state['min'][name] = d
                if pos is not None:
                    # where was the mover relative to the rover, and was the rover moving?
                    ax, ay = pos(t)
                    bearing = math.degrees(math.atan2(ay - p.y, ax - p.x) - yaw)
                    bearing = (bearing + 180) % 360 - 180
                    state.setdefault('closest', {})[name] = (bearing, speed)

    def on_ekf(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        state['ekf'] = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    n.create_subscription(Odometry, '/ground_truth/odom', on_gt, 20)
    n.create_subscription(Odometry, '/odometry/filtered', on_ekf, 20)
    client = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    print('waiting for navigate_to_pose ...', flush=True)
    client.wait_for_server()
    while state['pose'] is None or state.get('ekf') is None:
        rclpy.spin_once(n, timeout_sec=0.1)

    # Goals are given in WORLD coordinates; Nav2 runs in odom, whose origin is wherever the
    # robot started. odom->world offset = truth_pose * inverse(ekf_pose).
    wx, wy, wyaw = state['pose']
    ex, ey, eyaw = state['ekf']
    dyaw = wyaw - eyaw
    ox = wx - (ex * math.cos(dyaw) - ey * math.sin(dyaw))
    oy = wy - (ex * math.sin(dyaw) + ey * math.cos(dyaw))
    gx, gy = args.x - ox, args.y - oy
    gx, gy = gx * math.cos(-dyaw) - gy * math.sin(-dyaw), gx * math.sin(-dyaw) + gy * math.cos(-dyaw)
    # final heading: along the straight line from the robot to the goal (a car-like robot
    # cannot cheaply arrive facing an arbitrary direction)
    gyaw = math.atan2(args.y - wy, args.x - wx) - dyaw if args.yaw is None else args.yaw - dyaw

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'odom'
    goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
    goal.pose.pose.orientation.z = math.sin(gyaw / 2)
    goal.pose.pose.orientation.w = math.cos(gyaw / 2)
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, fut)
    handle = fut.result()
    if not handle.accepted:
        print('GOAL REJECTED')
        return
    t0 = n.get_clock().now()
    res = handle.get_result_async()
    last_print = 0.0
    while not res.done():
        rclpy.spin_once(n, timeout_sec=0.1)
        el = (n.get_clock().now() - t0).nanoseconds * 1e-9
        if el - last_print > 10:
            last_print = el
            x, y, _ = state['pose']
            print(f'  t={el:5.0f}s  at ({x:5.2f}, {y:5.2f})  to goal {math.hypot(args.x - x, args.y - y):5.2f} m',
                  flush=True)
        if el > args.timeout:
            handle.cancel_goal_async()
            print('TIMEOUT')
            break
    el = (n.get_clock().now() - t0).nanoseconds * 1e-9
    status = res.result().status if res.done() else -1
    x, y, _ = state['pose']
    tr = state['track']
    dist = sum(math.dist(tr[i], tr[i + 1]) for i in range(len(tr) - 1))
    names = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED', -1: 'TIMEOUT'}
    print(f'\n==== navigation to ({args.x}, {args.y}) ====')
    print(f'result            : {names.get(status, status)}')
    print(f'time (sim)        : {el:.1f} s, path length {dist:.1f} m')
    print(f'true final error  : {math.hypot(args.x - x, args.y - y):.2f} m')
    if args.world == 'stone_yard':
        on_gravel = sum(1 for x, y in tr if 1.5 <= x <= 4.5 and abs(y) <= 1.3)
        print(f'gravel patch      : {"drove over it" if on_gravel > 20 else "went around it"} '
              f'({on_gravel} pose samples on the patch)')
    print('min clearance to obstacles (true geometry):')
    for name, d in state['min'].items():
        flag = 'COLLISION' if d <= 0.0 else ''
        extra = ''
        if name in state.get('closest', {}):
            b, v = state['closest'][name]
            if d <= 0.0:
                # scripted actors never avoid anything; distinguish who moved into whom
                at_fault = v > 0.2 and abs(b) < 60      # rover moving AND mover in front
                flag = ('COLLISION - rover drove into it' if at_fault else
                        'CONTACT - mover reached a stopped rover or came from the side/behind')
            side = 'ahead' if abs(b) < 50 else 'behind' if abs(b) > 130 else 'at the side'
            extra = f'  [mover {side} ({b:+.0f} deg), rover speed {v:.2f} m/s at closest approach]'
        print(f'  {name:12s} {d:6.2f} m {flag}{extra}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
