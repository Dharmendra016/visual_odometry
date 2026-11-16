#!/usr/bin/env python3
"""
Test the independent safety layers (collision_monitor + safety_gate) WITHOUT the planner.

Phase 1 - sudden obstacle: spawn a box 2.5 m ahead, push a blind "drive forward" command
          into /cmd_vel_nav (as a buggy planner would). The rover must stop before the box.
Phase 2 - box removed: motion must resume.
Phase 3 - perception dies: kill depth_node while driving. The rover must stop within
          ~0.5 s (safety_gate perception_timeout).

Run with the full prototype launched in open space, e.g.
  ros2 launch rover4ws_navigation prototype.launch.py x:=-8.0 y:=-10.0
"""
import math
import subprocess

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

BOX = """<sdf version="1.6"><model name="sudden_box"><static>true</static><link name="l">
<collision name="c"><geometry><box><size>0.4 0.8 0.5</size></box></geometry></collision>
<visual name="v"><geometry><box><size>0.4 0.8 0.5</size></box></geometry>
<material><script><uri>file://media/materials/scripts/gazebo.material</uri>
<name>Gazebo/WoodPallet</name></script></material></visual></link></model></sdf>"""


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--kill', default='depth_node.py',
                    help='perception process to kill in phase 3 (rgb_perception_node.py for the RGB stack)')
    args = ap.parse_args()
    rclpy.init()
    n = Node('safety_test')
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    st = {'pose': None, 'speed': 0.0}

    def on_gt(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        st['pose'] = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        st['speed'] = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)

    n.create_subscription(Odometry, '/ground_truth/odom', on_gt, 10)
    pub = n.create_publisher(Twist, '/cmd_vel_nav', 10)
    spawn = n.create_client(SpawnEntity, '/spawn_entity')
    delete = n.create_client(DeleteEntity, '/delete_entity')

    def drive(sec, vx=0.4):
        t0 = n.get_clock().now()
        while (n.get_clock().now() - t0).nanoseconds < sec * 1e9:
            m = Twist()
            m.linear.x = vx
            pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.05)

    def call(client, req):
        client.wait_for_service()
        f = client.call_async(req)
        rclpy.spin_until_future_complete(n, f)
        return f.result()

    while st['pose'] is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    drive(3.0, 0.0)

    # ---- phase 1
    x, y, yaw = st['pose']
    bx, by = x + 2.5 * math.cos(yaw), y + 2.5 * math.sin(yaw)
    req = SpawnEntity.Request()
    req.name, req.xml = 'sudden_box', BOX
    req.initial_pose.position.x, req.initial_pose.position.y = bx, by
    req.initial_pose.position.z = 0.25
    req.initial_pose.orientation.z, req.initial_pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    call(spawn, req)
    drive(10.0)
    x, y, _ = st['pose']
    front_gap = math.hypot(bx - x, by - y) - 0.2 - 0.36   # box half-depth, robot front
    p1 = st['speed'] < 0.02 and front_gap > 0.0
    print(f'PHASE 1 sudden obstacle : speed {st["speed"]:.2f} m/s, gap robot-front to box '
          f'{front_gap:.2f} m -> {"PASS (stopped before box)" if p1 else "FAIL"}', flush=True)

    # ---- phase 2
    req = DeleteEntity.Request()
    req.name = 'sudden_box'
    call(delete, req)
    drive(4.0)
    p2 = st['speed'] > 0.2
    print(f'PHASE 2 box removed     : speed {st["speed"]:.2f} m/s -> {"PASS (moving again)" if p2 else "FAIL"}',
          flush=True)

    # ---- phase 3
    # not `pkill -f`: this script's own command line contains the pattern (--kill ...)
    import os
    import signal
    me = {os.getpid(), os.getppid()}
    pids = subprocess.run(['pgrep', '-f', args.kill], capture_output=True, text=True).stdout.split()
    for pid in pids:
        cmd = open(f'/proc/{pid}/cmdline').read() if os.path.exists(f'/proc/{pid}/cmdline') else ''
        if int(pid) not in me and 'safety_test' not in cmd:
            os.kill(int(pid), signal.SIGKILL)
    t0 = n.get_clock().now()
    stopped_at = None
    while (n.get_clock().now() - t0).nanoseconds < 4e9:
        m = Twist()
        m.linear.x = 0.4
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.05)
        if stopped_at is None and st['speed'] < 0.02:
            stopped_at = (n.get_clock().now() - t0).nanoseconds * 1e-9
    p3 = stopped_at is not None and stopped_at < 1.5
    print(f'PHASE 3 perception died : stopped after '
          f'{"never" if stopped_at is None else f"{stopped_at:.2f} s"} -> {"PASS" if p3 else "FAIL"}', flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
