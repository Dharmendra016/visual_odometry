#!/usr/bin/env python3
"""
Wheel odometry for the 4-wheel-steering rover.

Each wheel i at (xi, yi) with steering angle d_i and rolling speed v_i = w_i * r
measures its ground velocity (v_i cos d_i, v_i sin d_i). For a rigid body:
    v_i cos d_i = vx - wz * yi
    v_i sin d_i = vy + wz * xi
Eight equations, three unknowns (vx, vy, wz) -> least squares.

The residual of that fit is a slip indicator: if the four wheels disagree about how the
body is moving, at least one is slipping. The residual inflates the published twist
covariance so the EKF trusts the wheels less (and visual odometry more) while slipping.

Publishes:
  /wheel/odom      nav_msgs/Odometry  integrated pose + body twist (frame odom -> base_footprint)
  /wheel/slip      std_msgs/Float64   RMS residual of the rigid-body fit (m/s)
No TF is published - the EKF owns odom -> base_footprint.
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

CORNERS = ['fl', 'fr', 'rl', 'rr']


class WheelOdometry(Node):

    def __init__(self):
        super().__init__('wheel_odometry')
        self.declare_parameter('wheelbase', 0.54)
        self.declare_parameter('track_width', 0.50)
        self.declare_parameter('wheel_radius', 0.0825)
        # (10 cm/s)^2 floor: all four wheels can slip together (ice, grass, wheelspin) without
        # disagreeing, which the residual cannot detect - so never trust wheels too much.
        self.declare_parameter('base_variance', 0.01)
        self.declare_parameter('slip_gain', 4.0)          # variance added per (m/s)^2 of residual

        L = self.get_parameter('wheelbase').value
        W = self.get_parameter('track_width').value
        self.r = self.get_parameter('wheel_radius').value
        self.base_var = self.get_parameter('base_variance').value
        self.slip_gain = self.get_parameter('slip_gain').value
        self.pos = {'fl': (L / 2, W / 2), 'fr': (L / 2, -W / 2),
                    'rl': (-L / 2, W / 2), 'rr': (-L / 2, -W / 2)}

        # Design matrix: rows alternate (x-equation, y-equation) per wheel
        rows = []
        for c in CORNERS:
            x, y = self.pos[c]
            rows.append([1.0, 0.0, -y])
            rows.append([0.0, 1.0, x])
        self.A = np.array(rows)
        self.A_pinv = np.linalg.pinv(self.A)

        self.x = self.y = self.yaw = 0.0
        self.last_stamp = None
        self.index = None

        self.odom_pub = self.create_publisher(Odometry, 'wheel/odom', 10)
        self.slip_pub = self.create_publisher(Float64, 'wheel/slip', 10)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 20)

    def on_joints(self, msg):
        if self.index is None:
            names = {n: i for i, n in enumerate(msg.name)}
            try:
                self.index = {c: (names[f'{c}_steer_joint'], names[f'{c}_wheel_joint'])
                              for c in CORNERS}
            except KeyError:
                return
        if len(msg.velocity) < len(msg.name):
            return

        b = []
        for c in CORNERS:
            si, wi = self.index[c]
            d = msg.position[si]
            v = msg.velocity[wi] * self.r
            b.extend([v * math.cos(d), v * math.sin(d)])
        b = np.array(b)
        vx, vy, wz = self.A_pinv @ b
        residual = float(np.sqrt(np.mean((self.A @ np.array([vx, vy, wz]) - b) ** 2)))

        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if self.last_stamp is not None:
            dt = (stamp - self.last_stamp).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                # midpoint integration in the plane
                mid = self.yaw + 0.5 * wz * dt
                self.x += (vx * math.cos(mid) - vy * math.sin(mid)) * dt
                self.y += (vx * math.sin(mid) + vy * math.cos(mid)) * dt
                self.yaw += wz * dt
        self.last_stamp = stamp

        var = self.base_var + self.slip_gain * residual ** 2
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2)
        pc = [0.0] * 36
        pc[0] = pc[7] = pc[35] = 1e3   # integrated pose is not meant to be fused
        pc[14] = pc[21] = pc[28] = 1e6
        odom.pose.covariance = pc
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.angular.z = float(wz)
        tc = [0.0] * 36
        tc[0] = tc[7] = var
        tc[35] = var * 4.0
        tc[14] = tc[21] = tc[28] = 1e6
        odom.twist.covariance = tc
        self.odom_pub.publish(odom)
        self.slip_pub.publish(Float64(data=residual))


def main():
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
