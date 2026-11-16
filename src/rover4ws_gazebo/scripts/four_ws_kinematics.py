#!/usr/bin/env python3
"""
cmd_vel -> per-corner steering angle + wheel speed for the 4-wheel-steering rover.

For a rigid body moving with (vx, vy, wz), the ground velocity of a wheel at (xi, yi) is
    vxi = vx - wz * yi
    vyi = vy + wz * xi
Each wheel is steered to atan2(vyi, vxi) and spun at |v| / r. With vy = 0 this gives
counter-phase four-wheel steering (front and rear steer opposite ways, tighter turns);
with wz = 0 and vy != 0 it gives crab motion (all wheels parallel, limited to
atan(vy/vx) <= max_steer, so pure sideways motion is not possible with +-35 deg).

If a command needs more steering than the hardware allows, (vy, wz) are scaled down
until it is feasible instead of clamping individual wheels (which would make them
fight each other). Wheel speeds are then scaled uniformly to respect max_wheel_speed.

A watchdog stops the wheels if cmd_vel goes stale.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class FourWSKinematics(Node):

    def __init__(self):
        super().__init__('four_ws_kinematics')
        self.declare_parameter('wheelbase', 0.54)
        self.declare_parameter('track_width', 0.50)
        self.declare_parameter('wheel_radius', 0.0825)
        self.declare_parameter('max_steer', 0.61)
        self.declare_parameter('max_wheel_speed', 20.0)   # rad/s at the wheel
        self.declare_parameter('cmd_timeout', 0.5)        # s
        self.declare_parameter('rate', 50.0)              # Hz

        L = self.get_parameter('wheelbase').value
        W = self.get_parameter('track_width').value
        self.r = self.get_parameter('wheel_radius').value
        self.max_steer = self.get_parameter('max_steer').value
        self.max_w = self.get_parameter('max_wheel_speed').value
        self.timeout = self.get_parameter('cmd_timeout').value

        # FL, FR, RL, RR - must match controllers.yaml joint order
        self.wheels = [(L / 2, W / 2), (L / 2, -W / 2), (-L / 2, W / 2), (-L / 2, -W / 2)]
        self.steer = [0.0] * 4
        self.cmd = Twist()
        self.last_cmd_time = None

        self.steer_pub = self.create_publisher(Float64MultiArray, 'steering_controller/commands', 10)
        self.wheel_pub = self.create_publisher(Float64MultiArray, 'wheel_controller/commands', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.update)

    def on_cmd(self, msg):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def solve(self, vx, vy, wz):
        """Return (angles, speeds) or None if some wheel exceeds the steering limit."""
        angles, speeds = [], []
        for i, (x, y) in enumerate(self.wheels):
            wvx = vx - wz * y
            wvy = vy + wz * x
            v = math.hypot(wvx, wvy)
            if v < 1e-4:
                angles.append(self.steer[i])  # hold current angle when stationary
                speeds.append(0.0)
                continue
            a = math.atan2(wvy, wvx)
            # Drive in reverse rather than steering past +-90 deg
            if a > math.pi / 2:
                a -= math.pi
                v = -v
            elif a < -math.pi / 2:
                a += math.pi
                v = -v
            if abs(a) > self.max_steer + 1e-6:
                return None
            angles.append(a)
            speeds.append(v / self.r)
        return angles, speeds

    def update(self):
        stale = (self.last_cmd_time is None or
                 (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 > self.timeout)
        vx, vy, wz = (0.0, 0.0, 0.0) if stale else (
            self.cmd.linear.x, self.cmd.linear.y, self.cmd.angular.z)

        result = self.solve(vx, vy, wz)
        if result is None:
            # Bisect the largest feasible fraction of the lateral/yaw command
            lo, hi = 0.0, 1.0
            for _ in range(20):
                mid = (lo + hi) / 2
                if self.solve(vx, vy * mid, wz * mid) is None:
                    hi = mid
                else:
                    lo = mid
            result = self.solve(vx, vy * lo, wz * lo)
            self.get_logger().warn(
                f'Command exceeds steering limit; turn scaled to {lo:.2f}', throttle_duration_sec=2.0)

        angles, speeds = result
        peak = max(abs(s) for s in speeds)
        if peak > self.max_w:
            speeds = [s * self.max_w / peak for s in speeds]

        self.steer = angles
        self.steer_pub.publish(Float64MultiArray(data=angles))
        self.wheel_pub.publish(Float64MultiArray(data=speeds))


def main():
    rclpy.init()
    node = FourWSKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
