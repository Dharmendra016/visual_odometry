#!/usr/bin/env python3
"""
Last software gate before the motors. Passes /cmd_vel_safe -> /cmd_vel unless:
  - perception is stale (no /perception/obstacles for `perception_timeout`) -> STOP
    (Nav2's collision_monitor silently ignores stale sources, so it cannot catch this)
  - the IMU reports roll/pitch beyond `max_tilt` -> STOP
States are published on /safety/state as GO / STOP_STALE_PERCEPTION / STOP_TILT.

The physical e-stop must cut motor power in hardware - it never depends on this node.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String


class SafetyGate(Node):

    def __init__(self):
        super().__init__('safety_gate')
        self.declare_parameter('perception_timeout', 0.5)
        self.declare_parameter('max_tilt', 0.35)  # rad, ~20 deg
        self.timeout = self.get_parameter('perception_timeout').value
        self.max_tilt = self.get_parameter('max_tilt').value

        self.last_perception = None
        self.tilt = 0.0
        self.state = None

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, '/perception/obstacles', self.on_perception, qos)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos)
        self.create_subscription(Twist, 'cmd_vel_safe', self.on_cmd, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/safety/state', 10)

    def on_perception(self, _msg):
        self.last_perception = self.get_clock().now()

    def on_imu(self, msg):
        q = msg.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self.tilt = max(abs(roll), abs(pitch))

    def on_cmd(self, msg):
        now = self.get_clock().now()
        if self.last_perception is None or \
                (now - self.last_perception).nanoseconds * 1e-9 > self.timeout:
            state = 'STOP_STALE_PERCEPTION'
        elif self.tilt > self.max_tilt:
            state = 'STOP_TILT'
        else:
            state = 'GO'
        moving = abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z) > 1e-3
        if state != 'GO' and moving:
            self.get_logger().warn(f'{state}: blocking motion command', throttle_duration_sec=1.0)
            msg = Twist()
        if state != self.state:
            self.state = state
            self.state_pub.publish(String(data=state))
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
