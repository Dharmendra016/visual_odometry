#!/usr/bin/env python3
"""
Frame-to-frame visual odometry using the metric depth from depth_node.

  1. Detect corners in the previous frame (Shi-Tomasi), track them into the current
     frame with pyramidal Lucas-Kanade, keep tracks that pass a forward-backward check.
  2. Lift previous-frame corners to 3D with /perception/depth (metric, from the
     ground-plane-scaled Depth Anything V2 output).
  3. PnP + RANSAC gives the camera motion; convert it to body motion of base_footprint
     using the camera extrinsics, divide by dt -> body velocity (vx, vy, wz).
  4. Health gate: too few inliers / implausible motion -> publish nothing. The EKF then
     simply runs on wheels + IMU until tracking recovers.

Publishes:
  /vo/odom     nav_msgs/Odometry  body twist (+ integrated pose for evaluation only)
  /vo/status   std_msgs/Float32MultiArray [tracked, inliers, healthy]
"""
import math

import cv2
import numpy as np
import rclpy
from message_filters import Subscriber, TimeSynchronizer
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class VisualOdometry(Node):

    def __init__(self):
        super().__init__('visual_odometry')
        p = self.declare_parameter
        p('base_frame', 'base_footprint')
        p('max_features', 500)
        p('max_depth', 6.0)          # only use points closer than this (depth is better near)
        p('min_inliers', 30)
        p('min_inlier_ratio', 0.35)
        p('max_speed', 2.0)          # m/s, reject implausible jumps
        p('max_yaw_rate', 2.5)       # rad/s
        p('base_variance', 0.0025)   # (5 cm/s)^2 at 100 inliers
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.base_frame = gp('base_frame')
        self.max_features = gp('max_features')
        self.max_depth = gp('max_depth')
        self.min_inliers = gp('min_inliers')
        self.min_ratio = gp('min_inlier_ratio')
        self.max_speed = gp('max_speed')
        self.max_yaw_rate = gp('max_yaw_rate')
        self.base_var = gp('base_variance')

        self.K = None
        self.T_bc = None            # camera_optical_frame pose in base frame (4x4)
        self.prev = None            # (gray, depth, stamp_sec)
        self.x = self.y = self.yaw = 0.0
        self.lost_count = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 10)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        img_sub = Subscriber(self, Image, '/camera/image_raw', qos_profile=qos)
        dep_sub = Subscriber(self, Image, '/perception/depth')
        self.sync = TimeSynchronizer([img_sub, dep_sub], 10)
        self.sync.registerCallback(self.on_frame)

        self.odom_pub = self.create_publisher(Odometry, '/vo/odom', 10)
        self.status_pub = self.create_publisher(Float32MultiArray, '/vo/status', 10)

    def on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)

    def ensure_extrinsics(self, frame):
        if self.T_bc is not None:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame, rclpy.time.Time(),
                                                 timeout=Duration(seconds=0.1))
        except Exception:  # noqa: BLE001
            return False
        T = np.eye(4)
        T[:3, :3] = quat_to_rot(tf.transform.rotation)
        tr = tf.transform.translation
        T[:3, 3] = [tr.x, tr.y, tr.z]
        self.T_bc = T
        self.T_cb = np.linalg.inv(T)
        return True

    def on_frame(self, img, dep):
        if self.K is None or not self.ensure_extrinsics(img.header.frame_id):
            return
        rgb = np.frombuffer(img.data, np.uint8).reshape(img.height, img.width, 3)
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        depth = np.frombuffer(dep.data, np.float32).reshape(dep.height, dep.width)
        stamp = img.header.stamp.sec + img.header.stamp.nanosec * 1e-9

        if self.prev is not None:
            self.track(self.prev, (gray, depth, stamp), img.header.stamp)
        self.prev = (gray, depth, stamp)

    def track(self, prev, curr, ros_stamp):
        g0, d0, t0 = prev
        g1, _, t1 = curr
        dt = t1 - t0
        if dt <= 0 or dt > 0.5:
            return

        # features only where depth is valid and near enough
        mask = ((d0 > 0.1) & (d0 < self.max_depth)).astype(np.uint8) * 255
        p0 = cv2.goodFeaturesToTrack(g0, self.max_features, 0.01, 8, mask=mask, blockSize=7)
        tracked = inliers = 0
        healthy = False
        if p0 is not None and len(p0) >= self.min_inliers:
            lk = dict(winSize=(21, 21), maxLevel=3,
                      criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
            p1, st, _ = cv2.calcOpticalFlowPyrLK(g0, g1, p0, None, **lk)
            p0r, st2, _ = cv2.calcOpticalFlowPyrLK(g1, g0, p1, None, **lk)
            fb = np.linalg.norm((p0 - p0r).reshape(-1, 2), axis=1)
            good = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < 1.0)
            p0g, p1g = p0.reshape(-1, 2)[good], p1.reshape(-1, 2)[good]
            tracked = len(p0g)
            if tracked >= self.min_inliers:
                u = np.clip(p0g[:, 0].round().astype(int), 0, d0.shape[1] - 1)
                v = np.clip(p0g[:, 1].round().astype(int), 0, d0.shape[0] - 1)
                Z = d0[v, u]
                fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
                P = np.stack([(p0g[:, 0] - cx) / fx * Z, (p0g[:, 1] - cy) / fy * Z, Z], 1)
                ok, rvec, tvec, inl = cv2.solvePnPRansac(
                    P.astype(np.float64), p1g.astype(np.float64), self.K, None,
                    iterationsCount=100, reprojectionError=2.0, confidence=0.99,
                    flags=cv2.SOLVEPNP_EPNP)
                if ok and inl is not None and len(inl) >= self.min_inliers:
                    idx = inl.ravel()
                    ok, rvec, tvec = cv2.solvePnP(P[idx], p1g[idx], self.K, None, rvec, tvec,
                                                  True, cv2.SOLVEPNP_ITERATIVE)
                    inliers = len(idx)
                    if ok and inliers / tracked >= self.min_ratio:
                        healthy = self.publish_motion(rvec, tvec, dt, inliers, ros_stamp)

        self.status_pub.publish(Float32MultiArray(
            data=[float(tracked), float(inliers), 1.0 if healthy else 0.0]))
        if not healthy:
            self.lost_count += 1
            if self.lost_count % 15 == 1:
                self.get_logger().warn(
                    f'VO tracking weak (tracked={tracked}, inliers={inliers}); not publishing')
        else:
            self.lost_count = 0

    def publish_motion(self, rvec, tvec, dt, inliers, stamp):
        # PnP gives T_c1_c0 (maps prev-camera points into the current camera frame)
        R, _ = cv2.Rodrigues(rvec)
        T_c1c0 = np.eye(4)
        T_c1c0[:3, :3], T_c1c0[:3, 3] = R, tvec.ravel()
        # body motion: pose of current base in previous base frame
        T_b0b1 = self.T_bc @ np.linalg.inv(T_c1c0) @ self.T_cb
        dx, dy = T_b0b1[0, 3], T_b0b1[1, 3]
        dyaw = math.atan2(T_b0b1[1, 0], T_b0b1[0, 0])
        vx, vy, wz = dx / dt, dy / dt, dyaw / dt
        if math.hypot(vx, vy) > self.max_speed or abs(wz) > self.max_yaw_rate:
            return False

        mid = self.yaw + 0.5 * dyaw
        self.x += dx * math.cos(mid) - dy * math.sin(mid)
        self.y += dx * math.sin(mid) + dy * math.cos(mid)
        self.yaw += dyaw

        var = self.base_var * max(1.0, 100.0 / inliers)
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z = math.sin(self.yaw / 2)
        o.pose.pose.orientation.w = math.cos(self.yaw / 2)
        pc = [0.0] * 36
        pc[0] = pc[7] = pc[35] = 1e3
        pc[14] = pc[21] = pc[28] = 1e6
        o.pose.covariance = pc
        o.twist.twist.linear.x, o.twist.twist.linear.y = vx, vy
        o.twist.twist.angular.z = wz
        tc = [0.0] * 36
        tc[0] = tc[7] = var
        tc[35] = var * 2.0
        tc[14] = tc[21] = tc[28] = 1e6
        o.twist.covariance = tc
        self.odom_pub.publish(o)
        return True


def main():
    rclpy.init()
    node = VisualOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
