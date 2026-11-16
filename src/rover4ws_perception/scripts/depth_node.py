#!/usr/bin/env python3
"""
Monocular metric depth + obstacle/ground point clouds.

Pipeline (per camera frame):
  1. Depth Anything V2 -> relative inverse depth q (affine-invariant: 1/Z = a*q + b).
  2. Metric scale from the ground plane. The camera's pose relative to the ground is known
     from TF (mount height + pitch), so for every pixel whose ray hits the ground we know
     the depth the ground *would* have there: Z_g = -h / ray_z. Pixels that really are ground
     satisfy 1/Z_g = a*q + b; obstacles are outliers. RANSAC + least squares gives (a, b).
  3. Fit validity: too few ground inliers -> reuse the last good fit for at most
     `fit_hold_time`, otherwise publish nothing (downstream treats silence as STOP).
  4. Back-project to 3D in base_footprint, classify by height above ground:
        obstacle:  min_obstacle_height < z < max_obstacle_height, or z < -negative_obstacle_depth
        ground:    |z| < ground_band
     Points beyond max_range are dropped (monocular depth is unreliable far away).

source:=gt uses the simulator's ground-truth depth camera instead of the network
(for testing planning in isolation, and for measuring the network's error).

Publishes:
  /perception/depth       sensor_msgs/Image 32FC1 (metres, camera_optical_frame, same stamp as RGB)
  /perception/obstacles   sensor_msgs/PointCloud2 (base_footprint)
  /perception/ground      sensor_msgs/PointCloud2 (base_footprint)
  /perception/scale_fit   std_msgs/Float32MultiArray [a, b, inlier_fraction, valid]
"""
import array
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def make_cloud(points, frame, stamp):
    msg = PointCloud2()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate('xyz')]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    # array.array skips rclpy's per-byte validation (bytes would cost ~90 ms per frame)
    msg.data = array.array('B', points.astype(np.float32).tobytes())
    return msg


class DepthAnythingV2:
    """Thin wrapper around the Hugging Face Depth Anything V2 model."""

    def __init__(self, model_name, input_w, input_h, logger):
        import torch
        from transformers import AutoModelForDepthEstimation
        self.torch = torch
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.dtype = torch.float16 if self.device == 'cuda' else torch.float32
        logger.info(f'Loading {model_name} on {self.device} ...')
        self.model = AutoModelForDepthEstimation.from_pretrained(model_name).to(
            self.device, self.dtype).eval()
        self.size = (input_h, input_w)  # must be multiples of 14
        self.mean = torch.tensor([0.485, 0.456, 0.406], device=self.device).view(1, 3, 1, 1)
        self.std = torch.tensor([0.229, 0.224, 0.225], device=self.device).view(1, 3, 1, 1)
        if self.device == 'cpu':
            logger.warn('No CUDA - Depth Anything will run at ~1 Hz. Drive slowly.')

    def __call__(self, rgb):
        """rgb: HxWx3 uint8 -> HxW float32 relative inverse depth at input resolution."""
        torch = self.torch
        h, w = rgb.shape[:2]
        with torch.no_grad():
            x = torch.from_numpy(rgb).to(self.device).permute(2, 0, 1)[None].float() / 255.0
            x = torch.nn.functional.interpolate(x, size=self.size, mode='bilinear',
                                                align_corners=False)
            x = ((x - self.mean) / self.std).to(self.dtype)
            q = self.model(pixel_values=x).predicted_depth[:, None]
            q = torch.nn.functional.interpolate(q.float(), size=(h, w), mode='bilinear',
                                                align_corners=False)
        return q[0, 0].cpu().numpy()


class DepthNode(Node):

    def __init__(self):
        super().__init__('depth_node')
        p = self.declare_parameter
        p('source', 'model')                        # 'model' | 'gt'
        p('model_name', 'depth-anything/Depth-Anything-V2-Small-hf')
        p('input_width', 518)
        p('input_height', 392)
        p('base_frame', 'base_footprint')
        p('max_range', 4.0)                         # m, horizontal
        p('min_obstacle_height', 0.06)              # m, ~step the wheels cannot climb
        p('max_obstacle_height', 1.2)               # m, above this the robot passes under
        p('negative_obstacle_depth', 0.10)          # m, holes / drop-offs
        p('ground_band', 0.04)                      # m
        p('cloud_stride', 4)                        # pixel subsampling for the clouds
        p('fit_min_range', 0.4)
        p('fit_max_range', 5.0)
        p('fit_inlier_tol', 0.12)                   # relative inverse-depth error
        p('fit_min_inlier_fraction', 0.35)
        p('fit_hold_time', 0.5)                     # s
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self.source = gp('source')
        self.base_frame = gp('base_frame')
        self.max_range = gp('max_range')
        self.h_min, self.h_max = gp('min_obstacle_height'), gp('max_obstacle_height')
        self.neg = gp('negative_obstacle_depth')
        self.band = gp('ground_band')
        self.stride = gp('cloud_stride')
        self.fit_rng = (gp('fit_min_range'), gp('fit_max_range'))
        self.fit_tol = gp('fit_inlier_tol')
        self.fit_min_frac = gp('fit_min_inlier_fraction')
        self.fit_hold = gp('fit_hold_time')

        self.model = None
        if self.source == 'model':
            self.model = DepthAnythingV2(gp('model_name'), gp('input_width'),
                                         gp('input_height'), self.get_logger())

        self.K = None
        self.R = self.t = None          # camera_optical_frame -> base_frame
        self.rays = None                # full-res unit-z rays in optical frame (H, W, 3)
        self.ground_Z = None            # expected ground depth per pixel (nan if ray misses)
        self.last_fit = None            # (a, b, time)
        self.gt_depth = None
        self.timing = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, sensor_qos)
        if self.source == 'gt':
            self.create_subscription(Image, '/camera_gt/depth/image_raw', self.on_gt, sensor_qos)

        self.depth_pub = self.create_publisher(Image, '/perception/depth', 5)
        self.obs_pub = self.create_publisher(PointCloud2, '/perception/obstacles', 5)
        self.gnd_pub = self.create_publisher(PointCloud2, '/perception/ground', 5)
        self.fit_pub = self.create_publisher(Float32MultiArray, '/perception/scale_fit', 5)
        self.get_logger().info(f'depth_node ready (source={self.source})')

    # ---------------------------------------------------------------- setup
    def on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.size = (msg.height, msg.width)

    def ensure_geometry(self, frame_id):
        if self.R is not None:
            return True
        if self.K is None:
            return False
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame_id, rclpy.time.Time(),
                                                 timeout=Duration(seconds=0.1))
        except Exception as e:  # noqa: BLE001
            self.get_logger().info(f'waiting for TF {frame_id} -> {self.base_frame}: {e}',
                                   throttle_duration_sec=2.0)
            return False
        self.R = quat_to_rot(tf.transform.rotation)
        tr = tf.transform.translation
        self.t = np.array([tr.x, tr.y, tr.z])
        H, W = self.size
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        self.rays = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], -1)
        # z-component (in base frame) of each ray; ground hit where t_z + Z * dz = 0
        dz = self.rays @ self.R[2]
        with np.errstate(divide='ignore', invalid='ignore'):
            Zg = np.where(dz < -1e-6, -self.t[2] / dz, np.nan)
        self.ground_Z = Zg
        self.get_logger().info(
            f'camera height {self.t[2]:.3f} m above {self.base_frame}; geometry ready')
        return True

    # ---------------------------------------------------------------- callbacks
    def on_gt(self, msg):
        d = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        self.gt_depth = (msg.header.stamp, d)

    def on_image(self, msg):
        if not self.ensure_geometry(msg.header.frame_id or 'camera_optical_frame'):
            return
        t0 = time.perf_counter()
        if self.source == 'gt':
            if self.gt_depth is None:
                return
            Z = self.gt_depth[1]
            if Z.shape != (msg.height, msg.width):
                Z = cv2.resize(Z, (msg.width, msg.height), interpolation=cv2.INTER_NEAREST)
            Z = Z.copy()
            Z[~np.isfinite(Z)] = np.nan
            fit = (1.0, 0.0, 1.0, 1.0)
        else:
            rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == 'bgr8':
                rgb = rgb[..., ::-1]
            q = self.model(np.ascontiguousarray(rgb))
            t1 = time.perf_counter()
            Z, fit = self.metric_from_relative(q)
            if Z is None:
                self.fit_pub.publish(Float32MultiArray(data=[float(v) for v in fit]))
                return
        t2 = time.perf_counter()
        self.publish_all(msg, Z, fit)
        t3 = time.perf_counter()
        if self.source == 'gt':
            t1 = t0
        self.timing.append((t1 - t0, t2 - t1, t3 - t2))
        if len(self.timing) >= 50:
            m = 1000 * np.mean(self.timing, axis=0)
            self.get_logger().info(
                f'depth pipeline {m.sum():.1f} ms/frame (model {m[0]:.1f}, scale fit {m[1]:.1f}, '
                f'clouds {m[2]:.1f}); fit a={fit[0]:.3f} b={fit[1]:.3f} inliers={fit[2]:.2f}')
            self.timing = []

    # ---------------------------------------------------------------- scale recovery
    def metric_from_relative(self, q):
        Zg = self.ground_Z
        cand = np.isfinite(Zg) & (Zg > self.fit_rng[0]) & (Zg < self.fit_rng[1])
        # subsample candidates for speed
        idx = np.flatnonzero(cand.ravel())[::7]
        now = self.get_clock().now()
        fit_ok = False
        a = b = frac = 0.0
        if len(idx) > 200:
            qs = q.ravel()[idx]
            ys = 1.0 / Zg.ravel()[idx]
            rng = np.random.default_rng()
            best_in, best = None, 0
            for _ in range(60):
                i, j = rng.choice(len(idx), 2, replace=False)
                if abs(qs[i] - qs[j]) < 1e-6:
                    continue
                aa = (ys[i] - ys[j]) / (qs[i] - qs[j])
                bb = ys[i] - aa * qs[i]
                if aa <= 0:
                    continue
                inl = np.abs(aa * qs + bb - ys) < self.fit_tol * ys
                n = int(inl.sum())
                if n > best:
                    best, best_in = n, inl
            if best_in is not None:
                A = np.stack([qs[best_in], np.ones(best)], 1)
                a, b = np.linalg.lstsq(A, ys[best_in], rcond=None)[0]
                frac = best / len(idx)
                fit_ok = a > 0 and frac >= self.fit_min_frac
        if fit_ok:
            self.last_fit = (a, b, now)
        elif self.last_fit is not None and \
                (now - self.last_fit[2]).nanoseconds * 1e-9 < self.fit_hold:
            a, b = self.last_fit[0], self.last_fit[1]
            self.get_logger().warn(f'scale fit weak ({frac:.2f}); holding last fit',
                                   throttle_duration_sec=1.0)
        else:
            self.get_logger().warn(f'scale fit FAILED ({frac:.2f} inliers) - no output',
                                   throttle_duration_sec=1.0)
            return None, (a, b, frac, 0.0)
        inv = a * q + b
        with np.errstate(divide='ignore', invalid='ignore'):
            Z = np.where(inv > 1.0 / 50.0, 1.0 / inv, np.nan).astype(np.float32)
        return Z, (a, b, frac, 1.0)

    # ---------------------------------------------------------------- outputs
    def publish_all(self, msg, Z, fit):
        stamp = msg.header.stamp
        dimg = Image()
        dimg.header = msg.header
        dimg.height, dimg.width = Z.shape
        dimg.encoding = '32FC1'
        dimg.step = 4 * Z.shape[1]
        dimg.data = array.array('B', np.nan_to_num(Z, nan=0.0).astype(np.float32).tobytes())
        self.depth_pub.publish(dimg)

        s = self.stride
        Zs = Z[::s, ::s].ravel()
        rays = self.rays[::s, ::s].reshape(-1, 3)
        ok = np.isfinite(Zs) & (Zs > 0.05)
        P = (rays[ok] * Zs[ok, None]) @ self.R.T + self.t
        rng = np.hypot(P[:, 0], P[:, 1])
        P = P[rng < self.max_range]
        z = P[:, 2]
        obstacle = ((z > self.h_min) & (z < self.h_max)) | (z < -self.neg)
        ground = np.abs(z) < self.band
        self.obs_pub.publish(make_cloud(P[obstacle], self.base_frame, stamp))
        self.gnd_pub.publish(make_cloud(P[ground], self.base_frame, stamp))
        self.fit_pub.publish(Float32MultiArray(data=[float(v) for v in fit]))


def main():
    rclpy.init()
    node = DepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
