#!/usr/bin/env python3
"""
RGB-only perception: one ordinary camera, no depth sensor, no depth network.

Per frame (work resolution 320x240):
  SegFormer (ADE20K)  -> drivable / sky masks
  YOLO (COCO)         -> people, vehicles, animals (+ any object = obstacle)
  free-space scan     -> static obstacle boundary + free ground (flat-ground projection),
                         obstacle height from the vertical extent (pebbles < 6 cm stay drivable)
  corridor            -> most open heading, free range, width; vanishing point of path edges
  optical flow        -> ground-plane visual odometry (/vo/odom for the EKF)
                         + per-object independent-motion residual
  tracker             -> object positions/velocities in odom, moving flag

Publishes:
  /rgb/obstacles  PointCloud2 base_footprint  static obstacle boundary (every frame, even empty)
  /rgb/freespace  PointCloud2 base_footprint  ground points to clear the costmap
  /rgb/dynamic    PointCloud2 base_footprint  footprints of detected people/vehicles/animals
  /rgb/objects    Float32MultiArray           tracks: [id, class_idx, x, y, vx, vy, radius, moving]*N (odom)
  /rgb/corridor   Float32MultiArray           [heading, free_range, width, vp_u, vp_v, vp_conf]
  /rgb/overlay    Image rgb8                  debug view for RViz
  /vo/odom, /vo/status                        same interface as the depth-based VO
"""
import array
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField  # noqa: E402
from std_msgs.msg import Float32MultiArray, MultiArrayDimension  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

import corridor as corr  # noqa: E402
import freespace  # noqa: E402
from flow import GroundFlow  # noqa: E402
from ground_geometry import GroundGeometry, quat_to_rot  # noqa: E402
from models import SegFormer, Yolo  # noqa: E402
from tracker import Tracker  # noqa: E402
import views  # noqa: E402

CLASS_IDS = {}


def cloud(points, frame, stamp):
    pts = np.asarray(points, np.float32).reshape(-1, 3)
    msg = PointCloud2()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    msg.height, msg.width = 1, len(pts)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate('xyz')]
    msg.point_step, msg.row_step = 12, 12 * len(pts)
    msg.is_bigendian, msg.is_dense = False, True
    msg.data = array.array('B', pts.tobytes())   # array.array avoids rclpy per-byte checks
    return msg


def circle_points(x, y, r, z=0.5, n=16):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = np.stack([x + r * np.cos(a), y + r * np.sin(a), np.full(n, z)], 1)
    return np.vstack([ring, [[x, y, z]]])


class RgbPerception(Node):

    def __init__(self):
        super().__init__('rgb_perception')
        p = self.declare_parameter
        p('seg_model', 'nvidia/segformer-b2-finetuned-ade-512-512')
        p('seg_input_height', 384)
        p('seg_input_width', 512)
        p('yolo_weights', os.path.expanduser('~/.cache/rover4ws/yolo11n.pt'))
        p('yolo_conf', 0.35)
        p('work_width', 320)
        p('work_height', 240)
        p('base_frame', 'base_footprint')
        p('odom_frame', 'odom')
        p('max_range', 5.0)
        p('min_obstacle_height', 0.06)
        p('free_stride', 3)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.work = (gp('work_width'), gp('work_height'))
        self.base, self.odom = gp('base_frame'), gp('odom_frame')
        self.max_range = gp('max_range')
        self.min_h = gp('min_obstacle_height')
        self.free_stride = gp('free_stride')

        self.seg = SegFormer(gp('seg_model'), (gp('seg_input_height'), gp('seg_input_width')),
                             self.get_logger())
        weights = gp('yolo_weights')
        if not os.path.exists(weights):
            os.makedirs(os.path.dirname(weights), exist_ok=True)
            os.chdir(os.path.dirname(weights))   # ultralytics downloads into the cwd
            weights = os.path.basename(weights)
        self.yolo = Yolo(weights, gp('yolo_conf'), self.get_logger())
        for i, name in self.yolo.names.items():
            CLASS_IDS[name] = int(i)

        self.flow = GroundFlow()
        self.tracker = Tracker()
        self.K = None
        self.geom = None
        self.prev = None          # (gray, drivable, box_mask, stamp_sec)
        self.vo_pose = [0.0, 0.0, 0.0]
        self.timing = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos)
        pub = self.create_publisher
        self.pub_obs = pub(PointCloud2, '/rgb/obstacles', 5)
        self.pub_free = pub(PointCloud2, '/rgb/freespace', 5)
        self.pub_dyn = pub(PointCloud2, '/rgb/dynamic', 5)
        self.pub_objects = pub(Float32MultiArray, '/rgb/objects', 5)
        self.pub_corr = pub(Float32MultiArray, '/rgb/corridor', 5)
        self.pub_overlay = pub(Image, '/rgb/overlay', 2)
        # visualisation views (rendered only while someone subscribes) + 3x2 dashboard
        self.view_names = ['semantic', 'traversability', 'distance', 'obstacles', 'vanishing',
                           'detections', 'flow']
        self.pub_views = {n: pub(Image, f'/rgb/view/{n}', 2) for n in self.view_names}
        self.pub_dashboard = pub(Image, '/rgb/dashboard', 2)
        self.prev_ts = None
        self.pub_vo = pub(Odometry, '/vo/odom', 10)
        self.pub_vo_status = pub(Float32MultiArray, '/vo/status', 10)
        self.get_logger().info('rgb_perception ready')

    # ------------------------------------------------------------------ setup
    def on_info(self, msg):
        if self.K is None:
            self.K_full = np.array(msg.k).reshape(3, 3)
            self.full = (msg.width, msg.height)
            sx, sy = self.work[0] / msg.width, self.work[1] / msg.height
            self.K = self.K_full.copy()
            self.K[0] *= sx
            self.K[1] *= sy

    def ensure_geometry(self, frame):
        if self.geom is not None:
            return True
        if self.K is None:
            return False
        try:
            tf = self.tf_buffer.lookup_transform(self.base, frame, rclpy.time.Time(),
                                                 timeout=Duration(seconds=0.1))
        except Exception:  # noqa: BLE001
            return False
        tr = tf.transform.translation
        self.geom = GroundGeometry(self.K, quat_to_rot(tf.transform.rotation),
                                   [tr.x, tr.y, tr.z], *self.work)
        hz = np.nonzero(self.geom.valid.any(1))[0]
        self.horizon_row = int(hz[0]) if len(hz) else 0
        self.get_logger().info(f'camera {tr.z:.3f} m above ground, horizon at work row '
                               f'{self.horizon_row}; geometry ready')
        return True

    def odom_from_base(self, stamp):
        # never block: the EKF pose for the exact image stamp is usually not there yet, and
        # waiting for it cost ~30 ms per frame. Fall back to the latest pose (~2 cm off at 0.4 m/s).
        try:
            tf = self.tf_buffer.lookup_transform(self.odom, self.base, stamp)
        except Exception:  # noqa: BLE001
            try:
                tf = self.tf_buffer.lookup_transform(self.odom, self.base, rclpy.time.Time())
            except Exception:  # noqa: BLE001
                return None
        q, t = tf.transform.rotation, tf.transform.translation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    # ------------------------------------------------------------------ main loop
    def on_image(self, msg):
        if not self.ensure_geometry(msg.header.frame_id or 'camera_optical_frame'):
            return
        t0 = time.perf_counter()
        W, H = self.work
        rgb_full = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == 'bgr8':
            rgb_full = rgb_full[..., ::-1]
        rgb_full = np.ascontiguousarray(rgb_full)
        bgr_full = cv2.cvtColor(rgb_full, cv2.COLOR_RGB2BGR)
        rgb = cv2.resize(rgb_full, (W, H), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        stamp = msg.header.stamp
        ts = stamp.sec + stamp.nanosec * 1e-9

        labels = self.seg(rgb_full, (H, W))
        t_seg = time.perf_counter()
        dets = self.yolo(bgr_full)
        t_yolo = time.perf_counter()
        sx, sy = W / msg.width, H / msg.height
        for d in dets:
            d['wbox'] = d['box'] * [sx, sy, sx, sy]
        dyn_boxes = [d['wbox'] for d in dets if d['dynamic']]

        fs = freespace.scan(labels, self.seg.drivable_lut, self.seg.sky_lut, self.geom,
                            self.max_range, self.min_h, dyn_boxes)
        cor = corr.corridor(fs['col_range'], fs['col_az'])
        vpinfo = corr.vanishing_point(fs['drivable'], self.horizon_row)
        vp, vp_conf = vpinfo['vp'], vpinfo['conf']

        # ---- optical flow: ego-motion + object motion
        box_mask = np.zeros((H, W), bool)
        for d in dets:
            x1, y1, x2, y2 = d['wbox'].astype(int)
            box_mask[max(y1, 0):y2, max(x1, 0):x2] = True
        vo = {'ok': False, 'tracked': 0, 'inliers': 0}
        if self.prev is not None and 0 < ts - self.prev[3] <= 0.5:
            vo = self.flow.update(self.prev[0], gray, self.prev[1], self.prev[2], self.geom)
            if vo['ok']:
                self.publish_vo(vo, ts - self.prev[3], stamp)
        self.pub_vo_status.publish(Float32MultiArray(
            data=[float(vo['tracked']), float(vo['inliers']), 1.0 if vo['ok'] else 0.0]))
        self.prev = (gray, fs['drivable'] & self.geom.valid, box_mask, ts)

        # ---- objects: ground contact -> odom, track, motion
        pose = self.odom_from_base(stamp)
        track_in = []
        dyn_pts = []
        for d in dets:
            x1, y1, x2, y2 = d['wbox']
            G, ok = self.geom.ground_points(np.array([0.5 * (x1 + x2)]), np.array([min(y2, H - 1)]))
            d['ground'] = G[0] if ok[0] and np.hypot(*G[0]) < 12 else None
            if d['ground'] is None:
                continue
            if d['dynamic']:
                dyn_pts.append(circle_points(G[0][0], G[0][1], d['radius']))
            if pose is not None and d['dynamic']:
                c, s = math.cos(pose[2]), math.sin(pose[2])
                d['odom'] = (pose[0] + c * G[0][0] - s * G[0][1], pose[1] + s * G[0][0] + c * G[0][1])
                track_in.append({'cls': d['cls'], 'radius': d['radius'],
                                 'x': d['odom'][0], 'y': d['odom'][1],
                                 'flow_resid': self.flow.object_residual(d['wbox'], self.geom)})
        tracks = self.tracker.update(track_in, ts) if pose is not None else []
        for d in dets:                       # label detections with their track (for the views)
            if 'odom' in d:
                for tr in tracks:
                    if tr.cls == d['cls'] and math.hypot(tr.x - d['odom'][0], tr.y - d['odom'][1]) < 0.05:
                        d['track_id'], d['moving'] = tr.id, self.tracker.is_moving(tr)

        # ---- publish
        sp = fs['static_pts']
        self.pub_obs.publish(cloud(np.c_[sp[:, :2], np.full(len(sp), 0.2)], self.base, stamp))
        fr, fc = np.nonzero(fs['free_mask'][::self.free_stride, ::self.free_stride])
        fr, fc = fr * self.free_stride, fc * self.free_stride
        self.pub_free.publish(cloud(np.c_[self.geom.GX[fr, fc], self.geom.GY[fr, fc],
                                          np.zeros(len(fr))], self.base, stamp))
        self.pub_dyn.publish(cloud(np.vstack(dyn_pts) if dyn_pts else np.zeros((0, 3)),
                                   self.base, stamp))
        rows = []
        for tr in tracks:
            rows += [tr.id, CLASS_IDS.get(tr.cls, -1), tr.x, tr.y, tr.vx, tr.vy, tr.radius,
                     1.0 if self.tracker.is_moving(tr) else 0.0]
        om = Float32MultiArray(data=[float(v) for v in rows])
        om.layout.dim = [MultiArrayDimension(label='tracks', size=len(tracks), stride=8 * len(tracks)),
                         MultiArrayDimension(label='fields', size=8, stride=8)]
        self.pub_objects.publish(om)
        sxf = msg.width / W
        self.pub_corr.publish(Float32MultiArray(data=[
            cor['heading'] if cor else 0.0, cor['range'] if cor else 0.0,
            cor['width'] if cor else 0.0,
            vp[0] * sxf if vp else -1.0, vp[1] * sxf if vp else -1.0, vp_conf]))
        if self.pub_overlay.get_subscription_count() > 0:
            self.publish_overlay(msg, rgb, fs, dets, tracks, cor, vp, vp_conf)
        self.publish_views(msg.header, rgb, labels, fs, dets, dyn_boxes, cor, vpinfo, vo,
                           box_mask, ts)

        t_end = time.perf_counter()
        self.timing.append((t_seg - t0, t_yolo - t_seg, t_end - t_yolo))
        if len(self.timing) >= 50:
            m = 1000 * np.mean(self.timing, axis=0)
            self.get_logger().info(
                f'rgb pipeline {m.sum():.0f} ms/frame (seg {m[0]:.0f}, yolo {m[1]:.0f}, '
                f'scan+flow+rest {m[2]:.0f}); obstacles {len(sp)} cols, VO inliers {vo["inliers"]}, '
                f'tracks {len(tracks)}')
            self.timing = []

    # ------------------------------------------------------------------ helpers
    def publish_vo(self, vo, dt, stamp):
        dx, dy, dyaw = vo['dx'], vo['dy'], vo['dyaw']
        vx, vy, wz = dx / dt, dy / dt, dyaw / dt
        if math.hypot(vx, vy) > 2.0 or abs(wz) > 2.5:
            return
        x, y, yaw = self.vo_pose
        mid = yaw + 0.5 * dyaw
        self.vo_pose = [x + dx * math.cos(mid) - dy * math.sin(mid),
                        y + dx * math.sin(mid) + dy * math.cos(mid), yaw + dyaw]
        var = 0.0025 * max(1.0, 80.0 / vo['inliers'])
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, self.odom, self.base
        o.pose.pose.position.x, o.pose.pose.position.y = self.vo_pose[0], self.vo_pose[1]
        o.pose.pose.orientation.z = math.sin(self.vo_pose[2] / 2)
        o.pose.pose.orientation.w = math.cos(self.vo_pose[2] / 2)
        pc = [0.0] * 36
        pc[0] = pc[7] = pc[35] = 1e3
        pc[14] = pc[21] = pc[28] = 1e6
        o.pose.covariance = pc
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = vx, vy, wz
        tc = [0.0] * 36
        tc[0] = tc[7] = var
        tc[35] = 2 * var
        tc[14] = tc[21] = tc[28] = 1e6
        o.twist.covariance = tc
        self.pub_vo.publish(o)

    def image_msg(self, header, img):
        out = Image()
        out.header = header
        out.height, out.width = img.shape[:2]
        out.encoding, out.step = 'rgb8', 3 * img.shape[1]
        out.data = array.array('B', np.ascontiguousarray(img).tobytes())
        return out

    def publish_views(self, header, rgb, labels, fs, dets, dyn_boxes, cor, vpinfo, vo, box_mask, ts):
        want_dash = self.pub_dashboard.get_subscription_count() > 0
        want = {n for n, p in self.pub_views.items() if p.get_subscription_count() > 0}
        dash_set = ['traversability', 'distance', 'vanishing', 'semantic', 'detections', 'flow']
        if want_dash:
            want |= set(dash_set)
        dt = ts - self.prev_ts if self.prev_ts is not None else 0.0
        self.prev_ts = ts
        if not want:
            return
        makers = {
            'semantic': lambda: views.semantic(rgb, labels, self.seg.labels),
            'traversability': lambda: views.traversability(rgb, fs, self.geom, dyn_boxes),
            'distance': lambda: views.distance(rgb, fs, self.geom, labels),
            'obstacles': lambda: views.obstacles(rgb, fs, self.geom, labels),
            'vanishing': lambda: views.vanishing(rgb, vpinfo, cor, self.geom, self.horizon_row),
            'detections': lambda: views.detections(rgb, fs, dets, self.horizon_row),
            'flow': lambda: views.flow(rgb, self.flow.last, box_mask, vo, dt),
        }
        imgs = {n: makers[n]() for n in want}
        for n, img in imgs.items():
            if self.pub_views[n].get_subscription_count() > 0:
                self.pub_views[n].publish(self.image_msg(header, img))
        if want_dash:
            self.pub_dashboard.publish(self.image_msg(header, views.dashboard([imgs[n] for n in dash_set])))

    def publish_overlay(self, msg, rgb, fs, dets, tracks, cor, vp, vp_conf):
        H, W = rgb.shape[:2]
        img = rgb.copy()
        tint = np.zeros_like(img)
        tint[fs['drivable']] = (0, 160, 0)
        tint[fs['free_mask']] = (0, 255, 120)
        img = cv2.addWeighted(img, 0.6, tint, 0.4, 0)
        for c, r in fs['static_px']:
            cv2.circle(img, (int(c), int(r)), 1, (255, 0, 0), -1)
        for c, r in fs['small_px'][::2]:
            cv2.circle(img, (int(c), int(r)), 1, (255, 220, 0), -1)
        for d in dets:
            x1, y1, x2, y2 = d['wbox'].astype(int)
            col = (255, 80, 0) if d['dynamic'] else (255, 200, 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), col, 1)
            label = d['cls']
            if d.get('ground') is not None:
                label += f' {np.hypot(*d["ground"]):.1f}m'
            cv2.putText(img, label, (x1, max(y1 - 3, 8)), 0, 0.35, col, 1)
        moving = [t for t in tracks if self.tracker.is_moving(t)]
        cv2.putText(img, f'tracks {len(tracks)}  moving {len(moving)}', (4, 12), 0, 0.35,
                    (255, 255, 255), 1)
        if cor:
            # heading arrow drawn from the bottom centre, length ~ free range
            P = np.array([[math.cos(cor['heading']) * min(cor['range'], 3.0),
                           math.sin(cor['heading']) * min(cor['range'], 3.0), 0.0]])
            px, ok = self.geom.project(P)
            if ok[0]:
                cv2.arrowedLine(img, (W // 2, H - 2), (int(px[0][0]), int(px[0][1])),
                                (0, 255, 255), 2, tipLength=0.08)
            cv2.putText(img, f'corridor {math.degrees(cor["heading"]):+.0f}deg '
                        f'{cor["range"]:.1f}m', (4, 24), 0, 0.35, (0, 255, 255), 1)
        if vp and vp_conf > 0.3:
            cv2.drawMarker(img, (int(vp[0]), int(vp[1])), (255, 0, 255), cv2.MARKER_CROSS, 12, 2)
        out = Image()
        out.header = msg.header
        out.height, out.width, out.encoding, out.step = H, W, 'rgb8', 3 * W
        out.data = array.array('B', np.ascontiguousarray(img).tobytes())
        self.pub_overlay.publish(out)


def main():
    rclpy.init()
    node = RgbPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
