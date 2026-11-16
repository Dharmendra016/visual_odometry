#!/usr/bin/env python3
"""
Steering-aware local planner + navigator for the RGB-only prototype.

  goal (RViz /goal_pose or NavigateToPose action)
    -> global path from Nav2 planner_server (Smac Hybrid-A*, global costmap), refreshed at 1 Hz
    -> carrot point ~1.8 m ahead on the path
    -> sample steering arcs (curvature limit + steering rate, 3 speeds), roll out 2.5 s
    -> reject arcs that hit the local evidence map or the predicted positions of moving objects
    -> score: progress to carrot, heading, clearance, moving-object clearance, corridor,
       smoothness, speed
    -> /cmd_vel_nav (-> collision_monitor -> safety_gate -> /cmd_vel)
  Blocked: stop, replan with a cleared global costmap, back up 0.4 m if the rear is clear,
  abort after `abort_after` seconds.

Inputs: /rgb/obstacles, /rgb/freespace (base_footprint clouds), /rgb/objects (odom tracks),
        /rgb/corridor, TF odom -> base_footprint.
"""
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, Twist  # noqa: E402
from nav2_msgs.action import ComputePathToPose, NavigateToPose  # noqa: E402
from nav2_msgs.srv import ClearEntireCostmap  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402
from std_msgs.msg import Float32MultiArray, String  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

from arcs import CIRCLE_R, circle_centers, rollout  # noqa: E402
from local_map import LocalMap  # noqa: E402


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SteeringPlanner(Node):

    def __init__(self):
        super().__init__('steering_planner')
        p = self.declare_parameter
        p('rate', 10.0)
        p('v_max', 0.4)
        p('min_turning_radius', 0.75)      # feasible ~0.64 m at +-35 deg; margin for tracking
        p('steer_rate', 2.5)               # curvature change, 1/m per s
        p('horizon', 2.5)
        p('n_curvatures', 21)
        p('lookahead', 1.8)
        p('goal_tolerance', 0.3)
        p('safety_margin', 0.05)           # min clearance (m) beyond the footprint circles
        p('dyn_margin', 0.25)              # extra clearance to moving objects
        p('replan_period', 1.0)
        p('clear_global_period', 15.0)
        p('abort_after', 45.0)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.v_max = gp('v_max')
        self.k_max = 1.0 / gp('min_turning_radius')
        self.k_rate = gp('steer_rate')
        self.horizon = gp('horizon')
        self.n_k = gp('n_curvatures')
        self.lookahead = gp('lookahead')
        self.goal_tol = gp('goal_tolerance')
        self.margin = gp('safety_margin')
        self.dyn_margin = gp('dyn_margin')
        self.replan_period = gp('replan_period')
        self.clear_period = gp('clear_global_period')
        self.abort_after = gp('abort_after')

        # tau 20 s (LocalMap default): people are kept out of the static map, so long memory is
        # safe; 6 s made rocks beside the robot (out of camera view) vanish and get driven into
        self.map = LocalMap()
        self.lock = threading.Lock()
        self.goal = None              # PoseStamped in odom
        self.path = None              # (N, 2)
        self.result = None            # 'succeeded' | 'aborted' | None
        self.k_cmd = 0.0
        self.tracks = np.zeros((0, 8))
        self.corridor = None
        self.blocked_since = None
        self.backup_until = None
        self.last_plan_req = 0.0
        self.last_clear = 0.0
        self.plan_busy = False
        self.state = 'IDLE'

        cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, '/rgb/obstacles', self.on_obstacles, qos, callback_group=cb)
        self.create_subscription(PointCloud2, '/rgb/freespace', self.on_free, qos, callback_group=cb)
        self.create_subscription(Float32MultiArray, '/rgb/objects', self.on_objects, 5, callback_group=cb)
        self.create_subscription(Float32MultiArray, '/rgb/corridor', self.on_corridor, 5, callback_group=cb)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal_topic, 5, callback_group=cb)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.arc_pub = self.create_publisher(MarkerArray, '/rgb_nav/arcs', 2)
        self.map_pub = self.create_publisher(OccupancyGrid, '/rgb_nav/local_map', 2)
        self.state_pub = self.create_publisher(String, '/rgb_nav/state', 5)
        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose', callback_group=cb)
        self.clear_client = self.create_client(ClearEntireCostmap,
                                               '/global_costmap/clear_entirely_global_costmap',
                                               callback_group=cb)
        self.nav_server = ActionServer(self, NavigateToPose, 'navigate_to_pose', self.execute_nav,
                                       goal_callback=lambda g: GoalResponse.ACCEPT,
                                       cancel_callback=lambda g: CancelResponse.ACCEPT,
                                       callback_group=cb)
        self.create_timer(1.0 / gp('rate'), self.control_loop, callback_group=cb)
        self.create_timer(0.5, self.publish_map, callback_group=cb)
        self.pending_free = []
        self.pending_obs = []
        self.get_logger().info('steering_planner ready (NavigateToPose server + /goal_pose)')

    # ------------------------------------------------------------------ inputs
    def robot_pose(self, stamp=None):
        try:
            tf = self.tf_buffer.lookup_transform('odom', 'base_footprint',
                                                 stamp if stamp else rclpy.time.Time())
        except Exception:  # noqa: BLE001
            try:
                tf = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            except Exception:  # noqa: BLE001
                return None
        t = tf.transform.translation
        return t.x, t.y, yaw_of(tf.transform.rotation)

    def cloud_to_odom(self, msg):
        pts = np.frombuffer(msg.data, np.float32).reshape(-1, 3)
        pose = self.robot_pose(msg.header.stamp)
        if pose is None or len(pts) == 0:
            return np.zeros((0, 2))
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        return np.stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]], 1)

    def on_obstacles(self, msg):
        pts = self.cloud_to_odom(msg)
        with self.lock:
            self.pending_obs.append(pts)

    def on_free(self, msg):
        pts = self.cloud_to_odom(msg)
        with self.lock:
            self.pending_free.append(pts)

    def on_objects(self, msg):
        d = np.array(msg.data, float)
        with self.lock:
            self.tracks = d.reshape(-1, 8) if len(d) else np.zeros((0, 8))

    def on_corridor(self, msg):
        if len(msg.data) >= 3 and msg.data[1] > 0:
            self.corridor = (msg.data[0], msg.data[1], msg.data[2])

    def on_goal_topic(self, msg):
        self.set_goal(msg)

    def set_goal(self, msg):
        with self.lock:
            self.goal = msg
            self.path = None
            self.result = None
            self.blocked_since = None
            self.backup_until = None
            self.last_plan_req = 0.0
        self.get_logger().info(f'new goal ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    # ------------------------------------------------------------------ action server
    def execute_nav(self, goal_handle):
        self.set_goal(goal_handle.request.pose)
        my_goal = self.goal
        fb = NavigateToPose.Feedback()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                with self.lock:
                    if self.goal is my_goal:
                        self.goal = None
                self.stop()
                goal_handle.canceled()
                return NavigateToPose.Result()
            if self.goal is not my_goal:         # pre-empted by a newer goal
                goal_handle.abort()
                return NavigateToPose.Result()
            if self.result is not None:
                res = self.result
                if res == 'succeeded':
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return NavigateToPose.Result()
            pose = self.robot_pose()
            if pose is not None:
                fb.distance_remaining = math.hypot(my_goal.pose.position.x - pose[0],
                                                   my_goal.pose.position.y - pose[1])
                goal_handle.publish_feedback(fb)
            time.sleep(0.1)
        return NavigateToPose.Result()

    # ------------------------------------------------------------------ global path
    def request_plan(self, now):
        if self.plan_busy or not self.plan_client.server_is_ready():
            return
        self.plan_busy = True
        self.last_plan_req = now
        req = ComputePathToPose.Goal()
        req.goal = self.goal
        req.goal.header.frame_id = 'odom'
        req.planner_id = 'GridBased'
        req.use_start = False
        fut = self.plan_client.send_goal_async(req)
        fut.add_done_callback(self.on_plan_accepted)

    def on_plan_accepted(self, fut):
        h = fut.result()
        if not h.accepted:
            self.plan_busy = False
            return
        h.get_result_async().add_done_callback(self.on_plan_result)

    def on_plan_result(self, fut):
        self.plan_busy = False
        res = fut.result().result
        poses = res.path.poses
        if poses:
            self.path = np.array([[p.pose.position.x, p.pose.position.y] for p in poses])

    def clear_global(self, now):
        self.last_clear = now
        if self.clear_client.service_is_ready():
            self.clear_client.call_async(ClearEntireCostmap.Request())

    def carrot(self, x, y):
        g = (self.goal.pose.position.x, self.goal.pose.position.y)
        if self.path is None or len(self.path) < 2:
            return g
        d = np.hypot(self.path[:, 0] - x, self.path[:, 1] - y)
        i = int(np.argmin(d))
        acc = 0.0
        while i < len(self.path) - 1 and acc < self.lookahead:
            acc += float(np.hypot(*(self.path[i + 1] - self.path[i])))
            i += 1
        return tuple(self.path[i])

    # ------------------------------------------------------------------ control
    def stop(self):
        self.cmd_pub.publish(Twist())

    def publish_state(self, s):
        if s != self.state:
            self.state = s
            self.state_pub.publish(String(data=s))
            self.get_logger().info(f'state: {s}')

    def control_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        pose = self.robot_pose()
        if pose is None:
            return
        x, y, th = pose
        with self.lock:
            obs, free = self.pending_obs, self.pending_free
            self.pending_obs, self.pending_free = [], []
            tracks = self.tracks.copy()
        self.map.recenter(x, y)
        self.map.decay(now)
        for f in free:
            self.map.insert(np.zeros((0, 2)), f)
        for o in obs:
            self.map.insert(o, np.zeros((0, 2)))
        self.map.update_distance()

        if self.goal is None or self.result is not None:
            self.publish_state('IDLE')
            return
        gx, gy = self.goal.pose.position.x, self.goal.pose.position.y
        dist_goal = math.hypot(gx - x, gy - y)
        if dist_goal < self.goal_tol:
            self.stop()
            self.result = 'succeeded'
            self.publish_state('GOAL_REACHED')
            return
        if now - self.last_plan_req > self.replan_period:
            self.request_plan(now)
        if now - self.last_clear > self.clear_period:
            self.clear_global(now)

        # ---- backing up after being blocked
        if self.backup_until is not None:
            if now < self.backup_until:
                t = Twist()
                t.linear.x = -0.15
                self.cmd_pub.publish(t)
                return
            self.backup_until = None

        cx, cy = self.carrot(x, y)
        if self.turn_around(x, y, th, cx, cy, dist_goal):
            return
        v_cap = min(self.v_max, 0.15 + 0.35 * dist_goal)
        speeds_set = [v_cap, 0.6 * v_cap, 0.3 * v_cap]
        k_t = np.linspace(-self.k_max, self.k_max, self.n_k)
        K = np.repeat(k_t, len(speeds_set))
        V = np.tile(speeds_set, len(k_t))
        poses, ks = rollout(x, y, th, self.k_cmd, K, V, self.horizon, 0.1, self.k_rate)
        centers = circle_centers(poses)                                    # (C, N, 3, 2)
        clear = self.map.clearance(centers[..., 0], centers[..., 1]) - CIRCLE_R
        min_clear = clear.min(axis=(1, 2))
        # Escape rule: if the robot is already closer than the margin (e.g. a rock came back
        # into view right beside it), arcs that do not make clearance worse are allowed -
        # otherwise every arc is "colliding" and the robot deadlocks.
        here = circle_centers(np.array([[[x, y, th]]]))
        cur_clear = float((self.map.clearance(here[..., 0], here[..., 1]) - CIRCLE_R).min())
        threshold = min(self.margin, cur_clear - 0.01)
        feasible = min_clear > threshold

        # moving objects: predicted positions over the horizon
        min_dyn = np.full(len(K), 9.0)
        N = poses.shape[1]
        tk = (np.arange(N) + 1) * 0.1
        for tr in tracks:
            _, _, ox, oy, ovx, ovy, rad, moving = tr
            if not moving:
                ovx = ovy = 0.0
            px = ox + ovx * tk
            py = oy + ovy * tk
            d = np.hypot(centers[..., 0] - px[None, :, None], centers[..., 1] - py[None, :, None])
            d = (d - CIRCLE_R - rad).min(axis=2)                           # (C, N)
            early = d[:, :int(1.5 / 0.1)].min(axis=1)
            min_dyn = np.minimum(min_dyn, d.min(axis=1))
            feasible &= early > self.dyn_margin

        end = poses[:, -1]
        c_goal = np.hypot(end[:, 0] - cx, end[:, 1] - cy)
        bearing = np.arctan2(cy - end[:, 1], cx - end[:, 0])
        c_head = np.abs(wrap(bearing - end[:, 2])) * (c_goal > 0.3)
        c_clear = np.maximum(0.0, 0.6 - min_clear)
        c_dyn = np.maximum(0.0, 1.2 - min_dyn)
        c_corr = np.zeros(len(K))
        if self.corridor is not None:
            ch, crange, cwidth = self.corridor
            heading_odom = th + ch
            carrot_bearing = math.atan2(cy - y, cx - x)
            if abs(wrap(carrot_bearing - heading_odom)) < max(cwidth / 2, 0.2) and crange > 1.5:
                c_corr = np.abs(wrap(end[:, 2] - heading_odom))
        c_smooth = np.abs(K - self.k_cmd) / self.k_max
        c_speed = (self.v_max - V) / self.v_max
        cost = 2.0 * c_goal + 0.4 * c_head + 4.0 * c_clear + 3.0 * c_dyn + 0.2 * c_corr \
            + 0.3 * c_smooth + 0.6 * c_speed
        cost[~feasible] = np.inf

        best = int(np.argmin(cost))
        if not np.isfinite(cost[best]):
            self.handle_blocked(now, x, y, th)
            self.publish_arcs(poses, feasible, None)
            return
        self.blocked_since = None
        self.get_logger().debug(
            f'pose ({x:.2f},{y:.2f},{math.degrees(th):.0f}deg) carrot ({cx:.2f},{cy:.2f}) '
            f'path {0 if self.path is None else len(self.path)} pts; best k {K[best]:+.2f} '
            f'v {V[best]:.2f}; feasible {int(feasible.sum())}/{len(K)}; '
            f'cost goal {c_goal[best]:.2f} head {c_head[best]:.2f} clear {c_clear[best]:.2f} '
            f'corr {c_corr[best]:.2f}', throttle_duration_sec=1.0)
        self.k_cmd = float(ks[best, 0])
        t = Twist()
        t.linear.x = float(V[best])
        t.angular.z = float(V[best] * self.k_cmd)
        self.cmd_pub.publish(t)
        self.publish_state('DRIVING')
        self.publish_arcs(poses, feasible, best)

    def arc_clear(self, x, y, th, k, v, horizon):
        """True if a constant-target arc keeps the footprint clear in the local map."""
        poses, ks = rollout(x, y, th, self.k_cmd, np.array([k]), np.array([v]), horizon, 0.1,
                            self.k_rate)
        c = circle_centers(poses)
        here = circle_centers(np.array([[[x, y, th]]]))
        cur = float((self.map.clearance(here[..., 0], here[..., 1]) - CIRCLE_R).min())
        ok = (self.map.clearance(c[..., 0], c[..., 1]) - CIRCLE_R).min() > min(self.margin, cur - 0.01)
        return ok, float(ks[0, 0])

    def turn_around(self, x, y, th, cx, cy, dist_goal):
        """
        Target behind the robot (> 100 deg): arc sampling cannot rank the options (every arc
        ends about as far from it), so do an explicit steering-limited manoeuvre:
        full-lock forward turn towards the target's side; if there is no room ahead, reverse
        with opposite lock (three-point / K-turn). Reverse relies on the remembered map behind
        the robot - the camera cannot see there - so reverse moves are short.
        """
        beta = wrap(math.atan2(cy - y, cx - x) - th)
        if abs(beta) < math.radians(100) or dist_goal < 1.0:
            self.turn_dir = None
            return False
        side = 1.0 if beta > 0 else -1.0
        if getattr(self, 'turn_dir', None) is None:
            self.turn_dir = side                   # keep the side once chosen (no dithering)
        side = self.turn_dir
        ok, k_now = self.arc_clear(x, y, th, side * self.k_max, 0.25, 1.5)
        t = Twist()
        if ok:
            t.linear.x, t.angular.z = 0.25, 0.25 * k_now
            self.k_cmd = k_now
            self.publish_state('TURNING_AROUND')
        else:
            ok, k_now = self.arc_clear(x, y, th, -side * self.k_max, -0.15, 1.2)
            if not ok:
                return False                        # let the normal planner / blocked logic run
            t.linear.x, t.angular.z = -0.15, -0.15 * k_now
            self.k_cmd = k_now
            self.publish_state('K_TURN_REVERSE')
        self.blocked_since = None
        self.cmd_pub.publish(t)
        return True

    def handle_blocked(self, now, x, y, th):
        self.stop()
        self.k_cmd = 0.0
        if self.blocked_since is None:
            self.blocked_since = now
            self.publish_state('BLOCKED')
            self.clear_global(now)
            self.request_plan(now)
            return
        waited = now - self.blocked_since
        if waited > self.abort_after:
            self.result = 'aborted'
            self.publish_state('ABORTED')
            return
        if waited > 3.0 and int(waited) % 6 == 3:
            # back up straight if the remembered map behind is clear
            back = rollout(x, y, th, 0.0, np.array([0.0]), np.array([-0.15]), 2.7, 0.1, 1.0)[0]
            c = circle_centers(back)
            here = circle_centers(np.array([[[x, y, th]]]))
            cur = float((self.map.clearance(here[..., 0], here[..., 1]) - CIRCLE_R).min())
            if (self.map.clearance(c[..., 0], c[..., 1]) - CIRCLE_R).min() > min(self.margin, cur - 0.01):
                self.backup_until = now + 2.7
                self.publish_state('BACKING_UP')

    # ------------------------------------------------------------------ visualisation
    def publish_arcs(self, poses, feasible, best):
        if self.arc_pub.get_subscription_count() == 0:
            return
        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'arcs', 0, Marker.LINE_LIST, Marker.ADD
        m.scale.x = 0.01
        m.pose.orientation.w = 1.0
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA
        for i in range(0, len(poses), 3):              # one speed per curvature
            col = ColorRGBA(r=0.1, g=0.8, b=0.1, a=0.6) if feasible[i] else ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.4)
            for a, b in zip(poses[i, :-1:2], poses[i, 2::2]):
                m.points += [Point(x=a[0], y=a[1], z=0.05), Point(x=b[0], y=b[1], z=0.05)]
                m.colors += [col, col]
        ma.markers.append(m)
        if best is not None:
            b = Marker()
            b.header = m.header
            b.ns, b.id, b.type, b.action = 'best', 1, Marker.LINE_STRIP, Marker.ADD
            b.scale.x = 0.04
            b.pose.orientation.w = 1.0
            b.color = ColorRGBA(r=0.1, g=0.4, b=1.0, a=1.0)
            b.points = [Point(x=p[0], y=p[1], z=0.08) for p in poses[best]]
            ma.markers.append(b)
        self.arc_pub.publish(ma)

    def publish_map(self):
        if self.map.ox is None or self.map_pub.get_subscription_count() == 0:
            return
        g = OccupancyGrid()
        g.header.frame_id = 'odom'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.resolution = self.map.res
        g.info.width = g.info.height = self.map.n
        g.info.origin.position.x, g.info.origin.position.y = self.map.ox, self.map.oy
        g.info.origin.orientation.w = 1.0
        g.data = self.map.occupancy_grid_data().ravel().tolist()
        self.map_pub.publish(g)


def main():
    rclpy.init()
    node = SteeringPlanner()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
