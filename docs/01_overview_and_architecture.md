# 1. Overview & architecture

## 1.1 What the system does

1. The operator clicks **2D Goal Pose** in RViz (or sends a `NavigateToPose` action).
2. The camera image is turned into metric 3D points (Depth Anything V2 + ground-plane scale).
3. Points are split into *obstacle* and *ground*; Nav2 costmaps are built from them.
4. The robot's motion is estimated from wheels + IMU + visual odometry (EKF).
5. Smac Hybrid-A* plans a car-like path; MPPI follows it while scoring the local costmap.
6. Two independent safety layers can stop the robot; the 4WS kinematics node turns the final
   velocity command into four steering angles and four wheel speeds.

There is **no GPS and no map frame** in this phase. Everything is expressed in the `odom` frame,
whose origin is where the robot was when the EKF started. Goals are therefore relative to the
start position.

## 1.2 Block diagram

```
                 ┌──────────────────────────── Gazebo (or real hardware) ───────────────────────────┐
                 │ /camera/image_raw  /camera/camera_info  /imu/data  /joint_states  (/gps/fix unused)│
                 └───────┬───────────────────────────┬──────────────┬───────────────┬───────────────┘
                         │                           │              │               │
              ┌──────────▼──────────┐                │      ┌───────▼────────┐      │
              │ depth_node          │                │      │ wheel_odometry │      │
              │ DA-V2 → scale fit   │                │      │ 4WS least sq.  │      │
              │ → clouds            │                │      └───────┬────────┘      │
              └──┬────────┬──────┬──┘                │        /wheel/odom          │
   /perception/  │        │      │ /perception/depth │              │               │
   obstacles     │ ground │      └──────►┌───────────▼────┐         │               │
                 │        │              │ visual_odometry│ /vo/odom│               │
                 │        │              └───────┬────────┘         │               │
                 │        │                      └────────►┌────────▼───────┐◄──────┘ /imu/data
                 │        │                                │ ekf_filter_node│
                 │        │                                └────────┬───────┘
                 │        │                    TF odom→base_footprint, /odometry/filtered
          ┌──────▼────────▼──────┐                                  │
          │ Nav2 costmaps        │◄─────────────────────────────────┘
          │ (local 6×6 m rolling,│
          │  global 50×50 m)     │      /goal_pose (RViz) or NavigateToPose action
          └──────┬───────────────┘                 │
                 │                         ┌───────▼───────┐
                 └────────────────────────►│ bt_navigator  │  behaviour tree (no spin)
                                           └──┬─────────┬──┘
                                  planner_server         controller_server
                                  Smac Hybrid-A*          MPPI (Ackermann)
                                                     │  /cmd_vel_nav   (also behavior_server BackUp)
                                           ┌─────────▼─────────┐
         /perception/obstacles ───────────►│ collision_monitor │  stop / slow polygons
                                           └─────────┬─────────┘
                                                     │  /cmd_vel_safe
                                           ┌─────────▼─────────┐
   /perception/obstacles (freshness), IMU ►│ safety_gate       │  stale perception / tilt ⇒ zero
                                           └─────────┬─────────┘
                                                     │  /cmd_vel     ◄── teleop_twist_keyboard (bypasses both)
                                           ┌─────────▼─────────┐
                                           │ four_ws_kinematics│  watchdog 0.5 s
                                           └──┬─────────────┬──┘
                    /steering_controller/commands       /wheel_controller/commands
                                           (ros2_control → Gazebo / real drivers)
```

## 1.3 Nodes

| Node | Package | Language | Role |
|---|---|---|---|
| `gzserver`, `gzclient` | gazebo_ros | C++ | physics + sensors (sim only) |
| `controller_manager` (+ joint_state_broadcaster, steering_controller, wheel_controller) | ros2_control | C++ | joint command/state interface |
| `robot_state_publisher` | — | C++ | static TF from URDF |
| `four_ws_kinematics` | rover4ws_gazebo | Python | `/cmd_vel` → 4 steer angles + 4 wheel speeds |
| `wheel_odometry` | rover4ws_localization | Python | joint states → body twist + slip residual |
| `ekf_filter_node` | robot_localization | C++ | fuses wheels, IMU, VO → `odom→base_footprint` |
| `depth_node` | rover4ws_perception | Python + PyTorch | monocular metric depth + obstacle/ground clouds |
| `visual_odometry` | rover4ws_perception | Python + OpenCV | body twist from camera motion |
| `planner_server`, `controller_server`, `behavior_server`, `bt_navigator`, `collision_monitor`, `lifecycle_manager_navigation` | Nav2 | C++ | navigation |
| `safety_gate` | rover4ws_navigation | Python | last software gate before motors |
| `rviz2` | — | C++ | operator UI |

## 1.4 Topics (contract)

| Topic | Type | Publisher → Subscribers | Notes |
|---|---|---|---|
| `/camera/image_raw` | Image rgb8 640×480 ~23 Hz (sim) | Gazebo → depth_node, visual_odometry, RViz | frame `camera_optical_frame` |
| `/camera/camera_info` | CameraInfo | Gazebo → depth_node, VO | intrinsics |
| `/camera_gt/depth/image_raw` | Image 32FC1 320×240 10 Hz | Gazebo (only if `gt_depth:=true`) → depth_node (`source:=gt`), eval_depth | **sim-only reference sensor** |
| `/imu/data` | Imu 100 Hz | Gazebo → EKF, safety_gate | |
| `/joint_states` | JointState 100 Hz | joint_state_broadcaster → wheel_odometry, robot_state_publisher | wheel encoders + steering angles |
| `/gps/fix` | NavSatFix 5 Hz | Gazebo → (nobody yet) | for the future GPS phase |
| `/ground_truth/odom` | Odometry 50 Hz | Gazebo p3d → test tools only | **never used for control** |
| `/wheel/odom` | Odometry | wheel_odometry → EKF | twist used, pose is for evaluation |
| `/wheel/slip` | Float64 | wheel_odometry | RMS residual of the rigid-body fit |
| `/perception/depth` | Image 32FC1 640×480 ~16 Hz | depth_node → VO, RViz | same header stamp as the RGB frame |
| `/perception/obstacles` | PointCloud2 (base_footprint) | depth_node → costmaps, collision_monitor, safety_gate | |
| `/perception/ground` | PointCloud2 (base_footprint) | depth_node → costmaps (clearing only) | |
| `/perception/scale_fit` | Float32MultiArray `[a, b, inlier_frac, valid]` | depth_node | diagnostics |
| `/vo/odom` | Odometry | visual_odometry → EKF | published only while tracking is healthy |
| `/vo/status` | Float32MultiArray `[tracked, inliers, healthy]` | visual_odometry | diagnostics |
| `/odometry/filtered` | Odometry 50 Hz | EKF → Nav2 | |
| `/goal_pose` | PoseStamped | RViz → bt_navigator | frame `odom` |
| `/plan` | Path | planner_server | global path |
| `/global_costmap/costmap`, `/local_costmap/costmap` | OccupancyGrid | Nav2 | |
| `/cmd_vel_nav` | Twist | controller_server, behavior_server → collision_monitor | |
| `/cmd_vel_safe` | Twist | collision_monitor → safety_gate | |
| `/cmd_vel` | Twist | safety_gate (or teleop) → four_ws_kinematics | |
| `/polygon_stop`, `/polygon_slowdown` | PolygonStamped | collision_monitor → RViz | |
| `/safety/state` | String | safety_gate | `GO` / `STOP_STALE_PERCEPTION` / `STOP_TILT` (on change) |
| `/steering_controller/commands` | Float64MultiArray [FL, FR, RL, RR] rad | four_ws_kinematics → ros2_control | |
| `/wheel_controller/commands` | Float64MultiArray [FL, FR, RL, RR] rad/s | four_ws_kinematics → ros2_control | |

## 1.5 TF tree

```
odom ──(EKF, 50 Hz)──► base_footprint ──► base_link ──► chassis_link ──► battery_link, estop_link
                                                     ├─► camera_mount_link
                                                     ├─► camera_link ──► camera_optical_frame
                                                     ├─► imu_link
                                                     ├─► gps_link
                                                     └─► {fl,fr,rl,rr}_steer_link ──► {..}_wheel_link
```
- REP-103/105: x forward, y left, z up. `camera_optical_frame`: z forward, x right, y down.
- No `map` frame yet. The Gazebo `world` frame only appears in `/ground_truth/odom`.
- `odom` = world only if the robot spawns at (0,0,0). Test tools convert between them.

## 1.6 Launch structure

```
rover4ws_navigation/prototype.launch.py      ← the one you run
├── (scoped) rover4ws_gazebo/sim.launch.py   Gazebo, RSP, spawn, controllers, four_ws_kinematics
├── (scoped) rover4ws_localization/localization.launch.py   wheel_odometry, EKF (ekf_vo.yaml or ekf.yaml)
├── (scoped) rover4ws_perception/perception.launch.py       depth_node, visual_odometry
├── TimerAction 8 s → (scoped) rover4ws_navigation/nav.launch.py   Nav2 servers, collision_monitor,
│                                                                   lifecycle manager, safety_gate
└── rviz2 (rover4ws_navigation/rviz/prototype.rviz)
```
Includes are wrapped in scoped `GroupAction`s — otherwise arguments passed to an include (e.g.
`rviz:=false` to sim.launch.py) leak back and override the parent's arguments (this bug once
prevented RViz from opening).

| Argument | Default | Meaning |
|---|---|---|
| `world` | `test_yard` | `test_yard` or `stone_yard` (file in rover4ws_gazebo/worlds) |
| `depth_source` | `model` | `model` = Depth Anything V2, `gt` = sim perfect depth camera |
| `gt_depth` | auto (`true` iff `depth_source:=gt`) | spawn the perfect depth camera (costs camera FPS) |
| `use_vo` | `true` | fuse visual odometry (`ekf_vo.yaml`) or not (`ekf.yaml`) |
| `gui`, `rviz` | `true` | Gazebo client / RViz |
| `x`, `y`, `yaw` | `0` | spawn pose in the world |

Readiness: wait for `depth pipeline … ms/frame` (perception live) and `Managed nodes are active`
(Nav2 up) in the log — about 20–30 s.

## 1.7 Command chain and who can stop the robot

| Layer | Trigger | Effect |
|---|---|---|
| MPPI CostCritic | trajectories through high-cost / lethal cells | chooses another trajectory |
| collision_monitor PolygonSlow | > 5 obstacle points (`max_points: 5`) up to 1.2 m ahead of the bumper, ±0.70 m | speed × 0.4 |
| collision_monitor PolygonStop | > 5 obstacle points in 0–0.40 m ahead of the bumper, ±0.32 m | zero |
| safety_gate | no `/perception/obstacles` for 0.5 s, or roll/pitch > 20° | zero |
| four_ws_kinematics watchdog | no `/cmd_vel` for 0.5 s | wheels stop |
| hardware e-stop (real robot) | button | must cut motor power physically |

Manual teleop publishes straight to `/cmd_vel` and **bypasses** collision_monitor and safety_gate.
