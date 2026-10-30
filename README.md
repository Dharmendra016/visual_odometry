# rover4ws — camera-only navigation prototype (4-wheel steering)

> **Detailed documentation:** [docs/](docs/README.md) — architecture, every node and parameter,
> test methodology and baseline results, design decisions, environment gotchas, real-robot roadmap.

Click a goal in RViz → the rover builds a map from its single camera, plans a car-like path
and drives there avoiding obstacles. No LiDAR, no GPS, no prior map.

Perception follows DRISHTI's ideas (Depth Anything V2 + ground-plane metric scale, unseen ≠ free);
the system architecture follows monocular-depth-navigation (perception → standard ROS clouds → Nav2).

```
camera ─► depth_node: Depth Anything V2 ─► metric scale from known ground plane ─► 3D points
                                              │                         │
                                     /perception/ground        /perception/obstacles (z > 6 cm, holes < -10 cm)
                                              └──────────┬──────────────┘
                                                Nav2 costmaps (unknown stays unknown)
camera + metric depth ─► visual_odometry ─┐              │
joint_states ─► wheel_odometry (4WS LSQ) ─┼─► EKF ─► odom → base_footprint
IMU yaw rate ─────────────────────────────┘              │
RViz "2D Goal Pose" ─► Smac Hybrid-A* (Dubins, R ≥ 0.9 m) ─► MPPI (Ackermann, scores costmap)
   ─► /cmd_vel_nav ─► collision_monitor (stop 0.40 m ahead ±0.32 m / slow 1.2 m ahead ±0.70 m) ─► /cmd_vel_safe
   ─► safety_gate (stale perception > 0.5 s or tilt > 20° ⇒ STOP) ─► /cmd_vel ─► four_ws_kinematics
```

| Package | Contents |
|---|---|
| `rover4ws_description` | URDF/xacro (camera, IMU, GPS, wheel encoders; sim-only perfect depth camera) |
| `rover4ws_gazebo` | test world, ros2_control, `four_ws_kinematics.py` (cmd_vel → 4 steer angles + 4 wheel speeds) |
| `rover4ws_localization` | `wheel_odometry.py` (4WS least-squares + slip residual), EKF configs |
| `rover4ws_perception` | `depth_node.py` (DA-V2, scale fit, clouds), `visual_odometry.py` (LK + PnP-RANSAC) |
| `rover4ws_navigation` | Nav2 config, behaviour trees (no spin-in-place), `safety_gate.py`, `prototype.launch.py`, `tools/` |

## Run it

Ubuntu (WSL) terminal, not PowerShell:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch rover4ws_navigation prototype.launch.py
```

Wait ~20 s (Gazebo + Depth Anything load; the log prints `depth pipeline … ms/frame` when perception is live).
In RViz click **2D Goal Pose**, click the target and **drag the arrow in the direction you want the rover
to face on arrival** — this is a car-like robot, so asking it to arrive facing backwards forces a loop.

RViz shows: camera + metric depth images, red obstacle points, green ground points, global/local
costmaps (grey = unknown), blue planned path, red/yellow stop/slow zones.

| Launch argument | Default | |
|---|---|---|
| `world` | `test_yard` | `stone_yard` = gravel, large rocks, walking person, moving cart |
| `depth_source` | `model` | `gt` = sim-only perfect depth camera (to separate planning bugs from perception bugs) |
| `use_vo` | `true` | fuse visual odometry into the EKF |
| `gui` / `rviz` | `true` | Gazebo window / RViz |
| `x`, `y`, `yaw` | `0` | spawn pose (world). The odom frame starts at the spawn pose. |
| `gt_depth` | auto | add the perfect depth camera (needed for `tools/eval_depth.py`) |

Manual driving (bypasses Nav2 and both safety layers): `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.

### Stone yard — stones, rocks and moving objects

```bash
ros2 launch rover4ws_navigation prototype.launch.py world:=stone_yard
```
Suggested goal: (14, 0) at the far brick wall, then back to the start.

| Object | What should happen |
|---|---|
| gravel patch (x 1.5–4.5): 40 small stones, 1.5–3.5 cm | rover drives **over** them (below the 6 cm step limit) |
| 7 large rocks, 35–65 cm | rover plans **around** them |
| walker — person crossing the path at x = 7.5, 0.6 m/s | rover slows (yellow zone) and stops (red zone), then continues |
| cart — orange box crossing at x = 11, 0.4 m/s | same |

The movers are scripted Gazebo actors: the camera sees them but they have **no collision and never
dodge** — if the rover has stopped in their lane they walk straight through it. Rearrange the yard by
editing `rover4ws_gazebo/worlds/make_stone_yard.py` and running it (`python3 make_stone_yard.py`), then rebuild.

Moving objects leave "ghost" trails in the map (cells are only cleared when the camera looks at them
again). The navigator therefore forgets its *global* map every 10 s and on any planning failure;
the local map used for avoidance is kept.

## Test results (simulation, this machine: RTX 4060 laptop, WSL2)

All measured against Gazebo ground truth with the scripts in `rover4ws_navigation/tools/`.

**Depth** — `eval_depth.py`, 40 frames while driving, 0.3–4 m:

| | AbsRel | δ<1.25 |
|---|---|---|
| all pixels | 3.1 % | 99.8 % |
| ground | 2.5 % | |
| obstacles | 9.1 % | |

Pipeline 23–25 ms/frame (model 15 ms, scale fit 5 ms, clouds 3 ms) → depth at ~16 Hz (camera ~23 Hz).

**Odometry** — `eval_odometry.py`, final position error as % of distance driven:

| | wheels | visual odometry | EKF (wheels+IMU) | EKF (wheels+IMU+VO) |
|---|---|---|---|---|
| grass loop, 10.3 m | 1.6 % | 1.5 % | 0.1 % | 0.4 % |
| ice patch (μ = 0.05), 5.6 m | 5.8 % | 1.4 % | 5.7 % | **3.1 %** |

VO roughly halves the drift when wheels slip; on perfect sim grass it adds a little noise.

**Navigation** — `nav_test.py`, camera-only (Depth Anything + VO), three goals in a row:

| goal (world) | result | time | true final error | closest obstacle |
|---|---|---|---|---|
| (12, 0) through the course | succeeded | 59 s | 0.24 m | 0.21 m |
| (1, 2.5) — U-turn back | succeeded | 32 s | 0.43 m | 0.46 m |
| (9.5, −2.5) — through the gap | succeeded | 70 s | 0.32 m | 0.22 m |

**Stone yard** — `nav_test.py --world stone_yard`, three crossings in a row (14,0) → (0.5,0) → (14,0):

| | result | time | gravel | closest rock | movers |
|---|---|---|---|---|---|
| run 1 | succeeded | 48 s | drove over | ≥ 0.27 m | walker contact while rover **stopped** (0.00 m/s) |
| run 2 | succeeded | 137 s | drove over | ≥ 0.27 m | walker 0.22 m, cart 1.87 m |
| run 3 | succeeded | 119 s | drove over | ≥ 0.27 m | cart contact from the side while rover **stopped** |

The rover never drove into a mover; both contacts were the scripted actor walking into a stopped rover.
`nav_test.py` labels each contact (`COLLISION - rover drove into it` vs `CONTACT - … mover walked into it`).
Times vary a lot because the rover waits for the movers to pass.

**Safety** — `safety_test.py` (planner bypassed, blind "forward" command):
sudden box 2.5 m ahead → stopped 0.45 m before it · box removed → resumed · depth node killed → stopped in 0.6 s.

Reproduce (each needs the prototype running headless in another terminal):
```bash
cd ~/ros2_ws/src/rover4ws_navigation/tools
# ros2 launch rover4ws_navigation prototype.launch.py gui:=false rviz:=false gt_depth:=true
python3 eval_depth.py --frames 40 --drive
# ... x:=-8.0 y:=-10.0        (open space)
python3 eval_odometry.py --pattern loop
python3 safety_test.py
# ... x:=1.5 y:=7.0           (ice patch)
python3 eval_odometry.py --pattern ice
# ... (default spawn)
python3 nav_test.py --x 12 --y 0
```

## Known limitations — read before trusting it on the real rover

- **Gazebo images are not real images.** Depth Anything's 3 % error is on clean renders. Record rosbags
  of the real camera and re-run the comparison (Marigold offline is a good reference) before driving.
- **Flat-ground assumption.** Metric scale comes from the camera's known height above a flat ground
  plane. On slopes/ramps the scale fit degrades; when it fails the node publishes nothing and the safety
  gate stops the rover (by design).
- **Clearance is tight** (0.21 m closest). For the real robot raise `inflation_radius` and the stop zone.
- **Camera FOV ~100°, forward only.** Obstacles beside/behind are remembered in the costmap but not
  re-checked; the collision monitor only protects the front. Nothing can see a person approaching
  from the side or behind — the real robot needs at least a rear camera or bumpers.
- **No motion prediction.** Moving objects are treated as obstacles where they are *now*; the rover
  stops for them but does not predict where they are going or move out of their way.
- **No spin-in-place / pure sideways.** With ±35° steering the tightest turn is ~0.64 m radius; planner uses 0.9 m.
- **No terrain classes yet.** Traversability is geometric (step > 6 cm = obstacle). Semantic classes
  (grass/mud/water) need real-camera data — segmentation models are meaningless on Gazebo textures.
- **Odom frame only** — goals are relative to where the rover started. GPS/map frame comes later.
- **Real hardware:** replace the `<hardware>` plugin in `ros2_control.xacro`; measure and update
  `properties.xacro`; calibrate the camera. A Wi-Fi IP camera's latency (200–800 ms) will break this —
  use USB/CSI.

## Environment notes

- PyTorch 2.14 (CUDA), transformers 4.49 and `numpy<2` are installed with `pip --user`;
  `setuptools` is pinned `<80` because colcon needs it.
- Files edited from Windows lose the Linux executable bit; every `colcon build` restores it.
- Keep `": "` out of xacro comments — gazebo_ros2_control fails to parse the URDF otherwise.
