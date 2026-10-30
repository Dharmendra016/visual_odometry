# 7. Testing & results

Every claim about the system is measured against Gazebo ground truth (`/ground_truth/odom`, the
perfect depth camera, known obstacle geometry). Scripts live in `rover4ws_navigation/tools/` and are
run with `python3` (not installed as executables).

## 7.1 How to run a test

Terminal 1 (headless is fine and faster):
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch rover4ws_navigation prototype.launch.py gui:=false rviz:=false [args]
```
Wait ~30 s, then terminal 2:
```bash
cd ~/ros2_ws/src/rover4ws_navigation/tools && python3 <script> [args]
```

| Script | Launch args | Command | Measures |
|---|---|---|---|
| `eval_depth.py` | `gt_depth:=true` | `python3 eval_depth.py --frames 40 --drive` | AbsRel, δ<1.25 of `/perception/depth` vs perfect depth (0.3–4 m), ground vs obstacles |
| `eval_odometry.py` | `x:=-8.0 y:=-10.0` | `python3 eval_odometry.py --pattern loop` | wheel / VO / EKF final error vs ground truth |
| `eval_odometry.py` | `x:=1.5 y:=7.0` | `python3 eval_odometry.py --pattern ice` | same, on the μ = 0.05 patch |
| `nav_test.py` | (default) | `python3 nav_test.py --x 12 --y 0` | Nav2 result, time, true final error, min clearance per obstacle |
| `nav_test.py` | `world:=stone_yard` | `python3 nav_test.py --world stone_yard --x 14 --y 0` | + gravel crossed?, clearance to movers, who moved into whom |
| `safety_test.py` | `x:=-8.0 y:=-10.0` | `python3 safety_test.py` | stop for sudden obstacle, resume, stop on perception loss |

Notes
- `eval_odometry.py` publishes `/cmd_vel` directly (Nav2 idle); patterns are lists of
  `(seconds, vx, wz)` at the top of the file.
- `nav_test.py` takes **world** coordinates and converts them to odom using the start pose
  (`odom` origin = spawn pose). Final heading defaults to the bearing robot → goal; `--yaw` overrides.
- `safety_test.py` pushes a blind forward command into `/cmd_vel_nav` (planner bypassed), spawns a
  box 2.5 m ahead with `/spawn_entity`, deletes it, then `pkill`s `depth_node.py`.
  It kills the depth node — restart the launch afterwards.
- Run several goals in one session to test accumulated drift:
  `for g in '14 0' '0.5 0' '14 0'; do set -- $g; python3 nav_test.py --world stone_yard --x $1 --y $2; done`

## 7.2 Baseline results (2026-10-30, RTX 4060 laptop, WSL2, Gazebo Classic 11)

### Depth (Depth Anything V2-Small + ground-plane scale)
| Pixels (0.3–4 m) | AbsRel | δ < 1.25 |
|---|---|---|
| all | 3.1 % | 99.8 % |
| ground | 2.5 % | |
| obstacles | 9.1 % | |
Pipeline 23–25 ms/frame → ~16 Hz.

### Odometry (% of distance)
| Scenario | Wheels | VO | EKF w+IMU | EKF w+IMU+VO |
|---|---|---|---|---|
| grass loop 10.3 m | 1.6 | 1.5 | 0.1 | 0.4 |
| ice 5.6 m | 5.8 | 1.4 | 5.7 | 3.1 |

### Navigation — test_yard, camera-only, 3 goals in one session
| Goal (world) | Result | Time | Notes |
|---|---|---|---|
| (12, 0) | succeeded | 35 s | through crates, around wall |
| (1.0, 2.5) | succeeded | 129 s | U-turn back through the course |
| (9.5, −2.5) | succeeded | 35 s | |
Earlier full measurement with MPPI (before periodic global clearing; stop zone ±0.30 m): times
59 / 32 / 70 s, final errors 0.24–0.43 m, min clearance 0.21–0.46 m, no collisions.
With the earlier Regulated-Pure-Pursuit controller, goal (12, 0) took 34 s with perfect depth
(`depth_source:=gt`) vs 35 s with Depth Anything — in this world the network performed about as
well as perfect depth.

### Navigation — stone_yard, 3 crossings (14,0) → (0.5,0) → (14,0)
| Run | Result | Time | Gravel | Closest rock | Movers |
|---|---|---|---|---|---|
| 1 | succeeded | 48 s | drove over | ≥ 0.27 m | walker reached the **stopped** robot (0.00 m/s) |
| 2 | succeeded | 137 s | drove over | ≥ 0.27 m | walker 0.22 m, cart 1.87 m |
| 3 | succeeded | 119 s | drove over | ≥ 0.27 m | cart reached the **stopped** robot from the side |
The robot never drove into a mover; both contacts were scripted actors walking into a stopped robot.
Times vary because the robot waits for movers.

### Safety
| Test | Result |
|---|---|
| sudden box 2.5 m ahead, blind forward command | stopped 0.45 m before it |
| box removed | resumed at 0.40 m/s |
| depth_node killed while driving | stopped after 0.6 s |
| RViz `/goal_pose` path | drove from (0,0) to within 0.16 m of (6, −3) |

## 7.4 RGB-only prototype (v0.2) results — 2026-10-31

Launch `rover4ws_rgb_navigation rgb_prototype.launch.py`; same tools (`safety_test.py --kill rgb_perception_node.py`).

| Test | Result |
|---|---|
| perception pipeline | ~60 ms/frame (SegFormer-B2 28, YOLO11n 13, rest 18) → ~16 Hz |
| odometry, grass loop 10.3 m | wheels 1.6 %, flow VO 1.1 %, EKF 0.9 % |
| odometry, ice 5.6 m | wheels 5.8 %, EKF (wheels+IMU+flow VO) 3.3 % |
| stone_yard, 3 crossings in one session | **2/3** succeeded (71 s, 83 s); 3rd aborted after 22 m. Gravel driven over; closest rock 0.19–0.56 m. One cart contact from behind while the rover drove away (actor does not avoid) |
| test_yard, 3 goals in one session | **2/3** succeeded (35 s, 42 s); 3rd timed out after 9.9 m. The same goal run on its own succeeded in 33 s → failures appear later in long sessions, not yet diagnosed |
| safety | stopped 0.35 m before a sudden box · resumed · stopped 0.6 s after perception was killed |

v0.1 (Depth Anything + MPPI) reached 6/6 goals in the same tests; the RGB-only stack is not yet as
reliable. Open issue: intermittent failures on the 3rd goal of a session (candidates: odometry drift
accumulated in the local map, remembered false obstacles, K-turn near obstacles).

## 7.3 Regression checklist (run after changing perception / Nav2 params)

1. `eval_depth.py` — AbsRel all ≤ ~4 %.
2. `eval_odometry.py --pattern loop` and `--pattern ice` — EKF ≤ ~0.5 % / ≤ ~3.5 %.
3. test_yard 3-goal sequence — 3/3 succeeded, no `COLLISION`.
4. stone_yard 3-crossing sequence — 3/3 succeeded, gravel driven over, no `COLLISION - rover drove into it`.
5. `safety_test.py` — 3 × PASS.

Expect run-to-run variation (timing of movers, replanning); judge on several runs.
