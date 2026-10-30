# 11. RGB-only prototype (v0.2, branch `rgb-only`)

A second perception/navigation stack that uses **one ordinary RGB camera and no depth estimation
at all** — neither a depth sensor nor a depth network. It reuses the robot model, Gazebo worlds,
4WS kinematics, wheel odometry, EKF, Nav2 global planner, collision monitor and safety gate.

The Depth Anything V2 prototype (chapters 1–10) is unchanged and still runs with
`ros2 launch rover4ws_navigation prototype.launch.py`; its exact state is saved as git tag
`v0.1-depth-anything`.

```bash
ros2 launch rover4ws_rgb_navigation rgb_prototype.launch.py                 # stone_yard (default)
ros2 launch rover4ws_rgb_navigation rgb_prototype.launch.py world:=test_yard
```
Wait for `rgb pipeline … ms/frame` in the log, then use **2D Goal Pose** in RViz (drag the arrow in
the direction of travel). Arguments: `world`, `use_vo`, `gui`, `rviz`, `x`, `y`, `yaw`.

## 11.1 Requested components → implementation

| Requested | Implementation | File |
|---|---|---|
| one ordinary RGB camera | `/camera/image_raw` 640×480; processed at 320×240 | — |
| YOLO for people and obstacles | Ultralytics **YOLO11n** (COCO, 80 classes), conf 0.35 | `models.py` |
| SegFormer free-space segmentation | **SegFormer-B2** ADE20K (150 classes) → drivable / sky masks | `models.py` |
| vanishing point / corridor | polar free-space histogram (corridor) + path-edge line fit (vanishing point) | `corridor.py` |
| optical flow motion cues | LK flow → **ground-plane visual odometry** + **per-object independent-motion residual** | `flow.py` |
| IMU and wheel feedback | unchanged EKF (`ekf_vo.yaml`): wheels + IMU yaw rate + flow VO | rover4ws_localization |
| steering-aware local planner | **new arc planner**: curvature limit, steering-rate ramp, footprint circles, predicted moving objects | `steering_planner.py`, `arcs.py`, `local_map.py` |

## 11.2 Why no depth is needed: flat-ground geometry

The camera's height (0.297 m) and pitch (11.5°) are known from the URDF. With a flat-ground
assumption every pixel below the horizon corresponds to exactly one ground point
(`ground_geometry.py` — "inverse perspective mapping"). That single fact provides:
- metric position of the **bottom edge** of anything standing on the ground (obstacle boundary),
- the **height** of an object from the vertical extent of its pixels,
- metric **ground motion** for visual odometry (ground points move rigidly with the robot),
- **object positions** from the bottom-centre of each YOLO box.

Its limits are the same as before: slopes and bumps break the assumption (see §11.8).

## 11.3 Perception node (`rgb_perception_node.py`)

Per frame:
1. **SegFormer-B2** on the full image (input 512×384), labels resized to 320×240. Drivable classes:
   floor, road, grass, sidewalk, earth, field, sand, path, dirt track, runway, land, rug. Sky separately.
   (In Gazebo, objects get absurd class names — a rock is "bed"/"airplane"/"ball" — but they are
   never drivable, which is all the binary free-space test needs.)
2. **YOLO11n** on the full image. Every box is an object; COCO person/vehicle/animal classes are
   *dynamic* (footprint radius per class, e.g. person 0.3 m).
3. **Free-space column scan** (`freespace.py`), vectorised over 320 columns:
   - drivable pixels = free; vertical runs of non-drivable, non-sky pixels = candidate obstacles,
   - contact distance from the run's bottom row; height from its top row along the camera ray,
     evaluated `object_depth` (0.1 m) behind the contact,
   - runs lower than 6 cm are **drivable** (pebbles, speckle) and the scan continues above them,
   - first tall run per column = obstacle boundary; if it lies inside a dynamic YOLO box the column
     is reported as dynamic (kept out of the static map, so walking people leave no trails).
4. **Corridor** (`corridor.py`): per-column free range → polar histogram −50…+50° in 2° bins →
   min-filter as wide as the robot at 2.5 m → most open heading (ties → straight), free range,
   width. **Vanishing point**: Huber line fits to the left/right edges of the drivable region;
   confidence 0 in open fields (edges touch the image border).
5. **Optical flow** (`flow.py`): Shi-Tomasi corners on drivable ground within 4 m (+ inside YOLO
   boxes), pyramidal LK with forward-backward check.
   - *Ego-motion*: both ends of ground tracks projected to the ground → 2D rigid transform by
     2-point RANSAC (4 cm tolerance) + Kabsch → `/vo/odom` (≥ 25 inliers, ratio ≥ 0.4).
   - *Independent motion*: for each object, measured median flow in the lower box vs the flow
     its ground contact would have if static (from the ego-motion) → residual px/frame.
6. **Tracker** (`tracker.py`): odom-frame nearest-neighbour (1 m gate, same class), smoothed
   velocity, dropped after 1 s. *Moving* = ≥ 3 hits and (speed > 0.25 m/s or flow residual > 1.2 px).

| Topic | Content |
|---|---|
| `/rgb/obstacles` | static obstacle boundary (base_footprint, z = 0.2), **published every frame** (safety gate watches it) |
| `/rgb/freespace` | free ground points (every 3rd pixel), for clearing |
| `/rgb/dynamic` | circles around dynamic objects (collision monitor source) |
| `/rgb/objects` | Float32MultiArray, 8 fields per track: id, class idx, x, y, vx, vy, radius, moving (odom) |
| `/rgb/corridor` | heading (rad, base), free range, width, vp_u, vp_v, vp_conf |
| `/rgb/overlay` | debug image (only rendered when subscribed): green = drivable, bright green = free, red dots = obstacle boundary, yellow = small (drivable) stones, boxes with distance, cyan corridor arrow, magenta vanishing point |
| `/vo/odom`, `/vo/status` | same interface as before → EKF unchanged |

Parameters (`declare_parameter` in the node): `seg_model` (B0 is 2× faster, noisier),
`seg_input_height/width` 384/512, `yolo_weights` (`~/.cache/rover4ws/yolo11n.pt`, auto-download),
`yolo_conf` 0.35, `work_width/height` 320/240, `max_range` 5.0 m, `min_obstacle_height` 0.06 m,
`free_stride` 3.

Timing (RTX 4060, sim running): **~60 ms/frame** (SegFormer 28, YOLO 13, scan + flow + rest 18)
→ ~16 Hz with the camera at ~24 Hz.

## 11.4 Steering-aware planner (`rover4ws_rgb_navigation/scripts/steering_planner.py`)

Replaces Nav2's bt_navigator + controller_server. Nav2 `planner_server` (Smac Hybrid, global
costmap from `/rgb/obstacles` + `/rgb/freespace`) still provides the global path.

- **Interfaces:** serves `NavigateToPose` (`navigate_to_pose`) and listens to `/goal_pose`, so RViz
  and `tools/nav_test.py` work unchanged. Publishes `/cmd_vel_nav`, `/rgb_nav/arcs` (green feasible,
  red blocked, blue chosen), `/rgb_nav/local_map`, `/rgb_nav/state`.
- **Local evidence map** (`local_map.py`): 10 × 10 m, 5 cm, rolling, odom frame. Obstacle hit
  +0.35, free observation −0.25, occupied if evidence > 0.5 (**needs obstacle evidence from ≥ 2
  frames**), **exponential decay τ = 20 s**; distance transform for clearance.
- **Candidates** (`arcs.py`): 21 target curvatures in ±1/0.75 m⁻¹ × 3 speeds (v, 0.6v, 0.3v), rolled
  out 2.5 s at 0.1 s. Curvature moves towards the target at 2.5 m⁻¹/s (steering servo rate) — a
  hard turn is not instantaneous. Footprint = 3 circles (x −0.22, 0, 0.22; r 0.33).
- **Hard constraints:** footprint clearance > 0.05 m in the local map — or, if the robot is already
  closer than that, "no worse than now" (escape rule); clearance > 0.25 m to every tracked object's
  predicted position during the first 1.5 s (moving objects extrapolated with their velocity).
- **Turn-around / K-turn:** if the carrot is > 100° off the heading (goal behind), arc scoring is
  bypassed: full-lock forward turn towards the carrot's side (side kept once chosen); if that is
  blocked, reverse 0.15 m/s with opposite lock (three-point turn). Reverse uses the *remembered*
  map behind the robot — the camera cannot see there.
- **Cost:** `2.0·dist(end, carrot) + 0.4·heading error + 4·max(0, 0.6 − clearance)
  + 3·max(0, 1.2 − object clearance) + 0.2·corridor deviation + 0.3·curvature change + 0.6·slowness`.
  Carrot = 1.8 m ahead on the global path. The corridor term only applies when the carrot lies
  inside an open corridor ≥ 1.5 m.
- **Command:** first-step curvature of the best arc → `(v, v·κ)`; the 4WS kinematics node turns it
  into wheel angles. Speed cap `0.15 + 0.35·distance_to_goal` (≤ 0.4 m/s) for a gentle arrival.
- **Blocked** (no feasible arc): stop, clear the global costmap, replan; after 3 s back up 0.27 m
  straight if the remembered map behind is clear (repeats every 6 s); abort after 45 s.
- Global path refreshed every 1 s; global costmap cleared every 15 s.

Safety chain unchanged: `/cmd_vel_nav → collision_monitor (sources /rgb/obstacles + /rgb/dynamic)
→ safety_gate (perception_topic:=/rgb/obstacles) → /cmd_vel`.

## 11.5 Problems found while building it

| Problem | Cause | Fix |
|---|---|---|
| pipeline 106 ms/frame | TF lookup waited 30 ms for the EKF pose at the image stamp | non-blocking lookup, latest pose fallback → 60 ms |
| robot blocked at the gravel edge | vertical-face height estimate made a 3 cm pebble 0.3 m from the camera look 7.4 cm tall | `object_depth` prior 0.1 m |
| SegFormer labels nonsense on Gazebo objects | trained on real photos | only the binary drivable/non-drivable decision is used |
| YOLO calls rocks "frisbee" | COCO has no rock class | only person/vehicle/animal classes are treated as dynamic; rocks come from segmentation |
| robot pinned 4 cm from rock_2, then deadlocked | (a) local map decay τ = 6 s forgot rocks beside the robot (out of camera view); (b) once inside the margin every arc counted as colliding | escape rule ("no worse than now"); footprint circles 0.31 → 0.33 m |
| with τ = 30 s the robot got stuck *inside* the gravel | single-frame pebble mis-detections were remembered for 30 s, mostly beside/under the robot where the camera cannot clear them | occupied only after evidence from ≥ 2 frames; τ = 20 s |
| return trip never started (0 m in 200 s) | goal behind the robot: every arc ends about equally far from it, the planner chose "straight, slow", drove at the wall; once within 0.7 m no forward-only (Dubins) path exists | explicit turn-around / K-turn manoeuvre |
| safety_test phase 3 hung | `pkill -f rgb_perception_node.py` also matched the test's own command line (`--kill rgb_perception_node.py`) | kill by PID, excluding itself |

## 11.6 Results

See [07 — testing](07_testing_and_results.md) §7.4. Summary: perception ~16 Hz; 4 of 6 navigation goals in long multi-goal sessions (each failed goal succeeded when run on its own); safety tests pass; flow VO halves wheel-slip error.

## 11.7 Depth-Anything stack vs RGB-only stack

| | v0.1 Depth Anything | v0.2 RGB-only |
|---|---|---|
| geometry source | learned dense depth, scaled by ground plane | flat-ground projection only |
| obstacles | every pixel with height > 6 cm (3D) | bottom edge of non-drivable regions + height from extent |
| semantics | none | SegFormer drivable classes, YOLO objects |
| moving objects | none (just obstacles) | tracked, velocity, flow-based motion cue, predicted in planning |
| VO | depth-aided PnP | ground-plane homography (no depth) |
| local planning | Nav2 MPPI (Ackermann) | custom steering-aware arc planner |
| ghost trails | periodic global costmap clearing | people kept out of the static map + decaying local map |
| per-frame cost | ~24 ms | ~60 ms |
| weak spots | needs ground fit; obstacle height from depth noise | relies on segmentation quality; flat-ground assumption for *everything* |

## 11.8 Known limitations

- **Segmentation is the single point of failure.** Anything mis-labelled drivable is invisible; a
  drivable surface mis-labelled (shadows, puddles, colour changes) blocks the robot. Must be
  validated on real footage of the target terrain (and likely fine-tuned).
- **Overhangs** (tables, branches) are only seen at their ground contact.
- **Flat-ground assumption** for distances, heights, VO and object positions — slopes and curbs
  distort all of them. IMU-compensated pitch/roll is the first improvement.
- Object positions use the box bottom; partially visible people (cut by the image edge) are
  placed too close or too far.
- The tracker is single-hypothesis and has no re-identification.
- In the sims, actors do not avoid the robot; see chapter 6.
