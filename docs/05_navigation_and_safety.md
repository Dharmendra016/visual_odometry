# 5. Navigation & safety (`rover4ws_navigation`)

Config: `config/nav2_params.yaml`. Launch: `launch/nav.launch.py` (started 8 s after the rest by
`prototype.launch.py`). Nav2 version: Humble 1.1.20.

## 5.1 Servers

| Node | Plugins | Lifecycle-managed |
|---|---|---|
| `bt_navigator` | custom BTs (§5.5) | yes |
| `planner_server` | `nav2_smac_planner/SmacPlannerHybrid` | yes |
| `controller_server` | `nav2_mppi_controller::MPPIController` | yes |
| `behavior_server` | `BackUp`, `Wait` (no `Spin`) | yes |
| `collision_monitor` | 2 polygons, 1 point-cloud source | yes |
| `safety_gate` (ours) | — | no |

`velocity_smoother`, `smoother_server`, `waypoint_follower`, `map_server` and `amcl` are not used.
All frames are `odom` / `base_footprint`; the controller reads `/odometry/filtered`.

## 5.2 Costmaps

Both costmaps use only the camera clouds — an `ObstacleLayer` with two observation sources plus an
`InflationLayer`:

| Source | Topic | marking | clearing | Notes |
|---|---|---|---|---|
| `obstacles` | `/perception/obstacles` | ✓ | ✓ | ray-traces free space up to each obstacle point |
| `ground` | `/perception/ground` | ✗ | ✓ | ray-traces free space up to each ground point |

`sensor_frame: camera_link` (ray origin), `obstacle_max_range 4.5`, `raytrace_max_range 5.0`,
`min_obstacle_height −0.5` (so holes, z < −0.10, are marked), `max_obstacle_height 1.5`.
`track_unknown_space: true` — **cells never seen stay unknown**, never free.

| | Local | Global |
|---|---|---|
| frame / size | odom, rolling 6 × 6 m | odom, fixed 50 × 50 m, origin (−15, −25) |
| resolution | 0.05 m | 0.10 m |
| update / publish | 10 / 5 Hz | 2 / 1 Hz |
| inflation radius / cost scaling | 0.65 m / 4.0 | 0.75 m / 3.0 |

Footprint (both): `[[0.38, 0.29], [0.38, −0.29], [−0.38, −0.29], [−0.38, 0.29]]`.

**Global map ≥ 50 × 50 m only.** Driving beyond x ∈ [−15, 35], y ∈ [−25, 25] (odom) leaves the
global costmap — enlarge or make it rolling for longer missions.

## 5.3 Global planner — Smac Hybrid-A*

| Parameter | Value | Why |
|---|---|---|
| `motion_model_for_search` | `DUBIN` | forward-only car-like paths (reverse is handled by recovery) |
| `minimum_turning_radius` | **0.9 m** | feasible is 0.64 m; 0.75 m let the controller saturate the steering and cut corners near obstacles |
| `allow_unknown` | true | needed — the map starts empty; unknown cells carry a high cost (`cost_penalty 2.0`) so known-free routes are preferred |
| `angle_quantization_bins` | 72 | |
| `max_planning_time` | 3 s | |
| `smooth_path` | true | |

The goal **heading matters**: Smac plans to arrive with the requested yaw. Asking a car-like robot to
arrive facing back the way it came forces a loop — in RViz drag the goal arrow in the direction of
travel. (`tools/nav_test.py` defaults the yaw to the bearing from robot to goal.) Nav2 Humble has
no "ignore goal heading" option for Smac (added in later releases).

## 5.4 Controller — MPPI (Ackermann model)

Samples 2000 trajectories × 56 steps × 0.05 s (2.8 s horizon) every cycle (20 Hz) with the
Ackermann motion model (`min_turning_r 0.7 m`), and scores them.

| Critic | Weight | Purpose |
|---|---|---|
| ConstraintCritic | 4.0 | respect velocity / turning limits |
| CostCritic | 3.81 (`collision_cost 1e6`, `critical_cost 300`, `consider_footprint true`) | stay away from obstacles using the local costmap |
| GoalCritic / GoalAngleCritic | 5.0 / 3.0 | converge near the goal |
| PathAlignCritic / PathFollowCritic / PathAngleCritic | 14.0 / 5.0 / 2.0 | follow the global path |
| PreferForwardCritic | 5.0 | avoid reversing |

Limits: `vx_max 0.4`, `vx_min −0.2`, `vy_max 0`, `wz_max 1.0`. Goal checker: xy 0.30 m, yaw ignored
(6.28). Progress checker: 0.3 m within 15 s.

**History:** Regulated Pure Pursuit was used first. It follows the path purely geometrically; when
the commanded curvature exceeded the steering limit, the 4WS node reduced the turn, the robot cut
corners, passed a crate at 0.10 m and grazed the "person" (0.07 m) and timed out on one goal.
MPPI fixed this: all 3 test goals succeeded and times dropped (89 s → 59 s on the first goal).

## 5.5 Behaviour trees (`behavior_trees/`)

`navigate_4ws.xml` (NavigateToPose) and `navigate_through_poses_4ws.xml` (NavigateThroughPoses):

```
RecoveryNode (6 retries)
├─ PipelineSequence
│  ├─ RateController 0.1 Hz → ClearEntireCostmap (GLOBAL)        ← forget ghost trails every 10 s
│  ├─ RateController 1 Hz  → RecoveryNode
│  │                          ├─ ComputePathToPose (GridBased)
│  │                          └─ ClearEntireCostmap (GLOBAL)     ← on planning failure
│  └─ RecoveryNode → FollowPath (MPPI)  /  ClearEntireCostmap (LOCAL) on failure
└─ ReactiveFallback: GoalUpdated | RoundRobin[ BackUp 0.3 m @ 0.1 m/s, Wait 2 s ]
```

- **No Spin** anywhere: the robot cannot rotate in place. Nav2's default BTs reference `spin`
  and fail to load without it — both BT files are set explicitly in `nav.launch.py`.
- **Periodic global clearing** exists because moving people/carts leave "ghost" obstacle trails
  that are only removed when the camera looks at those cells again. Without it a route behind a
  mover stayed blocked and a goal aborted. Static obstacles reappear as soon as they are in view;
  the local costmap (used by MPPI for avoidance) is not cleared periodically.
- The `bt_navigator` logs `Failed to get result for follow_path in node halt!` whenever a replan
  pre-empts the controller — harmless in Humble.

## 5.6 Safety layers

### collision_monitor (Nav2)

`cmd_vel_nav → cmd_vel_safe`, source `/perception/obstacles` (0.05 < z < 1.5 m), base frame
`base_footprint`. `max_points 5` → acts when **more than 5** points are inside a polygon.

| Polygon | Points (base_footprint) | Meaning | Action |
|---|---|---|---|
| PolygonStop | x 0 → 0.76, y ±0.32 | 0.40 m ahead of the bumper, robot width + 2 cm | stop |
| PolygonSlow | x 0 → 1.56, y ±0.70 | 1.2 m ahead of the bumper, wide | speed × 0.4 |

History: stop zone first reached only 0.19 m ahead → stopped 0.10 m from a sudden box; moved to
0.40 m → stops ~0.45 m away. Widening the stop zone to ±0.40 m (to catch people stepping in from
the side) froze the robot while squeezing past walls in test_yard (goal timed out) — reverted to
±0.32 m; the wide slow zone handles side entries instead.

**Gotcha:** in Humble, `collision_monitor` silently **ignores** a source whose data is older than
`source_timeout` — it does not stop. That is why `safety_gate` exists.

### safety_gate (ours) — `scripts/safety_gate.py`

`cmd_vel_safe → cmd_vel`. Passes commands through unless:
- `/perception/obstacles` not received for `perception_timeout` (0.5 s) → `STOP_STALE_PERCEPTION`
- IMU roll or pitch > `max_tilt` (0.35 rad ≈ 20°) → `STOP_TILT`

State changes are published on `/safety/state`. Tested: killing `depth_node` mid-drive stops the
robot in 0.6 s.

### What is NOT protected

- Sides and rear (camera is forward-only; collision monitor polygons are forward-only).
- Moving objects are not predicted; the robot stops for them but does not get out of their way.
- Teleop bypasses both layers.
- Physical e-stop must be hardware (cut motor power) — never software.

## 5.7 Tuning cheat-sheet

| Symptom | First things to try |
|---|---|
| passes obstacles too closely | ↑ local `inflation_radius`, ↑ CostCritic weight, ↓ `vx_max` |
| refuses narrow gaps | ↓ inflation, ↓ `cost_scaling_factor` gradient, check footprint |
| loops / circles near goal | check goal heading; ↑ goal `xy_goal_tolerance`; ↑ `minimum_turning_radius` |
| stops for no reason | look at red points in RViz (false obstacles from depth noise) → ↑ `min_obstacle_height`; check `/safety/state` |
| stuck behind a mover's trail | lower the BT `RateController hz` period; consider STVL (decaying voxel layer) |
| slow, hesitant driving | slow zone too big; MPPI `vx_std`/`temperature` |
