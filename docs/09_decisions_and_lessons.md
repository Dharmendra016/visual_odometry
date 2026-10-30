# 9. Decisions & lessons learned

A lightweight decision log: what was chosen, the alternatives, and the evidence. Newest last.

## D1 — Adapt DRISHTI's perception, but use Nav2 for everything after perception
- **Context:** DRISHTI (camera-only off-road nav) has good perception ideas, but its README states it
  was never run on a robot, its ROS 2 package targets Jazzy/Harmonic and was never run, and its
  navigation results come from a decision-level simulator with perfect information.
  monocular-depth-navigation works end to end in Gazebo with Nav2, but its "depth" is a geometric
  ground-edge heuristic.
- **Decision:** DRISHTI-style perception (Depth Anything V2 + ground-plane scale + validity gating,
  unknown ≠ free) publishing standard PointCloud2, consumed by standard Nav2.
- **Consequence:** small amount of custom code; mature, configurable planning/control/safety.

## D2 — Depth Anything V2-Small, not Marigold, online
- Marigold is diffusion-based: ~0.5–2 s/frame even in fast variants → unusable for driving.
  Useful **offline** as a reference on recorded real footage.
- DA-V2-Small: 10–15 ms on the RTX 4060 in fp16.

## D3 — Metric scale from the *known* ground plane (fit a and b)
- The camera pose relative to the ground is known from the URDF, so the expected ground depth per
  pixel is known; both affine parameters of the relative inverse depth are identifiable.
- Measured: 3.1 % AbsRel in sim. Cost: assumes flat ground (see limitations).

## D4 — Geometric traversability only (no terrain classes yet)
- Segmentation models trained on real photos produce meaningless output on Gazebo textures, so they
  cannot be validated in sim. Step > 6 cm = obstacle, hole > 10 cm = obstacle.
- Terrain classes are planned once real camera data exists ([10](10_roadmap_real_robot.md)).

## D5 — Visual odometry included, fused as velocity, health-gated
- User asked "what is the point of the camera without VO?" — answer: the camera's main job is
  seeing obstacles; VO is a second use that matters when wheels slip.
- Proven with the invisible ice patch: EKF error 5.7 % → 3.1 % with VO.
- Required lowering trust in the wheels (`base_variance` 0.0004 → 0.01) because uniform slip is
  invisible to the wheel residual.

## D6 — Odom frame only, no GPS / map frame
- The physical robot has no GPS/RTK yet; the user's first goal is "mark a position and go there".
- `/gps/fix` is already simulated for the later phase (navsat_transform + map-frame EKF).

## D7 — Smac Hybrid-A* (Dubins) with R_min 0.9 m
- The rover is car-like at ±35° steering: no in-place rotation, no pure sideways motion.
- Correction made during the project: the true minimum turning radius is ≈ 0.64 m (inner front wheel
  saturates), not 0.39 m (bicycle model).
- 0.75 m planned radius let the controller saturate steering → corner cutting; 0.9 m is followable.

## D8 — MPPI (Ackermann) instead of Regulated Pure Pursuit
- RPP: 1 of 3 goals failed, 0.07 m pass by an obstacle, 0.10 m pass by a crate.
- MPPI: 3/3, ≥ 0.21 m clearance, faster. CostCritic uses the costmap directly.

## D9 — No Spin; custom behaviour trees
- Spin is physically impossible; Nav2's default BTs need the spin server to load.
- Recoveries: back up 0.3 m, wait 2 s, clear costmaps.

## D10 — Two independent safety layers
- `collision_monitor` (Nav2) for geometric stop/slow zones.
- `safety_gate` (ours) because Humble's collision_monitor ignores stale sources instead of stopping.
- Stop zone 0.40 m ahead of the bumper, robot width + 2 cm (wider froze the robot in narrow passages);
  slow zone 1.2 m ahead, ±0.70 m (catches side entries early).

## D11 — Forget the global costmap every 10 s
- Moving people/carts leave ghost obstacle trails; with them a goal aborted in the stone yard.
- Without sudo, STVL (decaying voxel layer) could not be installed; periodic clearing of the *global*
  map via the BT is the stop-gap. Local map untouched. Revisit with STVL.

## D12 — Measure everything against ground truth
- Every change was judged with the tools in [07](07_testing_and_results.md); several "obvious"
  improvements were reverted because the numbers got worse (wide stop zone, 0.75 m radius, tight
  wheel variance).

---

## Lessons learned (engineering)

1. **Test layout bugs look like algorithm bugs.** The first odometry test drove into a crate and
   reported 93 % "odometry error". Always sanity-check distance travelled.
2. **odom ≠ world unless the robot spawns at the origin.** A goal sent in the wrong frame produced a
   "9.7 m error" that was purely the test script.
3. **Final heading matters for car-like robots.** A goal requiring the opposite heading produced
   endless loops near the goal.
4. **Profile before optimising.** The depth model was 10 ms; 92 ms was rclpy validating message
   bytes; the camera itself was throttled by an unrelated depth sensor.
5. **Sim is too kind.** Perfect wheels and IMU make VO look useless on grass; deliberately add
   failure cases (ice) to see the value of a component.
6. **Scripted movers never dodge.** Distinguish "robot drove into it" from "it walked into a stopped
   robot" before concluding anything about safety.
7. **Process cleanup in test scripts:** `pkill -f pattern` also kills the script whose own command
   line contains the pattern (and its parent shell).
