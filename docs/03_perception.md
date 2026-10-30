# 3. Perception (`rover4ws_perception`)

Two nodes: `depth_node.py` (camera → metric depth → obstacle/ground point clouds) and
`visual_odometry.py` (camera + metric depth → body velocity).

## 3.1 depth_node — pipeline

```
RGB 640×480 ──► Depth Anything V2-Small (fp16, CUDA, input 518×392)
                 └─► q: relative inverse depth, upsampled to 640×480
                         │
   known camera pose ──► per-pixel expected ground depth Z_g (for rays that hit the ground)
                         │
                RANSAC + least squares:  1/Z_g ≈ a·q + b   on ground-candidate pixels
                         │
          fit valid? ──no──► reuse last fit if < 0.5 s old, else publish NOTHING
                         │yes
                Z = 1 / (a·q + b)          (pixels with a·q+b < 1/50 → invalid / sky)
                         │
      ├─► /perception/depth   (32FC1, same header as the RGB frame)
      └─► back-project every 4th pixel → points in base_footprint → classify by height z:
              obstacle : 0.06 < z < 1.2   or   z < −0.10  (holes, drop-offs)
              ground   : |z| < 0.04
              (0.04 … 0.06 m: neither — not marked, not cleared)
              horizontal range ≤ 4.0 m
          ├─► /perception/obstacles
          └─► /perception/ground
```

### Why the scale fit works

Depth Anything V2 (relative model) predicts **affine-invariant inverse depth**: the true inverse
depth is `1/Z = a·q + b` for unknown `a > 0`, `b` that change every frame.

The camera's pose relative to the ground is known from TF (`camera_optical_frame → base_footprint`:
height `h ≈ 0.297 m`, pitch 11.5°). For a pixel with ray direction `r` (optical frame, z = 1), the
ray's vertical component in the base frame is `d_z = R[2]·r`. If `d_z < 0` the ray hits the flat
ground at depth `Z_g = −h / d_z`. These `Z_g` are precomputed once.

Ground pixels satisfy `1/Z_g = a·q + b`; obstacle pixels in the same region do not and are outliers.
So:
1. Candidates: pixels with `0.4 m < Z_g < 5.0 m`, subsampled by 7.
2. RANSAC, 60 iterations: pick two candidates, solve `a, b`, reject `a ≤ 0`, count inliers with
   `|a·q + b − 1/Z_g| < 0.12 · (1/Z_g)`.
3. Least-squares refit on the best inlier set.
4. Valid if `a > 0` and inlier fraction ≥ 0.35.

DRISHTI fits the ground plane *and* the scale (and has to drop `b` because they are not jointly
identifiable on one plane). We know the plane from the robot geometry, so both `a` and `b` are
identifiable. Typical sim values: `a ≈ 0.357`, `b ≈ −0.12`, inlier fraction 0.99.

### Fit-validity policy (fail safe)

- Weak fit → reuse the last good fit for up to `fit_hold_time` (0.5 s).
- Otherwise → **publish nothing** for that frame. Silence on `/perception/obstacles` makes the
  `safety_gate` stop the robot after 0.5 s. "No information" is never treated as "free".

### Parameters

| Parameter | Default | Meaning / tuning advice |
|---|---|---|
| `source` | `model` | `gt` uses `/camera_gt/depth/image_raw` (sim) instead of the network |
| `model_name` | `depth-anything/Depth-Anything-V2-Small-hf` | Base/Large are more accurate but ~3–10× slower |
| `input_width`, `input_height` | 518, 392 | multiples of 14; 364×280 was not faster on the 4060 |
| `max_range` | 4.0 m | monocular depth error grows with distance; DRISHTI also caps at 4 m |
| `min_obstacle_height` | 0.06 m | step the wheels cannot climb (wheel radius 0.0825 m). Lower = more false obstacles from depth noise |
| `max_obstacle_height` | 1.2 m | ignore overhangs above the robot |
| `negative_obstacle_depth` | 0.10 m | holes / kerb drops |
| `ground_band` | 0.04 m | points used for clearing free space |
| `cloud_stride` | 4 | pixel subsampling → ~19 k candidate points per frame |
| `fit_min_range`, `fit_max_range` | 0.4, 5.0 m | ground pixels used for the scale fit |
| `fit_inlier_tol` | 0.12 | relative inverse-depth tolerance |
| `fit_min_inlier_fraction` | 0.35 | below this the fit is rejected |
| `fit_hold_time` | 0.5 s | |

### Performance (RTX 4060 laptop, sim running)

`depth pipeline 23–25 ms/frame (model 15, scale fit 5, clouds 3)` → ~16 Hz output with the camera at
~23 Hz. The model alone is 10 ms when the GPU is not shared with Gazebo rendering.

The node logs a timing line every 50 frames — use it to check health.

**Important implementation detail:** large message payloads are assigned as
`array.array('B', bytes)`. Assigning `bytes` makes rclpy (Humble) validate every byte in Python and
cost ~90 ms per frame (it was the original bottleneck).

### Accuracy (simulation)

Measured with `tools/eval_depth.py` against the perfect depth camera, 40 frames while driving,
0.3–4 m: AbsRel **3.1 %** (ground 2.5 %, obstacles 9.1 %), δ<1.25 = 99.8 %.
⚠ Gazebo renders are much easier than real images — re-measure on real data (see [10](10_roadmap_real_robot.md)).

### Known limitations

- **Flat-ground assumption**: on slopes/ramps the expected ground depth is wrong → the fit degrades
  or fails (then the robot stops). A tilt-compensated plane (from IMU roll/pitch) is a natural extension.
- Small objects 4–6 cm high are neither marked nor cleared.
- Thin objects (poles, wires) and transparent/reflective surfaces are hard for monocular depth.
- The camera sees only forward (~100° HFOV, ~83° VFOV); ground is visible from ~0.5 m ahead of
  base_link.

## 3.2 visual_odometry — algorithm

Frame-to-frame, sparse, depth-aided:

1. `TimeSynchronizer` pairs `/camera/image_raw` with `/perception/depth` (identical stamps).
2. In the previous grey frame, detect up to 500 Shi-Tomasi corners where depth is valid and < 6 m.
3. Track them into the current frame with pyramidal Lucas-Kanade (21×21, 3 levels); keep tracks
   that pass a forward–backward check (< 1 px).
4. Lift previous-frame corners to 3D with the previous metric depth.
5. `solvePnPRansac` (EPnP, 100 iterations, 2 px) then iterative refinement on the inliers →
   `T_c1c0`.
6. Body motion: `T_b0b1 = T_bc · T_c1c0⁻¹ · T_cb`; take planar `dx, dy, dyaw`; divide by `dt`.
7. Health gate (publish nothing if failed): ≥ 30 inliers, inlier ratio ≥ 0.35, speed ≤ 2 m/s,
   yaw rate ≤ 2.5 rad/s, 0 < dt ≤ 0.5 s.
8. Covariance: `var = 0.0025 · max(1, 100/inliers)` on vx, vy; ×2 on yaw rate.

The scale of VO comes from the ground-plane-scaled depth, so VO inherits any scale error of the
depth fit.

### Parameters

| Parameter | Default | |
|---|---|---|
| `max_features` | 500 | |
| `max_depth` | 6.0 m | far points have poor depth |
| `min_inliers`, `min_inlier_ratio` | 30, 0.35 | tracking health |
| `max_speed`, `max_yaw_rate` | 2.0 m/s, 2.5 rad/s | reject impossible jumps |
| `base_variance` | 0.0025 | (5 cm/s)² at 100 inliers |

### Accuracy (simulation)

When published, VO velocity matched ground truth to ~0.01–0.02 m/s and ~0.01 rad/s. As a
stand-alone integrated pose: 1.5 % drift on grass, 1.4 % on ice (where wheels drift 5.8 %).
See [07](07_testing_and_results.md).

### Known limitations

- Needs texture: plain concrete, darkness, motion blur or a lens covered in dust will lose tracking
  (the EKF then falls back to wheels + IMU).
- Frame-to-frame only: no keyframes, no loop closure, no bundle adjustment.
- Moving objects in view can bias the estimate (RANSAC usually rejects them while they are a
  minority of features).
