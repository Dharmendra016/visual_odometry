# 4. Localization (`rover4ws_localization`)

Goal of this phase: a smooth, continuous pose in the `odom` frame (`odom → base_footprint`).
No GPS and no global map yet, so the pose drifts slowly over distance; it never jumps.

## 4.1 4WS kinematics (shared by control and odometry)

For a rigid body moving with body velocity `(vx, vy, ωz)`, a wheel at `(xᵢ, yᵢ)` has ground velocity

```
vxᵢ = vx − ωz · yᵢ
vyᵢ = vy + ωz · xᵢ
```

Wheel positions (base_link): FL (+L/2, +W/2), FR (+L/2, −W/2), RL (−L/2, +W/2), RR (−L/2, −W/2),
with L = 0.54, W = 0.50.

### Command side — `four_ws_kinematics.py` (rover4ws_gazebo)

- Each wheel is steered to `atan2(vyᵢ, vxᵢ)` and spun at `|vᵢ| / r`. If the angle exceeds ±90° the
  wheel drives backwards instead. All wheels then share one instantaneous centre of rotation
  (no scrubbing).
- `vx + ωz` → **counter-phase 4WS** (front and rear steer opposite ways).
- `vx + vy` → **crab** (all wheels parallel), up to atan(vy/vx) = 35°.
- If any wheel would exceed ±35°, `(vy, ωz)` are scaled down (bisection) until feasible —
  never clamped per wheel (clamping makes wheels fight each other). A warning
  `Command exceeds steering limit; turn scaled to X` is logged.
- Wheel speeds are scaled uniformly to ≤ 20 rad/s.
- Watchdog: no `/cmd_vel` for 0.5 s → zero wheel speed (steering held).

**Consequences of ±35° steering**
- In-place rotation and pure sideways motion are impossible (the robot just does not move).
- Tightest turn: the inner front wheel saturates first: `R_min = W/2 + (L/2)/tan(35°) ≈ 0.64 m`
  (not the 0.39 m bicycle-model figure quoted early in the project — that ignored the track width).

### Measurement side — `wheel_odometry.py`

Each wheel reports steering angle `δᵢ` (position) and spin `wᵢ` (velocity) → measured ground
velocity `(wᵢ r cos δᵢ, wᵢ r sin δᵢ)`. Eight equations, three unknowns:

```
A · [vx, vy, ωz]ᵀ = b      rows per wheel: [1, 0, −yᵢ] and [0, 1, xᵢ]
[vx, vy, ωz] = pinv(A) · b
residual = RMS(A·x − b)     → /wheel/slip
```

- The residual is a **slip / inconsistency indicator**: if the wheels disagree about how the body
  moves, at least one is slipping.
- Twist covariance on vx, vy: `base_variance + slip_gain · residual²` = `0.01 + 4·res²`; yaw ×4.
- The pose is integrated (midpoint rule) and published for evaluation, with huge pose covariance so
  it is never fused.
- No TF is published — the EKF owns `odom → base_footprint`.

Why `base_variance = 0.01` ((10 cm/s)²) and not smaller: when all four wheels slip together (ice,
wheelspin on acceleration) they stay mutually consistent, so the residual cannot detect it. With a
tighter value the EKF ignored visual odometry on ice (5.1 % error); with 0.01 it uses VO (3.1 %).
Real hoverboard wheels on outdoor ground also justify a looser value.

## 4.2 EKF — `robot_localization` `ekf_node`

`config/ekf_vo.yaml` (default, `use_vo:=true`) and `config/ekf.yaml` (`use_vo:=false`).

| Setting | Value | Why |
|---|---|---|
| `frequency` | 50 Hz | |
| `two_d_mode` | true | ground robot; z, roll, pitch ignored |
| frames | `odom` / `base_footprint`, `world_frame: odom` | no map frame yet |
| `odom0: /wheel/odom` | vx, vy, vyaw | body twist only |
| `imu0: /imu/data` | vyaw only | absolute yaw from a magnetometer is unreliable next to hub motors; accelerations are too noisy to integrate |
| `odom1: /vo/odom` (ekf_vo only) | vx, vy, vyaw | only arrives while VO is healthy |
| `sensor_timeout` | 0.2 s | |

All sensors are fused as **velocities** (`differential: false` on twist), so a sensor dropping
out never makes the pose jump.

## 4.3 Results (simulation, `tools/eval_odometry.py`)

Final position error as % of distance driven, compared with Gazebo ground truth:

| Scenario | Wheels | VO | EKF wheels+IMU | EKF wheels+IMU+VO |
|---|---|---|---|---|
| grass loop, 10.3 m (spawn −8, −10) | 1.6 % | 1.5 % | 0.1 % | 0.4 % |
| ice patch μ = 0.05, 5.6 m (spawn 1.5, 7) | 5.8 % | 1.4 % | 5.7 % | **3.1 %** |

Reading: in simulation the wheels and the IMU yaw rate are unrealistically perfect on grass, so VO
adds a little noise there; when wheels slip, VO roughly halves the error. Expect larger errors on
the real robot (wheel radius error, steering backlash, IMU bias).

## 4.4 Tuning knobs

| Symptom | Try |
|---|---|
| EKF follows slipping wheels | increase `wheel_odometry.base_variance` or `slip_gain` |
| EKF jittery when VO is noisy | increase `visual_odometry.base_variance` or `min_inliers` |
| heading drifts | check IMU gyro bias; on the real robot calibrate the IMU at start-up |
| odom scale wrong on real robot | measure the effective wheel radius (distance per revolution) |
