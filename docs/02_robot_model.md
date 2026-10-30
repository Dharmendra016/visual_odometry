# 2. Robot model (`rover4ws_description`)

## 2.1 The physical robot (from `data/` photo and video)

- Plywood deck, roughly 0.70 × 0.48 m, about 17 cm above the ground.
- Four 6.5″ hoverboard hub-motor wheels at the corners, **each corner independently steered** (4WS).
- Bullet IP camera (white, with antenna) on a short post at the **front-left** corner.
- 12 V SLA battery at the rear; red e-stop and a red battery isolator switch on the deck.
- Electronics: microcontroller boards, motor drivers, buck converters, lots of wiring.
- No GPS/RTK fitted yet.

> **All geometry in the model was estimated from photos.** It must be measured on the real robot
> before any real-world testing (see §2.6).

## 2.2 Files

| File | Contents |
|---|---|
| `urdf/properties.xacro` | **single source of truth** for dimensions, masses, steering limits, camera mount |
| `urdf/macros.xacro` | materials, inertia helpers, `steered_wheel` corner macro |
| `urdf/rover4ws.urdf.xacro` | links/joints: body, battery, e-stop, 4 corners, camera, IMU, GPS |
| `urdf/sensors.gazebo.xacro` | Gazebo sensor plugins (camera, sim-only depth camera, IMU, GPS, ground truth) |
| `urdf/ros2_control.xacro` | joint interfaces + `gazebo_ros2_control` plugin |
| `launch/display.launch.py` | RViz + joint sliders (no simulation) |

xacro arguments: `use_sim` (include Gazebo/ros2_control parts), `controllers_file`, `gt_depth`.

## 2.3 Dimensions (current values)

| Property | Value | Notes |
|---|---|---|
| deck L × W × t | 0.70 × 0.48 × 0.012 m | |
| deck clearance (underside) | 0.175 m | |
| wheel radius / width | 0.0825 / 0.055 m | 6.5″ hoverboard |
| wheelbase (axle to axle) | 0.54 m | |
| track width (wheel centre to centre) | 0.50 m | |
| max steering angle | ±0.61 rad (±35°) | **assumed** — determines turning radius |
| steering rate / effort | 3.0 rad/s / 20 N·m | |
| masses | deck 3.0, wheels 4 × 2.5, brackets 4 × 0.3, battery 2.2 kg | ≈ 16.5 kg total |
| camera position (base_link) | x 0.27, y 0.13, z deck top + 0.11 m | lens ≈ 0.297 m above ground |
| camera pitch | 0.20 rad (11.5°) down | |
| navigation footprint | 0.76 × 0.58 m (`[[0.38,0.29],…]`) | deck + wheel overhang + margin |

## 2.4 Kinematic structure

```
base_footprint (on the ground, below the wheelbase centre)
└─ base_link (axle height = wheel radius)
   ├─ chassis_link (deck) ─ battery_link, estop_link
   ├─ {fl,fr,rl,rr}_steer_joint  revolute about z, ±max_steer, at (±L/2, ±W/2, 0)
   │   └─ {..}_wheel_joint       continuous about the steered y axis (+ = forward roll)
   ├─ camera_mount_link, camera_link (x forward) ─ camera_optical_frame (z forward)
   ├─ imu_link (deck centre, top)
   └─ gps_link (rear-right)
```
The steering axis passes vertically through the wheel centre (zero scrub radius) — an idealisation.

## 2.5 Sensors in simulation

| Sensor | Plugin | Topic(s) | Rate | Model |
|---|---|---|---|---|
| RGB camera | `libgazebo_ros_camera.so` | `/camera/image_raw`, `/camera/camera_info` | 30 Hz configured (≈23 Hz achieved) | 640×480, HFOV 1.74 rad (~100°), Gaussian noise σ 0.007 |
| Perfect depth camera (**sim only**) | `libgazebo_ros_camera.so` (depth) | `/camera_gt/depth/image_raw` | 10 Hz | 320×240, same pose/FOV; only with `gt_depth:=true` |
| IMU | `libgazebo_ros_imu_sensor.so` | `/imu/data` | 100 Hz | gyro σ 2e-3 rad/s, accel σ 1.7e-2 m/s², biases |
| GPS | `libgazebo_ros_gps_sensor.so` | `/gps/fix` | 5 Hz | σ 1.5 m horizontal, 3 m vertical (white noise — real GPS error is correlated) |
| Ground truth | `libgazebo_ros_p3d.so` | `/ground_truth/odom` | 50 Hz | world frame, `base_footprint` body; **evaluation only** |
| Wheel encoders / steering angles | joint_state_broadcaster | `/joint_states` | 100 Hz | perfect in sim |

Why the depth camera is 320×240 @ 10 Hz: at 640×480 its point-cloud generation slowed the RGB
camera from 23 Hz to 6 Hz.

## 2.6 Control interface (ros2_control)

| Joints | Command interface | State interfaces | Controller |
|---|---|---|---|
| `{fl,fr,rl,rr}_steer_joint` | position (±max_steer) | position, velocity | `position_controllers/JointGroupPositionController` |
| `{fl,fr,rl,rr}_wheel_joint` | velocity (±40 rad/s) | position, velocity | `velocity_controllers/JointGroupVelocityController` |

Joint order in both controllers is **FL, FR, RL, RR** (`rover4ws_gazebo/config/controllers.yaml`);
`four_ws_kinematics.py` publishes in that order.

**Moving to the real robot:** only the `<hardware>` plugin in `ros2_control.xacro` changes (a
`hardware_interface::SystemInterface` driving the hub-motor controllers and the steering actuators).
Joint names, controllers and every topic above stay the same.

## 2.7 What to measure on the real robot

Update `properties.xacro` **and** the duplicated parameters in `four_ws_kinematics.py`,
`wheel_odometry.py` (wheelbase, track, wheel radius, max steer), the Nav2 footprint and
`minimum_turning_radius` / `min_turning_r` (see [05](05_navigation_and_safety.md)).

- [ ] wheelbase and track width (wheel contact centres)
- [ ] loaded wheel radius (measure distance per wheel revolution, not the tyre diameter)
- [ ] real steering range per corner, and steering speed
- [ ] whether the steering axis really passes through the wheel centre (scrub radius)
- [ ] deck height, overall footprint including wheel overhang
- [ ] camera position (x, y, z of the lens) and pitch — **depth scale depends directly on camera height**
- [ ] camera resolution, HFOV and distortion (run `camera_calibration`)
- [ ] IMU position/orientation; total mass and CG
