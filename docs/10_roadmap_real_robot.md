# 10. Roadmap & real-robot bring-up

## 10.1 Open questions (need answers from the hardware side)

| Question | Why it matters |
|---|---|
| Compute on the robot: Jetson (Orin?), Raspberry Pi, or a laptop? | Depth Anything needs a GPU for ≥ 10 Hz; on CPU expect ~1 Hz → top speed must drop accordingly |
| Camera: the Wi-Fi bullet IP camera, or can it become USB/CSI? | IP cameras add 200–800 ms latency — far beyond the 0.5 s safety timeout and enough to hit things |
| Hub-motor controllers: hoverboard boards with custom FOC firmware (UART) or separate ESCs? Encoder/hall feedback? | defines the ros2_control hardware interface and wheel odometry quality |
| Steering actuators: servos / linear actuators? Angle feedback? Real range and speed? | steering limit sets the turning radius used by planner and controller |
| GPS module, standard (~2 m) or RTK (~2 cm)? | design of the global layer |
| Operating environment (campus sidewalks as in the video? grass? gravel?) | terrain classes, speed, thresholds |

## 10.2 Phase A — real-robot bring-up (no autonomy)

1. Measure the robot; update `properties.xacro`, `four_ws_kinematics.py`, `wheel_odometry.py`,
   Nav2 footprint, `minimum_turning_radius`, `min_turning_r` ([02 §2.7](02_robot_model.md)).
2. Write a `hardware_interface::SystemInterface` plugin (`rover4ws_hardware`) for the motor and
   steering controllers; keep the joint names. Replace the `<hardware>` block in `ros2_control.xacro`
   (select sim/real with a xacro arg).
3. Hardware e-stop must cut motor power. Verify it before anything else.
4. Teleop test: `/cmd_vel` → 4WS kinematics on the real wheels; verify steering signs and wheel
   directions corner by corner.
5. Camera driver (`usb_cam` / `v4l2_camera` / vendor), `camera_calibration`, set true HFOV and
   intrinsics; measure lens height/pitch.
6. IMU driver; check axes against REP-103; calibrate gyro bias at start.
7. Record rosbags (camera, IMU, joint states) on the target terrain — the dataset for everything below.

## 10.3 Phase B — validate perception and odometry on real data (offline first)

- Depth: run `depth_node` on the bags; compare with Marigold offline, or with tape-measured distances
  to cones/boxes. Accept only if obstacles within 3 m are detected reliably and ground is not
  marked as obstacle. Tune `min_obstacle_height`, `fit_inlier_tol`.
- Slopes: add IMU roll/pitch to the ground-plane model in `depth_node` (camera height is fixed, but
  the plane tilts with the robot).
- VO: check tracking health on the real terrain and lighting; compare EKF against a measured course
  (e.g. 20 m tape).
- Latency: measure camera → `/perception/obstacles` age on the robot; must be well below 0.5 s.

## 10.4 Phase C — autonomy on the real robot

- Same `prototype.launch.py` structure without Gazebo; start at 0.2 m/s with a person on the e-stop.
- Increase safety margins first (inflation, stop zone), relax later with data.
- Repeat the test_yard-style course with cardboard boxes.

## 10.5 Phase D — capability extensions

| Item | Notes |
|---|---|
| **Terrain classes** | DRISHTI's 7 classes (sky, trail, grass, rough vegetation, obstacle, water, dynamic) as a start; begin with an off-the-shelf model (e.g. SegFormer ADE20K) projected into a costmap layer; fine-tune on labelled frames from the robot |
| **2.5D local map** | height / slope / roughness / confidence / age layers (`grid_map`, already installed) instead of the binary obstacle layer; vehicle-aware cost from step, slope, roughness |
| **Decaying obstacles** | `spatio_temporal_voxel_layer` (needs sudo) to replace periodic global clearing |
| **Side/rear protection** | rear camera or ultrasonic/bumper; extend collision_monitor polygons |
| **Moving-object handling** | detect people (YOLO) → predict motion → keep-out zones; back away if approached |
| **GPS / global layer** | `navsat_transform_node` + second EKF (`map → odom`), GPS waypoints → NavigateThroughPoses; the sim already publishes `/gps/fix` |
| **Better VO** | keyframes / local bundle adjustment or an off-the-shelf VIO (e.g. RTAB-Map, OpenVINS) using the IMU |
| **Faster perception** | TensorRT / ONNX export of DA-V2 for Jetson |
| **Learned components (DRISHTI)** | world model / RL policy only after the classical stack is reliable — DRISHTI's own README reports 98 % collisions for its RL policy on held-out states |

## 10.6 Definition of done for "prototype on the real robot"

- [ ] Robot reaches 10/10 goals 15 m away on the target terrain around ≥ 3 static obstacles, no contact.
- [ ] Stops for a person stepping in front at walking speed, every time.
- [ ] Stops within 0.6 s when the camera is unplugged.
- [ ] Odometry error < 3 % over a 20 m measured course.
- [ ] All of the above recorded as rosbags with results written into [07](07_testing_and_results.md).
