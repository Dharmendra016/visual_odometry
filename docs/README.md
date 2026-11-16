# rover4ws documentation

Engineering documentation for the **rover4ws** camera-only navigation prototype: a 4-wheel-steering
rover that takes a goal, builds a map from a single RGB camera, plans a car-like path and drives
there while avoiding obstacles. No LiDAR, no GPS, no prior map.

Status (2026-10-30): **working in simulation** (Gazebo Classic, ROS 2 Humble). Not yet run on the
physical robot. All numbers below are simulation results unless stated otherwise.

The quick-start is in [../README.md](../README.md). These documents explain *how it works, why it was
built this way, what was tried and rejected, and what to do next*.

| # | Document | Read it when you want to … |
|---|---|---|
| 1 | [Overview & architecture](01_overview_and_architecture.md) | understand the whole system, nodes, topics, TF, command chain |
| 2 | [Robot model](02_robot_model.md) | change dimensions, sensors, or move to the real robot |
| 3 | [Perception](03_perception.md) | understand/tune Depth Anything V2, metric scale, point clouds, visual odometry |
| 4 | [Localization](04_localization.md) | understand/tune 4WS wheel odometry and the EKF |
| 5 | [Navigation & safety](05_navigation_and_safety.md) | understand/tune costmaps, planner, controller, behaviour trees, safety layers |
| 6 | [Simulation worlds](06_simulation_worlds.md) | use or modify test_yard / stone_yard, add obstacles or movers |
| 7 | [Testing & results](07_testing_and_results.md) | re-run the evaluation scripts and compare against the baseline numbers |
| 8 | [Setup & environment](08_setup_and_environment.md) | set up a fresh machine, or fix the WSL environment |
| 9 | [Decisions & lessons learned](09_decisions_and_lessons.md) | know why things are the way they are, and the bugs already hit |
| 10 | [Roadmap & real-robot bring-up](10_roadmap_real_robot.md) | plan the next phase (hardware, real camera, terrain classes, GPS) |
| 11 | [RGB-only prototype](11_rgb_only_prototype.md) | the v0.2 stack: SegFormer + YOLO + corridor + optical flow + steering-aware planner, no depth |

## Origins

| Source | What we took | What we did not take |
|---|---|---|
| [DRISHTI](https://github.com/X-DIABLO-X/DRISHTI) | Depth Anything V2 + metric scale from a known ground plane; fit-validity gating; "unseen space is never free"; vehicle-aware step/clearance limits; independent safety supervisor; VO with tracking-health gating | its monolithic Python runtime, 17-arc planner, learned world model / RL policy, PIDNet terrain segmentation (not yet) |
| [monocular-depth-navigation](https://github.com/Yassineg07/monocular-depth-navigation) | architecture: perception converts camera → standard ROS sensor messages, Nav2 does mapping/planning/control | its geometric ground-edge "depth" (replaced by a neural network) |

## Package map

```
.                                  repository root = ROS 2 workspace
├── README.md                      quick start
├── requirements.txt               Python packages rosdep does not install
├── docs/                          ← you are here (media/ holds the demo videos)
└── src/
    ├── rover4ws_description/      URDF/xacro, sensors, ros2_control, RViz model view
    ├── rover4ws_gazebo/           worlds, controllers.yaml, sim.launch.py, four_ws_kinematics.py
    ├── rover4ws_localization/     wheel_odometry.py, ekf.yaml, ekf_vo.yaml
    ├── rover4ws_perception/       depth_node.py, visual_odometry.py                (stack A)
    ├── rover4ws_navigation/       nav2_params.yaml, behaviour trees, safety_gate.py,
    │                              prototype.launch.py, rviz, tools/ (evaluation)   (stack A)
    ├── rover4ws_rgb_perception/   SegFormer, YOLO, corridor, optical flow, tracker (stack B)
    └── rover4ws_rgb_navigation/   steering-aware arc planner, rgb_prototype.launch.py (stack B)
```
