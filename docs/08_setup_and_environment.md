# 8. Setup & environment

## 8.1 Current machine

| | |
|---|---|
| Host | Windows 11 laptop, NVIDIA GeForce RTX 4060 Laptop GPU (8 GB) |
| Linux | WSL2, Ubuntu 22.04, 16 cores, ~10 GB RAM visible |
| ROS | ROS 2 Humble (apt), Gazebo Classic 11 |
| Workspace | `~/ros2_ws` (Windows path `\\wsl.localhost\Ubuntu-22.04\home\dharmendra\ros2_ws`) |
| GPU in WSL | CUDA via `/usr/lib/wsl/lib`; OpenGL via Mesa D3D12 (WSLg) |

**Run ROS commands in the Ubuntu terminal** (`wsl -d Ubuntu-22.04`, the Ubuntu app, or VS Code's
Ubuntu terminal) — not Windows PowerShell (no `&&`, no `source`, no ROS).

## 8.2 Fresh install (another Ubuntu 22.04 machine)

```bash
# ROS 2 Humble desktop: follow docs.ros.org, then
sudo apt install \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-robot-localization ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard ros-humble-rviz2 python3-opencv python3-colcon-common-extensions

# Python ML stack (user site). numpy<2: ROS Humble's python packages are built against numpy 1.x.
python3 -m pip install --user "numpy<2" torch torchvision
python3 -m pip install --user "numpy<2" "transformers>=4.45,<4.50" "huggingface_hub<0.30" ultralytics   # ultralytics = YOLO (v0.2)
# or simply: python3 -m pip install --user -r ~/ros2_ws/requirements.txt
python3 -m pip install --user "setuptools<80"        # colcon-core requires setuptools < 80

# First run downloads depth-anything/Depth-Anything-V2-Small-hf (~100 MB) to ~/.cache/huggingface
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

Optional (needs sudo): `ros-humble-spatio-temporal-voxel-layer` — decaying obstacle layer, a cleaner
fix for moving-object ghost trails than periodic clearing.

## 8.3 What was installed on this machine (2026-10-30)

- `pip` bootstrapped with `get-pip.py --user` (no passwordless sudo, `python3-pip` absent).
- `torch 2.14.1+cu130`, `torchvision 0.29.2`, `transformers 4.49.0`, `huggingface_hub 0.29.3`,
  `numpy 1.21.5` (system, kept), `setuptools 79.0.1` (pinned back from 84 which broke colcon).
- Check: `python3 -c "import torch;print(torch.cuda.is_available())"` → `True`.

## 8.4 Daily workflow

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch rover4ws_navigation prototype.launch.py            # or world:=stone_yard
```
Optional, once: add both `source` lines to `~/.bashrc`.

With `--symlink-install`, Python scripts, YAML, launch files and BTs are symlinked — edits take
effect on the next launch without rebuilding. New files still need a build.

## 8.5 Environment gotchas (all hit during development)

| Problem | Cause | Fix in place |
|---|---|---|
| `executable 'x.py' not found on the libexec directory` | files written from Windows over `\\wsl.localhost` lose the Linux executable bit | every package's CMakeLists runs `chmod +x scripts/*.py` on each `colcon build`; manual fix `chmod +x ~/ros2_ws/src/*/scripts/*.py` |
| gazebo_ros2_control: `Couldn't parse parameter override rule` | the URDF is passed on the command line; `": "` inside an XML comment breaks the YAML parser | never write `": "` in xacro comments |
| RViz never opens from `prototype.launch.py` | arguments to an included launch leak into the parent | includes wrapped in scoped `GroupAction` |
| Nav2 `bt_navigator` fails to activate: `Action server spin not available` | default NavigateThroughPoses BT uses Spin | custom BTs for both navigators |
| colcon errors after pip installs | pip upgraded setuptools to 84 | `pip install --user "setuptools<80"` |
| depth node 135 ms/frame | rclpy validates `bytes` payloads byte by byte | assign `array.array('B', …)` |
| camera at 6 Hz | perfect depth camera point cloud at 640×480 | 320×240 @ 10 Hz, off by default |
| `gzserver` exits with code 255 at start | a previous gzserver still running | `pkill -9 gzserver` |
| Ctrl-C shows `depth_node … exit code -2` | normal shutdown of a Python node with CUDA | ignore |
| Windows `python` not found | Microsoft Store alias | run Python inside WSL |
