# Human-Aware Social Navigation for a Differential-Drive Robot

ROS 2 Jazzy / Nav2 workspace for socially aware navigation of the `my_amr` robot
(2-wheel differential drive, 2D lidar, Intel RealSense D435). People are detected
with an RGB-D camera and a lidar, tracked over time, and taken into account both
in the costmap (asymmetric social zones) and in the local controller (a
human-aware dynamic window approach with a time-to-collision risk model).
A lidar safety layer sits between the planner and the motor driver.

## Pipeline

```
RGB-D camera ──> social_nav_perception ──┐   (YOLO pose + depth -> 3D people, odom frame)
                                         ├─> social_nav_tracking(_cpp) ──> /planning/tracked_humans
2D lidar ───────────────────────────────┘   (Hungarian assignment + Kalman filter, lidar/camera fusion)

/planning/tracked_humans ──> social_nav_costmap_layer  (AGHPM social cost in local + global costmap)
                         └─> social_nav_controller     (Nav2 FollowPath: human-aware DWA)

Nav2 ──> velocity_smoother ──> /cmd_vel_raw ──> social_nav_safety(_cpp) ──> /cmd_vel ──> robot
                                                (lidar corridor stop / slow-down)
```

## Packages

| Package | Description |
|---|---|
| `social_nav_perception` | Multi-person 3D detection: YOLO pose (ONNX, CPU) + depth back-projection |
| `social_nav_tracking_cpp` | Human tracker (C++, used by default) |
| `social_nav_tracking` | Human tracker (Python reference implementation, owns the parameter file) |
| `social_nav_costmap_layer` | AGHPM (Asymmetric Gaussian Human Proxemics Model) costmap layer |
| `social_nav_controller` | Human-aware DWA local controller (Nav2 `FollowPath` plugin) |
| `social_nav_safety_cpp` | Lidar safety layer (C++, used by default) |
| `social_nav_safety` | Lidar safety layer (Python reference implementation, owns the parameter file) |
| `social_nav_gazebo` | Gazebo plugins (pedestrian collision proxy, ground truth), metrics node, batch tools |
| `thesis_msgs` | Messages for detected and tracked people |
| `linorobot2/*` | Robot description, Gazebo world, Nav2 configuration and launch files, based on [linorobot2](https://github.com/linorobot/linorobot2) (Apache-2.0) |

The C++ tracker and safety layer are ports of the Python implementations; their
tests replay golden traces recorded from the Python code and compare every step.

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy, Nav2, Gazebo Harmonic
- Python packages: `pip3 install --break-system-packages -r requirements-sim.txt`
- **Simulation needs an NVIDIA GPU with the proprietary driver**: `gazebo.launch.py`
  forces the NVIDIA EGL/GLX vendor, because with animated actors gz-sim otherwise
  falls back to Mesa and hangs. On a machine without NVIDIA, remove the two
  `__EGL_VENDOR_LIBRARY_FILENAMES` / `__GLX_VENDOR_LIBRARY_NAME` variables there.

## Build

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test && colcon test-result --all     # 152 tests
```

## Run the simulation

In every terminal: `source env_sim.sh`

```bash
ros2 launch linorobot2_gazebo gazebo.launch.py world_name:=dymap spawn_x:=0.5   # T1
ros2 launch linorobot2_navigation navigation.launch.py sim:=true                # T2
ros2 launch linorobot2_bringup social_nav.launch.py sim:=true                   # T3
```

Then set a goal in RViz (command printed by `env_sim.sh`). The world contains a
corridor with a doorway, two people standing near it and one pedestrian walking
back and forth across the robot's path.

## Batch evaluation

```bash
src/social_nav/social_nav_gazebo/tools/run_batch.sh 100 results/my_batch
python3 src/social_nav/social_nav_gazebo/tools/summarize_batch.py results/my_batch
```

Each run restarts Gazebo, navigation and perception, resets AMCL from ground
truth and drives from the start pose to the same goal. A run counts only if the
robot reaches the goal; runs with at least one collision count as failed runs.
Collisions and clearances are computed by `social_nav_metrics` from
ground-truth positions. See [docs/RESULTS.md](docs/RESULTS.md).
