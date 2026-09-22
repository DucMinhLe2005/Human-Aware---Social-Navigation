# Human-Aware Social Navigation

<p align="center">
  <img src="docs/images/social_nav_real.webp" width="49%" alt="Human-Aware Social Navigation real robot"/>
  <img src="docs/images/social_nav_cad.webp" width="49%" alt="Human-Aware Social Navigation CAD model"/>
</p>

<p align="center">
  <b>Real Robot</b> &nbsp;&nbsp;&nbsp;&nbsp; <b>CAD Model</b>
</p>

A ROS 2 Jazzy human-aware mobile robot navigation platform built on top of [linorobot2](https://github.com/linorobot/linorobot2), extended with custom robot hardware calibration, RGB-D human perception, camera-LiDAR human tracking, asymmetric social-space modeling, socially-aware global planning, predictive local control, and an independent LiDAR safety layer.

The project is designed for a **real 2WD differential-drive robot** and keeps the standard linorobot2 workflow for low-level control, odometry, TF, SLAM, AMCL, Nav2, and sensor bringup while adding a dedicated social-navigation stack.

> **Base platform:** this repository was developed and calibrated from the ROS 2 Jazzy version of [linorobot2](https://github.com/linorobot/linorobot2) and its micro-ROS hardware architecture. The custom code in this repository changes the robot geometry, ESP32 configuration, sensor setup, navigation parameters, perception/tracking pipeline, social costmap, controller, and safety behavior for this robot.

---

## 1. System Overview

The complete real-robot pipeline is:

```text
                           ┌─────────────────────────────┐
                           │ Intel RealSense D435 RGB-D  │
                           └──────────────┬──────────────┘
                                          │
                                          ▼
                           ┌─────────────────────────────┐
                           │ YOLO Pose / ONNX inference  │
                           │ person + body keypoints     │
                           └──────────────┬──────────────┘
                                          │ 3D detections
                                          ▼
┌───────────────────┐       ┌─────────────────────────────┐
│ RPLIDAR A1M8      │──────▶│ Human tracking + fusion     │
│ filtered /scan    │       │ Hungarian + Kalman filter   │
└───────────────────┘       │ camera ↔ LiDAR association  │
                            └──────────────┬──────────────┘
                                          │ /planning/tracked_humans
                         ┌────────────────┴─────────────────┐
                         ▼                                  ▼
          ┌─────────────────────────┐         ┌──────────────────────────┐
          │ AGHPM social cost layer │         │ Human-aware local control│
          │ asymmetric Gaussian     │         │ predictive trajectory     │
          │ covariance + prediction │         │ sampling and social risk  │
          └────────────┬────────────┘         └─────────────┬────────────┘
                       │                                     │
                local/global costmaps                        │
                       │                                     │
                       ▼                                     ▼
          ┌─────────────────────────┐             ┌─────────────────────┐
          │ Nav2 A* global planning │────────────▶│ velocity smoother   │
          └─────────────────────────┘             └──────────┬──────────┘
                                                             │
                                                             ▼
                                                  ┌─────────────────────┐
                                                  │ Nav2 Collision      │
                                                  │ Monitor             │
                                                  └──────────┬──────────┘
                                                             │ cmd_vel_raw
                                                             ▼
                                                  ┌─────────────────────┐
                                                  │ LiDAR safety layer  │
                                                  │ stop/slow/TTC guard │
                                                  └──────────┬──────────┘
                                                             │ /cmd_vel
                                                             ▼
                                                  ┌─────────────────────┐
                                                  │ ESP32 + micro-ROS   │
                                                  │ BTS7960 + encoders  │
                                                  └─────────────────────┘
```

The important design choice is that **human-aware behavior is not implemented as only one costmap layer**. Human information is used at multiple levels:

- the global planner sees social costs and can choose a more socially appropriate route;
- the local controller predicts human motion and scores candidate robot trajectories using human distance, TTC, social cost, heading, path following and smoothness;
- the final LiDAR safety node remains independent of camera perception and acts as a last physical collision-avoidance layer.

---

## 2. Hardware

| Component | Configuration |
|---|---|
| Robot base | Custom 2WD differential-drive platform |
| Robot computer | Intel NUC |
| OS | Ubuntu 24.04 |
| ROS | ROS 2 Jazzy |
| Microcontroller | ESP32 |
| Motor driver | BTS7960 |
| Motors | 2 DC geared motors with quadrature encoders |
| IMU | MPU9250 |
| 2D LiDAR | RPLIDAR A1M8 |
| RGB-D camera | Intel RealSense D435 |
| Low-level communication | micro-ROS serial |
| Visualization workstation | ROS 2 / RViz over LAN |

### Final ESP32 calibration used by the robot

The custom ESP32 configuration is stored in:

```text
linorobot2_hardware/config/custom/esp32_config.h
```

Current calibrated values:

| Parameter | Value |
|---|---:|
| Base type | `DIFFERENTIAL_DRIVE` |
| Motor driver | `BTS7960` |
| IMU | `MPU9250` |
| Motor max RPM | 110 RPM |
| Max RPM ratio | 0.60 |
| Operating voltage | 12 V |
| Measured supply voltage | 11.7 V |
| Encoder CPR, left | 1799 |
| Encoder CPR, right | 1799 |
| Wheel diameter | 0.09 m |
| Left-right wheel distance | 0.30 m |
| PID | `Kp=0.6, Ki=0.8, Kd=0.5` |
| micro-ROS baud rate | 921600 |
| PWM | 10 bit, 20 kHz |

The encoder CPR was re-measured manually on **2026-09-17** from repeated measurements around 1796–1802 counts/revolution and set to 1799. Wheel diameter was also corrected to 0.09 m.

The two motor directions and encoder signs are inverted in the ESP32 configuration to match the actual mechanical mounting.

### IMU handling

The MPU9250 accelerometer has a scale correction:

```text
9.81 / 10.68 = 0.918230
```

applied to all three acceleration axes.

For the real robot, IMU yaw / yaw-rate was removed from the EKF fusion after testing showed a stationary gyro-Z bias large enough to make the robot rotate in RViz while physically stationary. Wheel odometry therefore remains the trusted heading source in the current real-robot EKF configuration.

---

## 3. Sensor Configuration

### RPLIDAR A1M8

Physical connection:

```text
ESP32        /dev/ttyUSB0
RPLIDAR A1   /dev/ttyUSB1
RPLIDAR link /dev/rplidar
```

The LiDAR runs at 115200 baud.

The scan pipeline is:

```text
/scan_raw  →  laser_filters  →  /scan
```

A small angular region blocked by the robot structure is removed before Nav2 consumes the scan. This prevents the robot from continuously detecting its own chassis as an obstacle.

### Intel RealSense D435

The D435 is used for human perception and 3D localization.

Typical real-robot configuration:

```text
RGB:   640 x 480 @ 15 FPS
Depth: 640 x 480 @ 15 FPS
```

For the social-navigation pipeline, depth is aligned to the color frame before 3D back-projection because YOLO keypoints are extracted in RGB pixel coordinates.

The social-navigation launch also supports a hardware reset of the D435 at startup. This was added after repeated USB/driver states where the RealSense node remained alive but no image frames were published.

---

## 4. Software Stack

The repository combines the standard linorobot2 stack with custom social-navigation packages.

```text
Human-Aware---Social-Navigation/
│
├── linorobot2_hardware/
│   ├── calibration/
│   ├── config/
│   │   └── custom/
│   │       └── esp32_config.h
│   ├── firmware/
│   └── test_motors/
│
├── linorobot2_ws1/
│   └── src/
│       ├── linorobot2/
│       │   ├── linorobot2_base/
│       │   ├── linorobot2_bringup/
│       │   ├── linorobot2_description/
│       │   ├── linorobot2_gazebo/
│       │   └── linorobot2_navigation/
│       │
│       ├── linorobot2_viz/
│       ├── micro_ros_setup/
│       ├── uros/
│       │
│       ├── social_nav/
│       │   ├── social_nav_perception/
│       │   ├── social_nav_tracking/
│       │   ├── social_nav_tracking_cpp/
│       │   ├── social_nav_costmap_layer/
│       │   ├── social_nav_controller/
│       │   ├── social_nav_safety/
│       │   ├── social_nav_safety_cpp/
│       │   └── social_nav_gazebo/
│       │
│       └── thesis_msgs/
│
└── docs/
    └── images/
```

---

## 5. linorobot2 Base

The robot keeps the architecture of [linorobot2](https://github.com/linorobot/linorobot2):

```text
/cmd_vel
   │
   ▼
ESP32 micro-ROS firmware
   │
   ├── motor PID / BTS7960
   ├── wheel encoders
   └── MPU9250
   │
   ├── /odom/unfiltered
   └── /imu/data
          │
          ▼
robot_localization EKF
          │
          ▼
        /odom
```

On top of this, linorobot2 provides URDF/TF, sensor bringup, SLAM Toolbox, AMCL, Nav2, RViz and Gazebo support.

### Main TF chain

```text
map
 └── odom
      └── base_footprint
           └── base_link
                ├── lidar
                ├── IMU
                └── RealSense camera frames
```

For navigation, AMCL publishes the `map → odom` correction while odometry provides `odom → base_footprint`.

---

## 6. Human Perception

Package:

```text
social_nav/social_nav_perception
```

Main node:

```text
yolo_pose_node1
```

The current pipeline uses **YOLO pose exported to ONNX**. Pose estimation is intentionally retained instead of switching to a bounding-box-only detector because body keypoints are also used to estimate human orientation. Orientation is important for the asymmetric social-space model.

The perception node consumes:

```text
/camera/color/image_raw
/camera/depth/... 
/camera/color/camera_info
```

and produces multi-person 3D pose information used by the tracking stage.

The current launch configuration uses a 0.3 s detection period to reduce CPU load on the NUC and supports an optional debug image topic.

---

## 7. Human Tracking and Camera-LiDAR Fusion

Packages:

```text
social_nav_tracking
social_nav_tracking_cpp
```

The C++ implementation is the default implementation for the real robot because it is substantially lighter on CPU than the original Python implementation.

The tracker combines:

- RGB-D human detections;
- LiDAR point clusters;
- Hungarian assignment for global data association;
- constant-velocity Kalman filtering;
- track confirmation and timeout logic;
- camera-to-LiDAR handover;
- covariance propagation;
- human velocity and motion direction estimation;
- temporary person memory and ID revival;
- short-term prediction when a moving person becomes temporarily unobserved.

Output:

```text
/planning/tracked_humans
```

The tracker publishes human state in the `odom` frame for the local social-navigation components.

### Why both camera and LiDAR are used

The camera provides semantic confirmation and pose/orientation. LiDAR provides robust geometric observation even when the person leaves the camera field of view.

The tracker therefore tries to avoid two common failure modes:

1. creating a second LiDAR-only track for a person already detected by the camera;
2. assigning a new ID every time a person briefly stops, leaves the RGB-D field of view, or is temporarily occluded.

The implemented person-memory mechanism stores recently confirmed tracks for a short TTL and can revive the previous ID when a compatible detection reappears.

---

## 8. AGHPM Social Cost Layer

Package:

```text
social_nav_costmap_layer
```

Plugin:

```text
social_nav_costmap_layer::AghpmLayer1
```

The layer subscribes to:

```text
/planning/tracked_humans
```

and adds a human social field directly into Nav2 costmaps.

The core model is an **asymmetric Gaussian** aligned with the person's heading:

```text
                 larger front space
                        ↑
                  . . . . . . .
               .                 .
             .         human       .
               .                 .
                  . . . . .
                        ↓
                 smaller rear space
```

Baseline social-space parameters in the tuned configuration are approximately:

| Parameter | Value |
|---|---:|
| `sigma_front` | 0.50 m |
| `sigma_back` | 0.30 m |
| `sigma_side` | 0.40 m |
| Human cost radius | 0.80 m |
| Maximum social cost | 220 |
| Maximum considered human distance | 10 m |

The social cost is deliberately kept below Nav2 lethal/inscribed obstacle cost. Humans therefore remain **soft social constraints** instead of being converted into permanent walls.

### Dynamic extensions

The AGHPM layer also adapts the social field using:

- human velocity;
- track observation age;
- Kalman covariance;
- future predicted human positions;
- time-decayed prediction cost.

The future-position model uses short horizon constant-velocity prediction so that a moving person influences not only the cell currently occupied, but also the area likely to be occupied shortly afterward.

The AGHPM layer is used in both the local and global costmaps in the latest social-navigation configuration so that both the local controller and the global A* planner are aware of people.

---

## 9. Global Planning

The Nav2 planner server uses:

```text
nav2_navfn_planner::NavfnPlanner
```

with:

```yaml
use_astar: true
```

The effective global planning problem is therefore:

```text
static map cost
+ LiDAR obstacle cost
+ inflation cost
+ AGHPM human social cost
        ↓
      A*
        ↓
socially-aware global path
```

This allows the global planner to prefer a longer path with lower human-social cost instead of forcing all human avoidance into the local planner.

---

## 10. Human-Aware Local Controller

Package:

```text
social_nav_controller
```

Plugin:

```text
social_nav_controller::HumanAwareController1
```

This controller replaces the original Rotation Shim + Regulated Pure Pursuit configuration in the social-navigation mode.

It samples candidate translational and angular velocities, simulates short robot trajectories and scores them using several terms:

```text
goal progress
+ forward speed
+ global-path tracking
+ social costmap cost
+ human proximity risk
+ human time-to-collision risk
+ heading alignment
+ command smoothness
```

The controller also predicts human motion over the trajectory horizon and evaluates robot-human separation in space and time.

Typical tuned values:

| Parameter | Value |
|---|---:|
| Max forward velocity | 0.25 m/s |
| Min velocity | -0.15 m/s |
| Max angular velocity | 1.0 rad/s |
| Linear samples | 7 |
| Angular samples | 9 |
| Simulation horizon | 1.5 s |
| Simulation step | 0.1 s |
| Lookahead distance | 0.60 m |
| `w_goal` | 3.0 |
| `w_speed` | 1.0 |
| `w_path` | 0.8 |
| `w_social` | 5.0 |
| `w_human_proximity` | 6.0 |
| `w_human_ttc` | 3.0 |
| `w_heading` | 1.0 |
| `w_smooth` | 0.5 |

Reverse velocity is intentionally allowed so that the robot can back away from an unexpectedly close person instead of having only stop-and-rotate behavior available.

---

## 11. Costmaps and Robot Footprint

The tuned Nav2 configuration uses:

```text
resolution = 0.05 m/cell
robot_radius = 0.17 m
```

The robot radius was derived from the physical/CAD dimensions rather than left at the generic linorobot2 default.

The local costmap contains:

```text
VoxelLayer
→ AGHPM social layer
→ InflationLayer
```

The global costmap contains:

```text
StaticLayer
→ ObstacleLayer
→ AGHPM social layer
→ InflationLayer
```

Layer order is important: the social layer is inserted before inflation so the final inflation stage can also provide smooth gradients around social-cost regions.

---

## 12. Independent LiDAR Safety Layer

Packages:

```text
social_nav_safety
social_nav_safety_cpp
```

The safety node runs after Nav2's Collision Monitor.

Velocity chain:

```text
controller
   ↓
velocity_smoother
   ↓
Nav2 Collision Monitor
   ↓
/cmd_vel_raw
   ↓
LiDAR safety node
   ↓
/cmd_vel
   ↓
ESP32
```

The LiDAR safety logic independently checks front/rear collision geometry, stopping distance, slowdown distance and time-to-collision.

Current safety parameters include:

| Parameter | Value |
|---|---:|
| Control rate | 20 Hz |
| Scan timeout | 0.25 s |
| Command timeout | 0.25 s |
| Robot safety radius | 0.20 m |
| Slow distance | 0.80 m |
| Stop distance | 0.45 m |
| Emergency distance | 0.30 m |
| TTC slow | 2.0 s |
| TTC stop | 1.0 s |
| Release distance | 0.60 m |
| Release hold | 0.30 s |

This layer is deliberately independent of YOLO, tracking and social costs. If camera perception fails, the robot still has a geometric LiDAR-based collision guard.

---

## 13. Maps and Localization

The real robot uses Nav2 + AMCL for localization.

Main real-robot navigation map:

```text
linorobot2_ws1/src/linorobot2/linorobot2_navigation/maps/map_nav_v2.yaml
linorobot2_ws1/src/linorobot2/linorobot2_navigation/maps/map_nav_v2.pgm
```

Map resolution:

```text
0.05 m/pixel
```

Additional maps in the workspace are used for SLAM, simulation and repeated real-robot tests.

---

## 14. Build

### ROS 2 workspace

Install ROS 2 Jazzy and dependencies first, then:

```bash
cd ~/linorobot2_ws1
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

For development it is useful to add:

```bash
source ~/linorobot2_ws1/install/setup.bash
```

to the shell environment after a successful build.

### ESP32 firmware

The firmware uses PlatformIO.

```bash
cd ~/linorobot2_hardware/firmware
pio run -e esp32
pio run -e esp32 -t upload
```

The ESP32 upload/monitor port is configured as:

```text
/dev/ttyUSB0
```

---

## 15. Real-Robot Bringup

### Device preparation

```bash
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1
sudo ln -sfn /dev/ttyUSB1 /dev/rplidar
```

### Environment

```bash
export ROS_DOMAIN_ID=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export LINOROBOT2_BASE=2wd
export LINOROBOT2_LASER_SENSOR=a1
```

### Base robot

```bash
cd ~/linorobot2_ws1
source install/setup.bash

ros2 launch linorobot2_bringup bringup.launch.py \
  base_serial_port:=/dev/ttyUSB0 \
  extra:=true
```

The `extra:=true` option is required in the deployed setup because the additional launch path is used for the full robot configuration.

### Navigation

```bash
ros2 launch linorobot2_navigation navigation.launch.py \
  map:=/home/nuc/linorobot2-jazzy123/linorobot2_navigation/maps/map_nav_v2.yaml
```

Set the initial pose in RViz before sending a navigation goal.

---

## 16. Social Navigation Bringup

The custom social-navigation launch is:

```text
linorobot2_bringup/launch/social_nav.launch.py
```

For the real robot with RealSense enabled:

```bash
ros2 launch linorobot2_bringup social_nav.launch.py \
  camera:=true \
  tracker_impl:=cpp
```

Useful launch arguments:

| Argument | Meaning |
|---|---|
| `sim` | use Gazebo simulation time |
| `camera` | launch RealSense D435 |
| `camera_reset` | hardware-reset D435 during startup |
| `camera_prefix` | camera topic prefix |
| `debug_image` | publish YOLO debug image |
| `tracker_impl` | `cpp` or `py` tracker |

For Gazebo:

```bash
ros2 launch linorobot2_bringup social_nav.launch.py \
  sim:=true \
  camera:=false \
  tracker_impl:=cpp
```

---

## 17. Important ROS Topics

| Topic | Purpose |
|---|---|
| `/cmd_vel` | final safe velocity to ESP32 |
| `/cmd_vel_raw` | Collision Monitor output before custom LiDAR safety |
| `/odom/unfiltered` | encoder odometry from micro-ROS |
| `/odom` | EKF-filtered odometry |
| `/imu/data` | IMU data |
| `/scan_raw` | raw RPLIDAR scan |
| `/scan` | filtered scan used by Nav2 |
| `/camera/color/image_raw` | RGB stream |
| `/camera/aligned_depth_to_color/image_raw` | aligned depth |
| `/planning/tracked_humans` | fused tracked-human states |
| `/local_costmap/costmap` | Nav2 local costmap |
| `/global_costmap/costmap` | Nav2 global costmap |

---

## 18. Calibration Notes

The project is intentionally calibrated from measured robot values rather than generic defaults.

Important real-robot corrections made during development include:

1. encoder CPR recalibration to 1799 counts/revolution;
2. wheel diameter correction to 0.09 m;
3. maximum usable velocity reduced to match the real motor/firmware limit;
4. blocked LiDAR sectors filtered from `/scan`;
5. IMU yaw removed from EKF because of stationary gyro bias;
6. RealSense depth aligned to RGB before 3D human projection;
7. RealSense topic namespace simplified so simulation and hardware use the same camera topic layout;
8. camera optical-frame orientation corrected for valid RGB-D back-projection;
9. tracker timeouts increased to match the actual YOLO inference period on the NUC;
10. person-memory logic added to reduce identity fragmentation during temporary observation loss.

---

## 19. Design Motivation

Conventional mobile-robot navigation normally treats people as moving obstacles. This project instead treats a person as both:

- a physical collision object; and
- a dynamic social entity with orientation, motion, uncertainty and preferred personal space.

The navigation stack therefore tries to avoid behavior such as passing unnecessarily close to a person, repeatedly stopping whenever a person is detected, cutting directly through a person's frontal social space, or immediately forgetting a moving person after a short sensing dropout.

The project is especially focused on keeping the system practical for a CPU-only real robot instead of relying on a large GPU computer.

---

## 20. Relationship to linorobot2

This repository does **not** reimplement the entire robot stack. It uses linorobot2 as the tested base for:

- micro-ROS robot integration;
- differential-drive odometry;
- robot_localization EKF;
- URDF and TF;
- LiDAR / RGB-D sensor integration;
- SLAM Toolbox;
- AMCL;
- Nav2;
- RViz and Gazebo infrastructure.

The project-specific contribution is the integration and calibration of the real platform plus the human-aware perception, tracking, social-cost, planning, control and safety extensions.

Upstream project:

[https://github.com/linorobot/linorobot2](https://github.com/linorobot/linorobot2)

---

## 21. Troubleshooting

### No ESP32 data

Check:

```bash
ls -l /dev/ttyUSB0
ros2 topic echo /odom/unfiltered
```

Make sure the micro-ROS serial agent is running and the baud/device configuration matches the firmware.

### No LiDAR scan

Check:

```bash
ls -l /dev/rplidar
ros2 topic hz /scan_raw
ros2 topic hz /scan
```

If `/scan_raw` exists but `/scan` does not, inspect the laser filter node/configuration.

### Robot rotates in RViz while stationary

Verify the EKF configuration. The real robot currently does not fuse IMU yaw/yaw-rate because the measured gyro bias caused false rotation.

### YOLO runs but no tracked humans appear

Check all three camera streams:

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
ros2 topic echo /camera/color/camera_info --once
```

Then inspect:

```bash
ros2 topic echo /planning/tracked_humans
```

A RealSense node can remain alive while publishing no image frames after a USB/driver failure; use `camera_reset:=true` in that case.

### Tracking IDs change too frequently

Check camera rate, tracker timeouts, LiDAR association and whether the person is actually being camera-confirmed. The current configuration uses longer track/coast windows and person-memory revival specifically to reduce this failure mode.

### Robot becomes too conservative around people

Do not immediately reduce the physical safety layer. First distinguish which layer is responsible:

```text
AGHPM social cost
HumanAwareController risk terms
Nav2 Collision Monitor
custom LiDAR safety node
```

These layers have different purposes and changing all of them together makes tuning difficult.

---

## 22. Repository Notes

Generated build products are intentionally excluded from version control:

```text
build/
install/
log/
.pio/
__pycache__/
```

Only source code, calibration/configuration, models, maps, launch files and documentation should be committed.

---

## 23. Credits

The mobile-robot base architecture is derived from and calibrated on:

- [linorobot2](https://github.com/linorobot/linorobot2)
- [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware)
- ROS 2 Jazzy
- Nav2
- micro-ROS
- robot_localization
- SLAM Toolbox
- Intel RealSense ROS
- RPLIDAR ROS 2

Custom human-aware navigation, tracking, social-cost and safety integrations in this repository were developed for the Human-Aware / Social Navigation research platform.

---

## License

Upstream linorobot2 and linorobot2_hardware components retain their respective licenses. See the license files inside the corresponding source directories for details.
