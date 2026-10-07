#!/usr/bin/env bash
# Simulation environment. Usage: source env_sim.sh   (in every terminal)

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Environment variables injected by the VS Code snap make rviz2 / gz crash.
unset GTK_PATH GTK_EXE_PREFIX LOCPATH GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR \
      GTK_IM_MODULE_FILE XDG_DATA_HOME XDG_DATA_DIRS 2>/dev/null || true

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u 2>/dev/null || true

# Required: with another base the camera and lidar mounts are wrong.
export LINOROBOT2_BASE=my_amr
# Map used by navigation.launch.py (also its default).
export LINOROBOT2_MAP=dymap_slam

echo "simulation environment ready:"
echo "  LINOROBOT2_BASE = $LINOROBOT2_BASE"
echo "  LINOROBOT2_MAP  = $LINOROBOT2_MAP"
echo ""
echo "  T1  ros2 launch linorobot2_gazebo gazebo.launch.py world_name:=dymap spawn_x:=0.5"
echo "  T2  ros2 launch linorobot2_navigation navigation.launch.py sim:=true"
echo "  T3  ros2 launch linorobot2_bringup social_nav.launch.py sim:=true"
echo "  T4  rviz2 -d \$(ros2 pkg prefix linorobot2_navigation)/share/linorobot2_navigation/rviz/linorobot2_navigation.rviz --ros-args -p use_sim_time:=true"
