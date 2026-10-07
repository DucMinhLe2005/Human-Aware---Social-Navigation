#!/bin/bash
# Restart the whole simulation stack: Gazebo, navigation and social navigation.
# Used by run_batch.sh before every run so each run starts from the same state.
set +u
S="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$S/../../../.." && pwd)"
LOG="${LOG_DIR:-$WS/log/run_batch}"
mkdir -p "$LOG"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export LINOROBOT2_BASE=my_amr LINOROBOT2_MAP=dymap_slam

echo "[$(date +%T)] stopping everything"
pkill -INT -f "social_nav.launch.py|navigation.launch.py|gazebo.launch.py" 2>/dev/null
sleep 6
pkill -KILL -f "social_nav.launch.py|navigation.launch.py|gazebo.launch.py|gz sim|human_tracker_node1|yolo_pose_node1|lidar_safety_node1|component_container_isolated|parameter_bridge|ekf_node|robot_state_publisher|social_nav_metrics|command_timeout|ros_gz_bridge" 2>/dev/null
sleep 4

# Repeated kill -9 leaves Fast DDS shared-memory files behind; remove the orphans.
python3 "$S/clean_dds_shm.py" 2>/dev/null || echo "[$(date +%T)] could not clean /dev/shm"

echo "[$(date +%T)] Gazebo"
nohup ros2 launch linorobot2_gazebo gazebo.launch.py world_name:=dymap gui:=false spawn_x:=0.5 > "$LOG/gazebo.log" 2>&1 &
for i in $(seq 1 120); do sleep 1; timeout 3 ros2 topic info /scan 2>/dev/null | grep -q "Publisher count: [1-9]" && break; done
echo "[$(date +%T)] /scan publishers: $(timeout 3 ros2 topic info /scan 2>/dev/null | grep -c 'Publisher count: [1-9]')"
sleep 5

echo "[$(date +%T)] navigation"
nohup ros2 launch linorobot2_navigation navigation.launch.py sim:=true initial_pose_x:=0.5 initial_pose_y:=0.0 initial_pose_yaw:=0.0 > "$LOG/navigation.log" 2>&1 &
for i in $(seq 1 120); do sleep 1; timeout 3 ros2 topic info /amcl_pose 2>/dev/null | grep -q "Publisher count: [1-9]" && break; done
echo "[$(date +%T)] /amcl_pose: OK"

echo "[$(date +%T)] social navigation"
nohup ros2 launch linorobot2_bringup social_nav.launch.py sim:=true > "$LOG/social_nav.log" 2>&1 &
for i in $(seq 1 120); do sleep 1; timeout 3 ros2 topic info /planning/tracked_humans 2>/dev/null | grep -q "Publisher count: [1-9]" && break; done
echo "[$(date +%T)] /planning/tracked_humans: OK"
sleep 8
timeout 60 python3 "$S/reset_amcl.py"
