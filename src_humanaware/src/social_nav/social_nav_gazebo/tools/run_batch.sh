#!/bin/bash
# Run N valid start -> goal runs in simulation, restarting the whole stack before
# each run, and store the metrics of every run.
#
# A run is VALID only if the robot reaches the goal (GOAL_REACHED) and moved at
# least 1 m; invalid runs (start-up failures) are repeated. A run with at least
# one collision is still a valid run (a failed one).
#
# Usage: run_batch.sh [runs=100] [output_dir]
#   KEEP_BAG=1  keep the rosbag of every run (default: only runs with collisions)
set +u
S="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$S/../../../.." && pwd)"
GOAL_X=11.1760; GOAL_Y=2.7660; GOAL_YAW=1.2430
RUNS="${1:-100}"
OUT_ROOT="${2:-$WS/results/$(date +%Y%m%d_%H%M)_${RUNS}runs}"
export LOG_DIR="$WS/log/run_batch"
mkdir -p "$OUT_ROOT" "$LOG_DIR"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# Only one batch at a time: two batches would restart each other's simulation.
exec 9>"$LOG_DIR/run_batch.lock"
if ! flock -n 9; then
  echo "another run_batch.sh is already running -- aborting"; exit 1
fi

# Record the exact binaries used, so results can be traced to the code.
{
  echo "started: $(date '+%F %T')"
  echo "commit:  $(git -C "$WS" log --oneline -1 2>/dev/null)"
  for pkg in social_nav_controller social_nav_safety_cpp social_nav_tracking_cpp; do
    for so in "$WS"/install/$pkg/lib/*.so; do
      [ -e "$so" ] || continue
      real=$(readlink -f "$so")
      echo "$pkg $(md5sum "$real" | cut -c1-12) $(stat -c %y "$real" | cut -c1-19)"
    done
  done
} > "$OUT_ROOT/code_version.txt"

BAG_TOPICS=(/social_nav/ground_truth /planning/tracked_humans /cmd_vel /cmd_vel_raw
  /odom /amcl_pose /scan /plan /local_plan /local_costmap/costmap /goal_pose /tf /tf_static)

cpu_temp() { cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -rn | head -1 | awk '{printf "%.0f", $1/1000}'; }
echo "[$(date '+%F %T')] START: $RUNS valid runs -> $OUT_ROOT"
i=0; VALID=0
while [ "$VALID" -lt "$RUNS" ]; do
  i=$((i+1))
  echo "===== cycle $i (valid $VALID / $RUNS) | $(date +%T) | CPU $(cpu_temp)C ====="
  "$S/restart_all.sh" > "$LOG_DIR/restart_$i.log" 2>&1
  sleep 8
  RUN="$OUT_ROOT/run_$i"; mkdir -p "$RUN"
  pkill -INT -f "[s]ocial_nav_metrics" 2>/dev/null; sleep 3
  pkill -KILL -f "[s]ocial_nav_metrics" 2>/dev/null; sleep 1
  ros2 run social_nav_gazebo social_nav_metrics --ros-args -p use_sim_time:=true \
    -p output_dir:="$RUN" > "$RUN/metrics.log" 2>&1 &
  sleep 4
  ros2 bag record -o "$RUN/bag" --use-sim-time "${BAG_TOPICS[@]}" > "$RUN/bag.log" 2>&1 &
  BAG_PID=$!
  python3 "$S/send_goal.py" --x $GOAL_X --y $GOAL_Y --yaw $GOAL_YAW --timeout 200 2>&1 | tail -1 | tee "$RUN/goal.txt"
  # SIGTERM, not SIGINT: background jobs of a script ignore SIGINT and the bag would never stop.
  kill -TERM $BAG_PID 2>/dev/null; wait $BAG_PID 2>/dev/null
  pkill -INT -f "[s]ocial_nav_metrics" 2>/dev/null; sleep 5
  JSON=$(ls -t "$RUN"/run_*.json 2>/dev/null | head -1)
  [ -n "$JSON" ] && mv "$JSON" "$RUN/metrics.json"
  read DIST COLL < <(python3 -c "
import json, os
p = '$RUN/metrics.json'
s = json.load(open(p)) if os.path.exists(p) else {}
print(s.get('robot_distance_m', 0), s.get('collisions_total', 0))" 2>/dev/null)
  DIST=${DIST:-0}; COLL=${COLL:-0}
  if [ "$COLL" -gt 0 ] 2>/dev/null; then
    cp "$LOG_DIR/navigation.log" "$RUN/navigation.log" 2>/dev/null
  elif [ "${KEEP_BAG:-0}" != "1" ]; then
    rm -rf "$RUN/bag"
  fi
  if grep -q "^GOAL_REACHED" "$RUN/goal.txt" 2>/dev/null && awk "BEGIN{exit !($DIST >= 1.0)}"; then
    VALID=$((VALID+1))
    [ "$COLL" -gt 0 ] 2>/dev/null && echo "  -> FAILED RUN ($COLL collisions)" || echo "  -> CLEAN RUN"
  else
    echo "  !! NOT COUNTED (goal: $(cat "$RUN/goal.txt" 2>/dev/null), distance $DIST m) -- will be repeated"
    cp "$LOG_DIR/navigation.log" "$RUN/navigation_invalid.log" 2>/dev/null
  fi
  if [ "$VALID" -gt 0 ] && [ $((VALID % 10)) -eq 0 ] && [ "$VALID" -lt "$RUNS" ]; then
    echo "[$(date +%T)] $VALID valid runs -- cooling down for 3 minutes (CPU $(cpu_temp)C)"
    sleep 180
  fi
done
echo "[$(date '+%F %T')] DONE: $VALID valid runs after $i cycles"
python3 "$S/summarize_batch.py" "$OUT_ROOT" | tee "$OUT_ROOT/summary.txt"
