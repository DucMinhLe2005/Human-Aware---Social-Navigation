#!/bin/bash
# Phase-randomised evaluation batch (kept outside the frozen src_humanaware/src).
#
# Why: the pedestrian in dymap walks a fixed 13.4286 s cycle and run_batch.sh
# always sends the goal at the same simulation time, so every run meets the
# pedestrian at the same phase. The collision rate depends strongly on that
# phase, so a fixed-phase batch measures one arbitrary situation.
#
# What this runner does differently from run_batch.sh:
#   1. Before each goal it waits an extra delay. The N delays are stratified
#      over one pedestrian cycle (one per 1/N of the cycle, jittered, shuffled)
#      and generated from SEED, so two methods run with the same SEED face the
#      same schedule (paired design).
#   2. It runs exactly N scheduled runs. A run that times out or is aborted is a
#      failure and is NOT repeated. Only infrastructure failures (stack did not
#      start, goal never accepted) are retried, up to 3 times, and stay on disk.
#   3. It releases the batch lock in child processes and stops the stack at the end.
#
# Usage: run_phase_batch.sh [N=200] [output_dir]      env: SEED (default 20261010)
#
# Resuming an interrupted batch: START_SLOT=<k> continues the SAME schedule from
# slot k in the SAME output directory (new run directories continue the
# numbering). The seed, N and the installed configuration must be unchanged;
# the script checks the schedule and the configuration hash and writes
# resume.txt. A resumed batch must be reported as such.
set +u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${WS:-$(cd "$HERE/../src_humanaware" && pwd)}"
S="$WS/src/social_nav/social_nav_gazebo/tools"
GOAL_X=11.1760; GOAL_Y=2.7660; GOAL_YAW=1.2430
PERIOD=13.4286                      # pedestrian cycle in dymap.sdf (s)
N="${1:-200}"
SEED="${SEED:-20261010}"
OUT_ROOT="${2:-$WS/results/$(date +%Y%m%d_%H%M)_phase_${N}}"
MAX_TRIES=3
export LOG_DIR="$WS/log/run_batch"
mkdir -p "$OUT_ROOT" "$LOG_DIR"
START_SLOT="${START_SLOT:-1}"
if [ "$START_SLOT" -le 1 ] && ls -d "$OUT_ROOT"/run_* >/dev/null 2>&1; then
  echo "output directory already contains runs -- aborting (use START_SLOT to resume)"; exit 1
fi
if [ "$START_SLOT" -gt 1 ] && [ ! -f "$OUT_ROOT/schedule.csv" ]; then
  echo "START_SLOT given but $OUT_ROOT has no schedule.csv -- aborting"; exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

exec 9>"$LOG_DIR/run_batch.lock"
if ! flock -n 9; then
  echo "another batch is already running -- aborting"; exit 1
fi

SCHED="$OUT_ROOT/schedule.csv"; [ "$START_SLOT" -gt 1 ] && SCHED="$(mktemp)"
python3 - "$N" "$SEED" "$PERIOD" > "$SCHED" <<'PY'
import random, sys
n, seed, period = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
rng = random.Random(seed)
delays = [(k + rng.random()) * period / n for k in range(n)]
rng.shuffle(delays)
print('slot,extra_delay_s')
for k, d in enumerate(delays, 1):
    print(f'{k},{d:.3f}')
PY

INST_YAML="$WS/install/linorobot2_navigation/share/linorobot2_navigation/config/navigation.yaml"
if [ "$START_SLOT" -gt 1 ]; then
  cmp -s "$SCHED" "$OUT_ROOT/schedule.csv" || { echo "schedule differs from the original batch (N or SEED changed) -- aborting"; exit 1; }
  rm -f "$SCHED"
  OLD_MD5=$(grep '^navigation.yaml md5:' "$OUT_ROOT/code_version.txt" | awk '{print $3}')
  NEW_MD5=$(md5sum "$(readlink -f "$INST_YAML")" | cut -c1-12)
  [ "$OLD_MD5" = "$NEW_MD5" ] || { echo "installed navigation.yaml ($NEW_MD5) differs from the batch ($OLD_MD5) -- aborting"; exit 1; }
  LAST=$(ls -d "$OUT_ROOT"/run_* 2>/dev/null | sed 's/.*run_//' | sort -n | tail -1)
  echo "resumed: $(date '+%F %T') from slot $START_SLOT; run directories continue after run_${LAST:-0}; commit $(git -C "$WS" log --oneline -1 2>/dev/null)" >> "$OUT_ROOT/resume.txt"
fi
[ "$START_SLOT" -gt 1 ] || {
  echo "started: $(date '+%F %T')"
  echo "commit:  $(git -C "$WS" log --oneline -1 2>/dev/null)"
  echo "runner:  analysis/run_phase_batch.sh  N=$N  SEED=$SEED  PERIOD=$PERIOD"
  echo "schedule md5: $(md5sum "$OUT_ROOT/schedule.csv" | cut -c1-12)"
  INST="$WS/install/linorobot2_navigation/share/linorobot2_navigation/config/navigation.yaml"
  echo "navigation.yaml md5: $(md5sum "$(readlink -f "$INST")" | cut -c1-12)"
  echo "controller plugin: $(grep -m1 -A1 '^ *FollowPath:' "$INST" | grep plugin | sed 's/.*plugin: *//')"
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
stop_stack() {
  pkill -INT -f "social_nav.launch.py|navigation.launch.py|gazebo.launch.py" 2>/dev/null; sleep 6
  pkill -KILL -f "social_nav.launch.py|navigation.launch.py|gazebo.launch.py|gz sim|human_tracker_node1|yolo_pose_node1|lidar_safety_node1|component_container_isolated|parameter_bridge|ekf_node|robot_state_publisher|social_nav_metrics|command_timeout|ros_gz_bridge" 2>/dev/null
}

echo "[$(date '+%F %T')] START: $N scheduled runs (seed $SEED), from slot $START_SLOT -> $OUT_ROOT"
i=0; DONE=0; INFRA=0
if [ "$START_SLOT" -gt 1 ]; then i="${LAST:-0}"; DONE=$((START_SLOT-1)); fi
while IFS=, read -r SLOT DELAY; do
  [ "$SLOT" = "slot" ] && continue
  [ "$SLOT" -lt "$START_SLOT" ] && continue
  TRY=0
  while :; do
    TRY=$((TRY+1)); i=$((i+1))
    echo "===== run $i | slot $SLOT/$N try $TRY | delay ${DELAY}s | $(date +%T) | CPU $(cpu_temp)C ====="
    RUN="$OUT_ROOT/run_$i"; mkdir -p "$RUN"
    echo "slot=$SLOT try=$TRY extra_delay_s=$DELAY" > "$RUN/phase.txt"
    "$S/restart_all.sh" 9>&- > "$LOG_DIR/restart_$i.log" 2>&1
    sleep 8
    pkill -INT -f "[s]ocial_nav_metrics" 2>/dev/null; sleep 3
    pkill -KILL -f "[s]ocial_nav_metrics" 2>/dev/null; sleep 1
    ros2 run social_nav_gazebo social_nav_metrics --ros-args -p use_sim_time:=true \
      -p output_dir:="$RUN" 9>&- > "$RUN/metrics.log" 2>&1 &
    # The recorder starts first so the bag always covers the whole goal window.
    ros2 bag record -o "$RUN/bag" --use-sim-time "${BAG_TOPICS[@]}" 9>&- > "$RUN/bag.log" 2>&1 &
    BAG_PID=$!
    sleep 4
    sleep "$DELAY"
    python3 "$S/send_goal.py" --x $GOAL_X --y $GOAL_Y --yaw $GOAL_YAW --timeout 200 9>&- 2>&1 | tail -1 | tee "$RUN/goal.txt"
    kill -TERM $BAG_PID 2>/dev/null; wait $BAG_PID 2>/dev/null
    pkill -INT -f "[s]ocial_nav_metrics" 2>/dev/null; sleep 5
    JSON=$(ls -t "$RUN"/run_*.json 2>/dev/null | head -1)
    [ -n "$JSON" ] && mv "$JSON" "$RUN/metrics.json"
    cp "$LOG_DIR/navigation.log" "$RUN/navigation.log" 2>/dev/null
    GOAL="$(cat "$RUN/goal.txt" 2>/dev/null)"
    # Infrastructure failure = the goal never started. Everything else counts.
    case "$GOAL" in
      GOAL_REACHED*|TIMEOUT*|GOAL_ENDED*)
        if [ -f "$RUN/metrics.json" ]; then
          echo "  -> COUNTED ($GOAL)"; DONE=$((DONE+1)); break
        fi
        REASON="no metrics.json" ;;
      *) REASON="goal: ${GOAL:-<empty>}" ;;
    esac
    INFRA=$((INFRA+1))
    echo "  !! INFRASTRUCTURE FAILURE ($REASON)"
    echo "infrastructure_failure: $REASON" >> "$RUN/phase.txt"
    if [ "$TRY" -ge "$MAX_TRIES" ]; then
      echo "  !! slot $SLOT failed $MAX_TRIES times -- stopping the batch"; stop_stack; exit 1
    fi
  done
  if [ $((DONE % 10)) -eq 0 ] && [ "$DONE" -lt "$N" ]; then
    echo "[$(date +%T)] $DONE runs done -- cooling down for 3 minutes (CPU $(cpu_temp)C)"
    sleep 180
  fi
done < "$OUT_ROOT/schedule.csv"
stop_stack
echo "[$(date '+%F %T')] DONE: $DONE scheduled runs, $INFRA infrastructure retries, $i run directories"
