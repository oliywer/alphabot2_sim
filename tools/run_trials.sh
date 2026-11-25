#!/usr/bin/env bash
# Run repeatable patrol trials, each from a FRESH start (new Gazebo + new SLAM map).
#
# Usage:  tools/run_trials.sh <default|custom> [number_of_trials] [first_index]
#   e.g.  tools/run_trials.sh default 3        -> results/default_1..3.csv
#         tools/run_trials.sh custom 2 2       -> results/custom_2..3.csv
#
# Note: kills running Gazebo / Nav2 / RViz / slam_toolbox processes between trials.
# (no "set -u": ROS setup.bash uses unset variables)
MODE=${1:?"usage: $0 <default|custom> [N] [first_index]"}
N=${2:-3}
START=${3:-1}

source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

RES="$HOME/ros2_ws/results"
mkdir -p "$RES/logs" "$RES/maps"

BT_ARGS=()
if [ "$MODE" = "custom" ]; then
  BT="$(ros2 pkg prefix alphabot2_sim)/share/alphabot2_sim/behavior_trees/patrol_small_robot.xml"
  BT_ARGS=(-p "behavior_tree:=$BT")
elif [ "$MODE" != "default" ]; then
  echo "MODE must be 'default' or 'custom'"; exit 1
fi

cleanup() {
  pkill -INT -f "ros2 launch alphabot2_sim" 2>/dev/null
  sleep 5
  for p in "gz sim" parameter_bridge robot_state_publisher rviz2 slam_toolbox \
           controller_server planner_server behavior_server bt_navigator \
           lifecycle_manager waypoint_patrol _ros2_daemon; do
    pkill -9 -f "$p" 2>/dev/null
  done
  sleep 2
}

for i in $(seq "$START" $((START + N - 1))); do
  echo "================ $MODE trial $i ================"
  cleanup
  rm -rf "$RES/tmp"

  ros2 launch alphabot2_sim sim.launch.py > "$RES/logs/${MODE}_${i}_sim.log" 2>&1 &
  sleep 15
  ros2 launch alphabot2_sim nav.launch.py > "$RES/logs/${MODE}_${i}_nav.log" 2>&1 &
  sleep 10

  # waypoint_patrol waits for Nav2 itself; hard limit 20 min per trial
  timeout 1200 ros2 run alphabot2_sim waypoint_patrol.py --ros-args \
      -p "results_dir:=$RES/tmp" "${BT_ARGS[@]}"

  # save the SLAM map of this trial (to compare maps of failed vs. successful runs)
  timeout 30 ros2 run nav2_map_server map_saver_cli -f "$RES/maps/${MODE}_${i}" \
      > "$RES/logs/${MODE}_${i}_mapsaver.log" 2>&1 \
    && echo ">>> map saved $RES/maps/${MODE}_${i}.pgm" \
    || echo ">>> map NOT saved (see logs/${MODE}_${i}_mapsaver.log)"

  if ls "$RES"/tmp/patrol_*.csv >/dev/null 2>&1; then
    mv "$RES"/tmp/patrol_*.csv "$RES/${MODE}_${i}.csv"
    echo ">>> saved $RES/${MODE}_${i}.csv"
  else
    echo ">>> trial $i produced no CSV (see $RES/logs/${MODE}_${i}_*.log)"
  fi
done

cleanup
echo "All $MODE trials done."
