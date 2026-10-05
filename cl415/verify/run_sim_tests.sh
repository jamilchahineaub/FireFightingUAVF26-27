#!/bin/bash
# build cl415_description in a WSL workspace and run the headless gazebo checks through the ros 2 launch file
#   bash verify/run_sim_tests.sh            (from the project root, inside the AMR distro or any jazzy + harmonic box)
# results: verify/out/<scenario>.csv and .launch.log, analysed by verify/analyse.py on the windows side
source /opt/ros/jazzy/setup.bash
PROJ="$(cd "$(dirname "$0")/.." && pwd)"
WS="$HOME/cl415_ws"
OUT="$PROJ/verify/out"
mkdir -p "$WS/src" "$OUT"
ln -sfn "$PROJ/cl415_description" "$WS/src/cl415_description"
cd "$WS" && colcon build --symlink-install --packages-select cl415_description 2>&1 | tail -n 2
source "$WS/install/setup.bash"
echo "GZ_SIM_RESOURCE_PATH after sourcing: $GZ_SIM_RESOURCE_PATH"

cleanup() {
  pkill -INT -f "ros2 launch cl415_description" 2>/dev/null
  sleep 3
  pkill -f "gz sim" 2>/dev/null; pkill -f parameter_bridge 2>/dev/null; pkill -f robot_state_publisher 2>/dev/null
  pkill -f "ros_gz_sim/create" 2>/dev/null
  sleep 1
}

run() {   # name, sim seconds, probe mode, launch args...
  local name=$1 dur=$2 mode=$3
  shift 3
  echo "== $name ($*)"
  cleanup
  local t0=$(date +%s)
  ros2 launch cl415_description sim.launch.py headless:=true "$@" > "$OUT/$name.launch.log" 2>&1 &
  timeout $((dur * 4 + 90)) python3 "$PROJ/verify/sim_probe.py" "$OUT/$name.csv" "$dur" "$mode" || echo "   probe did not finish"
  echo "   wall time $(( $(date +%s) - t0 )) s for $dur s of sim"
  cleanup
  grep -iE "\[err\]|error|exception" "$OUT/$name.launch.log" | grep -v "use_sim_time" | head -n 5
  tail -n 1 "$OUT/$name.csv" | cut -d, -f1-10
}

run water_full 30 none world:=water tank:=full
run water_full_damped 30 none world:=water tank:=full water_damping:=true
run water_empty_damped 30 none world:=water tank:=empty water_damping:=true
run ground_full 15 none world:=ground tank:=full
run water_actuate 20 actuate world:=water tank:=full water_damping:=true

# the exported SDF model in plain gazebo, no ROS at all
echo "== sdf_only (models/cl415_full in worlds/water_cl415.sdf, gz only)"
cleanup
GZ_SIM_RESOURCE_PATH="$PROJ/cl415_description/models:$PROJ" gz sim -s -r -v 3 "$PROJ/cl415_description/worlds/water_cl415.sdf" > "$OUT/sdf_only.log" 2>&1 &
sleep 25
gz topic -e -t /cl415/odometry -n 1 > "$OUT/sdf_only_odom.txt"
gz topic -e -t /stats -n 1 > "$OUT/sdf_only_stats.txt"
cleanup
grep -A3 "position" "$OUT/sdf_only_odom.txt" | head -4
grep -E "real_time_factor|^sim_time" -A2 "$OUT/sdf_only_stats.txt" | head -6
grep -iE "\[err\]|error" "$OUT/sdf_only.log" | head -5
echo "all runs done"
