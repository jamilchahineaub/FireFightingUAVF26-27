#!/bin/bash
# open-loop hull test, no autopilot: gazebo alone with the water world, a fixed throttle and a fixed elevator,
# the pose polled every 2 s and the hull state recorded.  THR (0..1), ELEV (joint rad, + = TE down), T (s)
#   wsl -d AMR -- THR=0.7 ELEV=-0.3 bash /mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl/hull_test.sh
set -e
HERE=/mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl
THR=${THR:-0.7}; ELEV=${ELEV:--0.3}; T=${T:-60}
LOGS=$HOME/cl415_sim_logs
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH="$HERE/models:$HOME/cl415_ws/install/cl415_description/share:$GZ_SIM_RESOURCE_PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/cl415_gz_plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GZ_IP=127.0.0.1
bash "$HERE/stop_sim.sh" > /dev/null 2>&1 || true
setsid gz sim -r -s -v 1 "$HERE/worlds/cl415_water.sdf" > "$LOGS/gz_hulltest.log" 2>&1 < /dev/null &
for i in $(seq 1 60); do gz topic -l 2>/dev/null | grep -q "/cl415/hull" && break; sleep 1; done
sleep 6                                             # settle on the water
setsid gz topic -e -t /cl415/hull --json-output > "$LOGS/hull_test_state.jsonl" 2>/dev/null < /dev/null &
cmd=$(python3 -c "print(int($THR*1000))")
gz topic -t /model/cl415/servo_2 -m gz.msgs.Double -p "data: $ELEV"
gz topic -t /cl415/command/motor_speed -m gz.msgs.Actuators -p "velocity: [$cmd, $cmd]"
echo "throttle $THR elevator $ELEV rad"
echo "   t     x      y      z   roll  pitch   yaw"
for i in $(seq 0 2 $T); do
  gz model -m cl415 -p 2>/dev/null | grep -A2 "Pose \[" | tail -2 | tr -d '[]' | tr '\n' ' ' | awk -v t=$i '{printf "%4d %7.1f %6.1f %6.3f %6.1f %6.1f %6.1f\n", t, $1, $2, $3, $4*57.3, -$5*57.3, $6*57.3}'
  # re-send the command in case the plugin loaded late
  gz topic -t /cl415/command/motor_speed -m gz.msgs.Actuators -p "velocity: [$cmd, $cmd]" 2>/dev/null
  sleep 2
done
bash "$HERE/stop_sim.sh" > /dev/null 2>&1 || true
