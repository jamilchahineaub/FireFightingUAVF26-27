#!/bin/bash
# start gz (the chosen world with the cl415 in it) and px4 sitl attached to it
#   wsl -d AMR -- bash /mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl/run_sim.sh                 runway
#   wsl -d AMR -- VARIANT=water bash /mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl/run_sim.sh   water
# env: VARIANT=runway|water, HEADLESS=1 (no gz gui), QGC=1 (also start qgroundcontrol inside wsl),
#      KEEP_PARAMS=1 (keep px4's saved params)
# ground station: Mission Planner on windows, connection type UDPCl, host = the wsl ip printed below, port 18570
# stop with px4_sitl/stop_sim.sh
set -e
HERE=/mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
VARIANT=${VARIANT:-runway}
if [ "$VARIANT" = "water" ]; then WORLD=cl415_water; AUTOSTART=4051; else WORLD=cl415_runway; AUTOSTART=4050; fi
LOGS=$HOME/cl415_sim_logs
mkdir -p "$LOGS"
source /opt/ros/jazzy/setup.bash

export GZ_SIM_RESOURCE_PATH="$HERE/models:$HOME/cl415_ws/install/cl415_description/share:$GZ_SIM_RESOURCE_PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/cl415_gz_plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GZ_IP=127.0.0.1                     # keep gz-transport on loopback inside wsl
# camera sensors (chase cam, tail cam) render in the server: hardware gl through wslg's d3d12 path
export DISPLAY=${DISPLAY:-:0} GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}

bash "$HERE/stop_sim.sh" > /dev/null 2>&1 || true

setsid gz sim -r -s -v 3 "$HERE/worlds/$WORLD.sdf" > "$LOGS/gz_server.log" 2>&1 < /dev/null &
for i in $(seq 1 60); do
  gz topic -l 2>/dev/null | grep -q "/world/$WORLD/clock" && break
  sleep 1
done
gz topic -l | grep -q "/world/$WORLD/clock" || { echo "gz server did not come up, see $LOGS/gz_server.log"; exit 1; }
echo "gz server up: $WORLD ($VARIANT)"

if [ -z "$HEADLESS" ]; then
  setsid gz sim -g -v 1 > "$LOGS/gz_gui.log" 2>&1 < /dev/null &
fi
if [ "$VARIANT" = "water" ]; then
  # hull state (buoyancy, resistance, planing lift, draft) for the flight analysis
  setsid gz topic -e -t /cl415/hull --json-output > "$LOGS/hull_state.jsonl" 2>/dev/null < /dev/null &
fi

# px4 attaches to the model already in the world
cd "$PX4_DIR"
if [ -z "$KEEP_PARAMS" ]; then
  rm -f build/px4_sitl_default/rootfs/parameters.bson build/px4_sitl_default/rootfs/parameters_backup.bson
fi
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=$AUTOSTART PX4_GZ_MODEL_NAME=cl415 PX4_GZ_WORLD=$WORLD \
  setsid ./build/px4_sitl_default/bin/px4 -d > "$LOGS/px4.log" 2>&1 < /dev/null &
echo "px4 started, airframe $AUTOSTART (log $LOGS/px4.log)"

WSL_IP=$(ip -4 addr show eth0 | grep -oP '(?<=inet )[\d.]+')
echo "Mission Planner (windows): connection type UDPCl, host $WSL_IP, port 18570"

if [ -n "$QGC" ]; then
  cd "$LOGS"
  QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-xcb} setsid "$HOME/tools/QGroundControl.AppImage" --appimage-extract-and-run \
    > "$LOGS/qgc.log" 2>&1 < /dev/null &
  echo "qgroundcontrol started (log $LOGS/qgc.log)"
fi
