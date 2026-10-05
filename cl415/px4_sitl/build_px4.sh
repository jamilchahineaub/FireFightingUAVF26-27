#!/bin/bash
# build px4 sitl with the gz bridge against the ros jazzy gz vendor libs
#   wsl -d AMR -- bash /mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl/build_px4.sh
set -e
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
VENV=${VENV:-$HOME/px4_venv}
JOBS=${JOBS:-10}                     # 7 GB in wsl, full parallel runs out of memory
source /opt/ros/jazzy/setup.bash
export PATH="$VENV/bin:$PATH"
cd "$PX4_DIR"
make px4_sitl_default j="$JOBS"
echo BUILD_OK
