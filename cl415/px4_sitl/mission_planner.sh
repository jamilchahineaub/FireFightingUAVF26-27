#!/bin/bash
# start mission planner on windows and print the connection settings for the px4 sitl running in wsl
#   bash px4_sitl/mission_planner.sh          (from git bash on windows)
MP="/c/Program Files (x86)/Mission Planner/MissionPlanner.exe"
WSL_IP=$(MSYS_NO_PATHCONV=1 wsl -d AMR -- hostname -I | awk '{print $1}' | tr -d '\r')
echo "Mission Planner: top right, connection type UDPCl, Connect, host $WSL_IP, port 18570"
[ -f "$MP" ] && (cd "$(dirname "$MP")" && cmd //c start "" "MissionPlanner.exe") || echo "Mission Planner not found at $MP"
