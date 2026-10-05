#!/bin/bash
# fly the water mission and record it: external chase camera and the onboard tail camera (both server-side
# CameraVideoRecorder, 25 fps in sim time), hull state, px4 log. videos land in px4_sitl/flight_logs/.
#   wsl -d AMR -- bash /mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl/record_flight.sh
# env: VARIANT=water|runway (default water), HEADLESS=1 to skip the gz gui window
set -e
HERE=/mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl
OUT=$HERE/flight_logs
VARIANT=${VARIANT:-water}
STAMP=$(date +%Y-%m-%d_%H-%M)
source /opt/ros/jazzy/setup.bash
export GZ_IP=127.0.0.1

VARIANT=$VARIANT bash "$HERE/run_sim.sh"
sleep 25
# gui follow camera too, if the gui is up
if gz service -l 2>/dev/null | grep -q "^/gui/follow$"; then
  gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: "cl415"' > /dev/null || true
  gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 3000 --req 'x: -6, y: -4, z: 2' > /dev/null || true
fi
CHASE=$OUT/${VARIANT}_chase_$STAMP.mp4
TAIL=$OUT/${VARIANT}_tailcam_$STAMP.mp4
gz service -s /chase/record_video --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 5000 \
  --req "start: true, format: \"mp4\", save_filename: \"$CHASE\""
gz service -s /cl415/record_tailcam --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 5000 \
  --req "start: true, format: \"mp4\", save_filename: \"$TAIL\""
echo "recording started: $CHASE"
sleep 3
if [ "$VARIANT" = "water" ]; then WATER=--water; else WATER=; fi
~/px4_venv/bin/python -u "$HERE/fly_mission.py" $WATER --timeout ${MISSION_TIMEOUT:-2400} | tee ~/cl415_sim_logs/mission_record.log || true
sleep 6
gz service -s /chase/record_video --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 5000 --req "stop: true"
gz service -s /cl415/record_tailcam --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 5000 --req "stop: true"
sleep 8
ls -la "$CHASE" "$TAIL" 2>&1
cp ~/cl415_sim_logs/hull_state.jsonl "$OUT/hull_state_${VARIANT}_$STAMP.jsonl" 2>/dev/null || true
bash "$HERE/stop_sim.sh"
