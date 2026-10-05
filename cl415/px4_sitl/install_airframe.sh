#!/bin/bash
# copy the cl415 airframes into px4's posix airframes and rebuild (incremental, under a minute)
set -e
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
HERE=/mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl
DST="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
for af in "$HERE"/airframes/*; do
  name=$(basename "$af")
  cp "$af" "$DST/$name"
  if ! grep -q "$name" "$DST/CMakeLists.txt"; then
    sed -i "s/\(\s*\)4001_gz_x500$/&\n\1$name/" "$DST/CMakeLists.txt"
  fi
  grep -n "$name" "$DST/CMakeLists.txt"
done
bash "$HERE/build_px4.sh" | tail -2
