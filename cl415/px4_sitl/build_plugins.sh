#!/bin/bash
# compile the cl415 gz plugins (prop map) into ~/cl415_gz_plugins
set -e
source /opt/ros/jazzy/setup.bash
SRC=/mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl/gz_plugins
OUT=${OUT:-$HOME/cl415_gz_plugins}
cmake -S "$SRC" -B "$OUT" -G Ninja -DCMAKE_MAKE_PROGRAM=$HOME/px4_venv/bin/ninja > /dev/null
cmake --build "$OUT" -j 8
ls -la "$OUT"/libCl415*.so
