#!/bin/bash
# stop px4, gz and qgc started by run_sim.sh
pkill -f "px4_sitl_default/bin/px4" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f "QGroundControl" 2>/dev/null
pkill -f "gz topic -e" 2>/dev/null
sleep 1
pkill -9 -f "px4_sitl_default/bin/px4" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "QGroundControl" 2>/dev/null
true
