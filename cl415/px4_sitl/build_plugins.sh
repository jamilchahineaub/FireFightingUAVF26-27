#!/bin/bash
# compile the cl415 gz plugins (prop map) into ~/cl415_gz_plugins
set -e
source /opt/ros/jazzy/setup.bash

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/gz_plugins" && pwd)"
OUT=${OUT:-$HOME/cl415_gz_plugins}

cmake -S "$SRC" -B "$OUT" -G Ninja > /dev/null
cmake --build "$OUT" -j 8
ls -la "$OUT"/libCl415*.so