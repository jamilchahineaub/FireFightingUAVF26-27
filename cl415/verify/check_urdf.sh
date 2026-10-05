#!/bin/bash
# xacro -> urdf for both tank cases, check_urdf, and urdf -> sdf with gz sdf (Harmonic, sdformat 14)
# run in WSL:  bash verify/check_urdf.sh
set -e
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
D=cl415_description
OUT=verify/out
mkdir -p $OUT
for tank in full empty; do
  xacro $D/urdf/cl415.urdf.xacro tank:=$tank > $OUT/cl415_$tank.urdf
  echo "== check_urdf tank:=$tank"
  check_urdf $OUT/cl415_$tank.urdf | tee $OUT/check_urdf_$tank.txt | head -5
  grep -c "<link" $OUT/cl415_$tank.urdf | sed 's/^/links: /'
  gz sdf -p $OUT/cl415_$tank.urdf > $OUT/cl415_$tank.sdf
  echo "sdf links: $(grep -c '<link name' $OUT/cl415_$tank.sdf), joints: $(grep -c '<joint name' $OUT/cl415_$tank.sdf), plugins: $(grep -c '<plugin' $OUT/cl415_$tank.sdf)"
  gz sdf -k $OUT/cl415_$tank.sdf && echo "gz sdf -k: valid"
done
