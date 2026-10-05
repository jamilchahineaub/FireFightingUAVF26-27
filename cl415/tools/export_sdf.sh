#!/bin/bash
# xacro -> urdf -> sdf (gz sdf, sdformat 14 / Harmonic) for both tank cases, as gz model folders
#   bash tools/export_sdf.sh          (run inside WSL / Ubuntu with ROS 2 Jazzy sourced or installed)
# then, without ROS:
#   export GZ_SIM_RESOURCE_PATH=<project>/cl415_description/models:<project>
#   gz sim cl415_description/worlds/water_cl415.sdf
source /opt/ros/jazzy/setup.bash
set -e
cd "$(dirname "$0")/.."
D=cl415_description
for tank in full empty; do
  M=$D/models/cl415_$tank
  mkdir -p $M
  xacro $D/urdf/cl415.urdf.xacro tank:=$tank > /tmp/cl415_$tank.urdf
  gz sdf -p /tmp/cl415_$tank.urdf > $M/model.sdf
  gz sdf -k $M/model.sdf > /dev/null
  mass=$([ $tank = full ] && echo 10.5 || echo 7.5)
  cat > $M/model.config <<CFG
<?xml version="1.0"?>
<model>
  <name>cl415_$tank</name>
  <version>0.1</version>
  <sdf version="1.11">model.sdf</sdf>
  <author><name>Jamil Chahine</name></author>
  <description>Scaled CL-415 UAV, 2.51 m span, tank $tank ($mass kg). Generated from cl415_description/urdf/cl415.urdf.xacro by tools/export_sdf.sh. Meshes resolve as model://cl415_description/meshes, so the folder holding cl415_description must be on GZ_SIM_RESOURCE_PATH.</description>
</model>
CFG
  echo "$M: $(grep -c '<link name' $M/model.sdf) links, $(grep -c '<plugin' $M/model.sdf) plugins, mass $(grep -m1 -o '<mass>[^<]*' $M/model.sdf | cut -c7-) kg on base_link"
done
