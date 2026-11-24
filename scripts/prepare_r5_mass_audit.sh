#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
mkdir -p /work/src
cp -a /source/ros2_ws/src/mooncar_description /source/ros2_ws/src/mooncar_gazebo /source/ros2_ws/src/mooncar_control /work/src/
cd /work
colcon build --merge-install --packages-select mooncar_description mooncar_gazebo mooncar_control > /evidence/overlay-build.log 2>&1
source /work/install/setup.bash
mkdir /evidence/models
for variant in mooncar_r5 mooncar; do
  xacro /work/install/share/mooncar_description/urdf/$variant.urdf.xacro > /evidence/models/$variant.urdf
  gz sdf -p /evidence/models/$variant.urdf > /evidence/models/$variant.sdf 2> /evidence/models/$variant-conversion.log
done
dpkg-query -W > /evidence/environment-packages.txt
python3 -m unittest discover -s /source/ros2_ws/src/mooncar_gazebo/tests -p test_mass_audit.py -v > /evidence/mass-tests.log 2>&1
python3 /source/scripts/build_r5_mass_catalog.py --repo /source --hardware /hardware --output /evidence/catalog
python3 /source/scripts/audit_r5_mass.py --urdf /evidence/models/mooncar_r5.urdf --sdf /evidence/models/mooncar_r5.sdf --catalog /evidence/catalog/catalog.json --output /evidence/result > /evidence/audit.log 2>&1
