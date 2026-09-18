#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
mkdir -p /work/ros2_ws /work/scripts /work/firmware
cp -a /source/ros2_ws/src /work/ros2_ws/
cp -a /source/scripts/. /work/scripts/
cp -a /source/firmware/. /work/firmware/
cp -a /source/web /work/
cd /work
find ros2_ws/src scripts firmware web -type f ! -path '*/node_modules/*' ! -path '*/__pycache__/*' -print0 \
  | sort -z | xargs -0 sha256sum > /evidence/source-sha256.txt
cd /work/ros2_ws
colcon build --symlink-install --executor sequential
source install/setup.bash
bash /work/scripts/check_software.sh
python3 -m compileall -q src
echo 'Container build and ROS software regression passed; Gazebo runtime is a separate gate.'
