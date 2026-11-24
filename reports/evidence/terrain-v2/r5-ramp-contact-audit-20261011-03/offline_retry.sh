#!/usr/bin/env bash
# Software only. No Gazebo server or physical experiment.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd /probe
colcon --log-base /probe/offline-work/log build --base-paths /source/ros2_ws/src/mooncar_gazebo --packages-select mooncar_gazebo --symlink-install --build-base /probe/offline-work/build --install-base /probe/offline-work/install > /probe/offline-overlay.log 2>&1
source /probe/offline-work/install/setup.bash
PYTHONPATH=/source/scripts bash /source/scripts/check_software.sh > /probe/software-regression-retry.log 2>&1
