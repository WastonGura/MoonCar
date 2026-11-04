#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
mkdir -p /work/ros2_ws /evidence
cd /work/ros2_ws
colcon build --base-paths /source/ros2_ws/src > /evidence/runtime-build.log 2>&1
source /work/ros2_ws/install/setup.bash
python3 -B /source/scripts/runtime_baseline.py verify \
  --manifest /evidence/runtime-source.json --install /work/ros2_ws/install \
  --output /evidence/runtime-baseline.json
