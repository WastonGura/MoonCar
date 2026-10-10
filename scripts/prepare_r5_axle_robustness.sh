#!/usr/bin/env bash
set -eo pipefail
bash /source/scripts/prepare_r5_axle_allocation.sh
source /opt/ros/jazzy/setup.bash
source /work/install/setup.bash
export PYTHONPATH=/source/scripts${PYTHONPATH:+:$PYTHONPATH}
python3 -m unittest discover -s /source/ros2_ws/src/mooncar_gazebo/tests -p test_axle_robustness.py -v > /evidence/robustness-tests.log 2>&1
