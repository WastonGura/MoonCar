#!/usr/bin/env bash
set -eo pipefail
bash /source/scripts/prepare_r5_turn_slip.sh
source /opt/ros/jazzy/setup.bash
source /work/install/setup.bash
export PYTHONPATH=/source/scripts${PYTHONPATH:+:$PYTHONPATH}
python3 /source/scripts/r5_test_startup_guard.py \
  --source /source/ros2_ws/src/mooncar_gazebo/launch/image_dds_guard.py \
  --output /evidence/test-startup/image_dds_guard.py > /evidence/test-startup-generation.log
python3 -m unittest discover -s /source/ros2_ws/src/mooncar_gazebo/tests -p test_r5_startup_guard.py -v > /evidence/startup-tests.log 2>&1
python3 -m unittest discover -s /source/ros2_ws/src/mooncar_gazebo/tests -p test_turn_gate_validation.py -v > /evidence/gate-entry-tests.log 2>&1
bash -n /source/scripts/prepare_r5_turn_gate_validation.sh /source/scripts/execute_r5_turn_gate_validation.sh
