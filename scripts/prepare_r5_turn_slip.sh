#!/usr/bin/env bash
set -eo pipefail
bash /source/scripts/prepare_r5_contact_source.sh
source /opt/ros/jazzy/setup.bash
source /work/install/setup.bash
export PYTHONPATH=/source/scripts${PYTHONPATH:+:$PYTHONPATH}
mkdir /evidence/turn-slip
cmake -S /source/containers/r5_turn_slip_control -B /work/turn-slip-build > /evidence/turn-slip-configure.log 2>&1
cmake --build /work/turn-slip-build -j4 > /evidence/turn-slip-build.log 2>&1
ctest --test-dir /work/turn-slip-build --output-on-failure > /evidence/turn-slip-policy.log 2>&1
cp /work/turn-slip-build/libmooncar-turn-slip-control.so /evidence/turn-slip/
ldd /evidence/turn-slip/libmooncar-turn-slip-control.so > /evidence/turn-slip-dependencies.txt
python3 - <<'PY'
from r5_turn_slip_control import generate
generate('/evidence/allocator-models/candidate.sdf','/evidence/turn-models')
PY
python3 -m unittest discover -s /source/ros2_ws/src/mooncar_gazebo/tests -p test_turn_slip_control.py -v > /evidence/turn-slip-tests.log 2>&1
