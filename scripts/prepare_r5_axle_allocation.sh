#!/usr/bin/env bash
set -eo pipefail
bash /source/scripts/prepare_r5_slip_probe.sh
source /opt/ros/jazzy/setup.bash
source /work/install/setup.bash
export PYTHONPATH=/source/scripts${PYTHONPATH:+:$PYTHONPATH}
mkdir /evidence/allocator
cmake -S /source/containers/r5_axle_allocation -B /work/allocator-build > /evidence/allocator-configure.log 2>&1
cmake --build /work/allocator-build -j4 > /evidence/allocator-build.log 2>&1
cp /work/allocator-build/libmooncar-axle-allocation.so /evidence/allocator/
python3 - <<'PY'
from r5_axle_allocation import generate
generate('/evidence/slip-models/candidate.sdf','/evidence/allocator-models')
PY
cp -a /evidence/derived-preflight-ready /evidence/derived-preflight
cp -a /evidence/boundary-preflight /evidence/boundary-preflight-final
python3 -m unittest discover -s /source/ros2_ws/src/mooncar_gazebo/tests -p test_axle_allocation.py -v > /evidence/allocator-tests.log 2>&1
ldd /evidence/allocator/libmooncar-axle-allocation.so > /evidence/allocator-dependencies.txt
