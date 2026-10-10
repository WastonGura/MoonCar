#!/usr/bin/env bash
set -eo pipefail
bash /source/scripts/prepare_r5_axle_allocation.sh
source /opt/ros/jazzy/setup.bash
source /work/install/setup.bash
export PYTHONPATH=/source/scripts${PYTHONPATH:+:$PYTHONPATH}
mkdir /evidence/contact-observer
cmake -S /source/containers/r5_contact_observer -B /work/contact-observer-build > /evidence/contact-configure.log 2>&1
cmake --build /work/contact-observer-build -j4 > /evidence/contact-build.log 2>&1
cp /work/contact-observer-build/libmooncar-contact-observer.so /evidence/contact-observer/
ldd /evidence/contact-observer/libmooncar-contact-observer.so > /evidence/contact-dependencies.txt
python3 - <<'PY'
from r5_contact_source import generate
generate('/evidence/allocator-models/candidate.sdf','/evidence/contact-models')
PY
python3 -m unittest discover -s /source/ros2_ws/src/mooncar_gazebo/tests -p test_contact_source.py -v > /evidence/contact-tests.log 2>&1
