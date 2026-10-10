#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
test ! -e /work/install
test ! -e /work/build
mkdir -p /work
cd /work
colcon --log-base /work/log build --base-paths /source/ros2_ws/src --symlink-install --merge-install --build-base /work/build --install-base /work/install > /probe/overlay-build.log 2>&1
source /work/install/setup.bash
python3 /source/scripts/prepare_r5_contact_geometry.py --evidence /evidence --output /probe/fixtures > /probe/fixtures-prepare.log
mkdir /probe/metadata
cmake -S /source/containers/r5_contact_metadata -B /work/metadata-build > /probe/metadata-configure.log 2>&1
cmake --build /work/metadata-build -j4 > /probe/metadata-build.log 2>&1
cp /work/metadata-build/libmooncar-contact-metadata.so /probe/metadata/
ldd /probe/metadata/libmooncar-contact-metadata.so > /probe/metadata-dependencies.txt
for name in plate coplanar_mesh mirrored_mesh; do gz sdf -k "/probe/fixtures/$name.sdf" > "/probe/$name-sdf-check.log" 2>&1; done
python3 -m unittest discover -s /source/ros2_ws/src/mooncar_gazebo/tests -p test_contact_geometry.py -v > /probe/geometry-tests.log 2>&1
python3 /source/scripts/preflight_r5_contact_geometry.py > /probe/preflight.log
test -z "$(pgrep -x 'gz sim' || true)"
