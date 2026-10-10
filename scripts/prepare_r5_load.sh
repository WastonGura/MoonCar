#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
mkdir -p /work/src
cp -a /source/ros2_ws/src/mooncar_description /source/ros2_ws/src/mooncar_gazebo /source/ros2_ws/src/mooncar_control /work/src/
cd /work
colcon build --merge-install --packages-select mooncar_description mooncar_gazebo mooncar_control > /evidence/overlay-build.log 2>&1
source /work/install/setup.bash
export PYTHONPATH=/source/scripts${PYTHONPATH:+:$PYTHONPATH}
cp /source/reports/evidence/terrain-v2/r5-origin-20261009/r5-before.sdf /source/reports/evidence/terrain-v2/r5-origin-20261009/r5-before.urdf /evidence/
python3 /source/scripts/preflight_image_dds_production.py --runtime "$MOONCAR_IMAGE_DDS_RUNTIME" --output /evidence/derived-preflight-ready > /evidence/derived-preflight.log 2>&1
python3 /source/scripts/preflight_image_dds_boundary.py --output /evidence/boundary-preflight > /evidence/boundary-preflight.log 2>&1
python3 /source/scripts/preflight_image_dds_lifecycle.py --output /evidence/lifecycle-preflight > /evidence/lifecycle-preflight.log 2>&1
python3 /source/scripts/preflight_image_dds_teardown.py --output /evidence/teardown-preflight > /evidence/teardown-preflight.log 2>&1
mkdir /evidence/observer /evidence/source-audit
cp /source/reports/evidence/terrain-v2/r5-origin-20261009/observer/libmooncar-origin-observer.so /source/reports/evidence/terrain-v2/r5-origin-20261009/observer/mooncar-image-observer /evidence/observer/
cp -a /source/reports/evidence/terrain-v2/r5-origin-20261009/source-audit/. /evidence/source-audit/
cp /evidence/derived-preflight-ready/models/mooncar_r5.urdf.xacro.original.sdf /evidence/r5-before.sdf
cp /evidence/derived-preflight-ready/models/mooncar_r5.urdf.xacro.original.urdf /evidence/r5-before.urdf
python3 /source/scripts/preflight_r5_origin.py > /evidence/origin-preflight.log 2>&1
python3 /source/scripts/r5_load_diagnostic.py --source /evidence/r5-d0.sdf --output /evidence/mass-models > /evidence/model-generation.log 2>&1
python3 -m unittest discover -s /source/ros2_ws/src/mooncar_gazebo/tests -p 'test_load_diagnostic.py' -v > /evidence/load-tests-preflight.log 2>&1
tar -czf /evidence/overlay.tar.gz -C /work install
dpkg-query -W > /evidence/environment-packages.txt
python3 /source/scripts/preflight_r5_load.py > /evidence/load-preflight.log 2>&1
