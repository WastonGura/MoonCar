#!/usr/bin/env bash
# Full software regression, including real ROS preflight receipts; never starts Gazebo.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /work/install/setup.bash
export PYTHONPATH=/source/scripts${PYTHONPATH:+:$PYTHONPATH}
export ROS_DOMAIN_ID=95 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export MOONCAR_IMAGE_DDS_RUNTIME=/opt/mooncar/image-dds/rmw-8.4.4
cp /evidence/models/mooncar_r5.urdf /evidence/r5-before.urdf
cp /evidence/models/mooncar_r5.sdf /evidence/r5-before.sdf
python3 /source/scripts/preflight_image_dds_production.py --runtime "$MOONCAR_IMAGE_DDS_RUNTIME" --output /evidence/derived-preflight > /evidence/derived-preflight.log 2>&1
python3 /source/scripts/preflight_image_dds_boundary.py --output /evidence/boundary-preflight-final > /evidence/boundary-preflight.log 2>&1
python3 /source/scripts/preflight_image_dds_lifecycle.py --output /evidence/lifecycle-preflight > /evidence/lifecycle-preflight.log 2>&1
python3 /source/scripts/preflight_image_dds_teardown.py --output /evidence/teardown-preflight > /evidence/teardown-preflight.log 2>&1
bash /source/scripts/check_software.sh > /evidence/software-final.log 2>&1
