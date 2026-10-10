#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /work/install/setup.bash
python3 /source/scripts/run_r5_contact_geometry.py --output /probe/campaign > /probe/campaign.log 2>&1
