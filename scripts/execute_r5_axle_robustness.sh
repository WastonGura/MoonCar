#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /work/install/setup.bash
export PYTHONPATH=/source/scripts${PYTHONPATH:+:$PYTHONPATH}
python3 /source/scripts/run_r5_axle_robustness.py --output /evidence/campaign > /evidence/campaign.log 2>&1
