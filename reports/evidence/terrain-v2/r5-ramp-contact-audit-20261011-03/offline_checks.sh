#!/usr/bin/env bash
# No gz sim / ROS launch; only scientific plots, SDF parsing and software checks.
set -eo pipefail
python3 /probe/plot_offline.py
source /opt/ros/jazzy/setup.bash
for name in plate coplanar_mesh mirrored_mesh; do
  gz sdf -k "/probe/fixtures/$name.sdf" > "/probe/fixtures/$name-check.log" 2>&1
done
PYTHONPATH=/source/scripts bash /source/scripts/check_software.sh > /probe/software-regression.log 2>&1
