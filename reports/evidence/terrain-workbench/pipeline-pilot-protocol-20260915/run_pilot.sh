#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /work/ros2_ws/install/setup.bash
set -uo pipefail
cd /source
python3 reports/evidence/terrain-workbench/research-freeze-20260914/check_freeze.py
python3 reports/evidence/terrain-workbench/pipeline-pilot-protocol-20260915/check_protocol.py
output=/evidence/pipeline-pilot-20260915
if [[ -e "$output/plan.json" ]]; then
  echo "Refusing to overwrite an existing pilot: $output" >&2
  exit 2
fi
export ROS_DOMAIN_ID=89 OPENBLAS_NUM_THREADS=1 MOONCAR_CJK_FONT=/evidence/fonts/msyh.ttc
python3 scripts/compare_survey_strategies.py \
  --budget 30 \
  --repeats 1 \
  --scene visible_ramp_v1 \
  --start -1.4 1.55 -0.3 \
  --start 6.5 -2.0 2.4 \
  "$output"
