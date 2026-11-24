#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /work/ros2_ws/install/setup.bash
set -uo pipefail
cd /source
python3 reports/evidence/terrain-workbench/research-freeze-20260914/check_freeze.py
python3 reports/evidence/terrain-workbench/formal-strategy-protocol-20260915/check_protocol.py
root=/evidence/formal-strategy-20260915
if [[ -e "$root/campaign.log" || -e "$root/COMPLETE" ]]; then
  echo "Refusing to overwrite an existing campaign: $root" >&2
  exit 2
fi
mkdir -p "$root"
exec >"$root/campaign.log" 2>&1
trap 'code=$?; date -u +%FT%TZ > "$root/FAILED"; exit $code' EXIT
export ROS_DOMAIN_ID=89 OPENBLAS_NUM_THREADS=1 MOONCAR_CJK_FONT=/evidence/fonts/msyh.ttc
python3 scripts/compare_survey_strategies.py "$root/survey" --budget 90 --repeats 3 --scene survey --start -15 -16 0 --start 15 16 3.141592653589793
python3 scripts/compare_survey_strategies.py "$root/visible-ramp" --budget 90 --repeats 3 --scene visible_ramp_v1 --start -1.4 1.55 -0.3 --start 6.5 -2 2.4
python3 scripts/compare_survey_strategies.py "$root/visible-curve" --budget 90 --repeats 3 --scene visible_curve_v1 --start -1.4 1.55 -0.3 --start 6.5 -2 2.4
date -u +%FT%TZ > "$root/COMPLETE"
rm -f "$root/FAILED"
trap - EXIT
