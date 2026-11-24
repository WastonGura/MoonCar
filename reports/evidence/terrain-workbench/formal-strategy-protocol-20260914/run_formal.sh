#!/usr/bin/env bash
set -e
root=/evidence/formal-strategy-20260914
mkdir -p "$root"
if [[ -f "$root/campaign.log" ]]; then
  suffix=$(date -u +%Y%m%dT%H%M%SZ)
  cp "$root/campaign.log" "$root/campaign-before-resume-$suffix.log"
  [[ ! -f "$root/FAILED" ]] || cp "$root/FAILED" "$root/FAILED-before-resume-$suffix"
fi
exec >"$root/campaign.log" 2>&1
trap 'code=$?; date -u +%FT%TZ > "$root/FAILED"; exit $code' EXIT
source /opt/ros/jazzy/setup.bash
source /work/ros2_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=89 OPENBLAS_NUM_THREADS=1 MOONCAR_CJK_FONT=/evidence/fonts/msyh.ttc
python3 /source/scripts/compare_survey_strategies.py "$root/survey" --budget 90 --repeats 3 --scene survey --start -15 -16 0 --start 15 16 3.141592653589793
python3 /source/scripts/compare_survey_strategies.py "$root/visible-ramp" --budget 90 --repeats 3 --scene visible_ramp_v1 --start -1.4 1.55 -0.3 --start 6.5 -2 2.4
python3 /source/scripts/compare_survey_strategies.py "$root/visible-curve" --budget 90 --repeats 3 --scene visible_curve_v1 --start -1.4 1.55 -0.3 --start 6.5 -2 2.4
date -u +%FT%TZ > "$root/COMPLETE"
rm -f "$root/FAILED"
trap - EXIT
