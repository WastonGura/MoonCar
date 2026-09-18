#!/usr/bin/env bash
base=/evidence/formal-strategy-v2-tf-replay-300s-20260918
report=/source/reports/evidence/terrain-workbench/formal-strategy-v2-tf-replay-results-20260918
analyzer=/source/reports/evidence/terrain-workbench/formal-strategy-v2-300s-20260918/analyze.py
source /work/ros2_ws/install/setup.bash
set -u
export MOONCAR_FORMAL_RUN="$base" MOONCAR_FORMAL_REPORT="$report"

fail() {
  printf '%s\n' "$1" >&2
  printf '%s\n' "$1" > "$base/campaign-failure.txt"
  printf '%s\n' 1 > "$base/campaign-exit-code.txt"
  exit 1
}

[[ ! -e "$base/campaign-started.txt" ]] || fail 'Campaign already started; refusing duplicate run'
[[ $(sha256sum "$base/protocol.json" | cut -d' ' -f1) == $(cat "$base/protocol.sha256") ]] || fail 'Registered protocol hash changed'
registered_analysis=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["analysis_sha256"])' "$base/protocol.json")
[[ $(sha256sum "$analyzer" | cut -d' ' -f1) == "$registered_analysis" ]] || fail 'Registered analysis script changed'
date -u +%FT%TZ > "$base/campaign-started.txt"

for folder in survey visible-ramp visible-curve; do
  [[ ! -e "$base/$folder" ]] || fail "$folder output already exists; refusing overwrite"
  if [[ $folder == survey ]]; then
    scene=survey
    starts=(--start -15 -16 0 --start 15 16 3.141592653589793)
  elif [[ $folder == visible-ramp ]]; then
    scene=visible_ramp_v1
    starts=(--start -1.4 1.55 -0.3 --start 6.5 -2 2.4)
  else
    scene=visible_curve_v1
    starts=(--start -1.4 1.55 -0.3 --start 6.5 -2 2.4)
  fi
  python3 /source/scripts/compare_survey_strategies.py "$base/$folder" \
    --budget 300 --repeats 3 "${starts[@]}" --scene "$scene" \
    --strategy nearest_view_v2 --strategy coverage_view_v2 --strategy surface_view_v2 \
    > "$base/$folder-run.log" 2>&1
  result=$?
  printf '%s\n' "$result" > "$base/$folder-exit-code.txt"
  [[ $result == 0 ]] || fail "$folder runner failed; inspect its trial records"
  python3 "$analyzer" "$scene" > "$base/$folder-audit.log" 2>&1 || fail "$folder evidence audit failed"
done

python3 "$analyzer" > "$base/final-analysis.log" 2>&1 || fail 'Final 54-trial analysis failed'
printf '%s\n' 0 > "$base/campaign-exit-code.txt"
