#!/usr/bin/env bash
base=/evidence/formal-strategy-v2-tf-replay-300s-20260918-font-recovery
original=/evidence/formal-strategy-v2-tf-replay-300s-20260918
analyzer=/source/reports/evidence/terrain-workbench/formal-strategy-v2-300s-20260918/analyze.py
source /work/ros2_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=89 MOONCAR_CJK_FONT=/evidence/fonts/msyh.ttc OPENBLAS_NUM_THREADS=1
export MOONCAR_FORMAL_RUN="$base" MOONCAR_FORMAL_REPORT="$base/analysis"

fail() {
  printf '%s\n' "$1" >&2
  printf '%s\n' "$1" > "$base/recovery-failure.txt"
  printf '%s\n' 1 > "$base/recovery-exit-code.txt"
  exit 1
}

[[ ! -e "$base/recovery-started.txt" ]] || fail 'Recovery already started; refusing duplicate run'
[[ -f "$MOONCAR_CJK_FONT" ]] || fail 'Registered Chinese font not found'
[[ $(sha256sum "$base/protocol.json" | cut -d' ' -f1) == $(cat "$base/protocol.sha256") ]] || fail 'Registered protocol hash changed'
[[ $(cat "$original/survey-exit-code.txt") == 0 ]] || fail 'Original survey runner did not finish'
[[ $(readlink -f "$base/survey") == "$original/survey" ]] || fail 'Original survey evidence link changed'
[[ ! -e "$base/visible-ramp" && ! -e "$base/visible-curve" ]] || fail 'Remaining scene output already exists'
python3 - "$base" "$original" "$analyzer" <<'PY' || fail 'Registered scripts or survey audit changed'
import hashlib, json, pathlib, sys
base, original, analyzer = map(pathlib.Path, sys.argv[1:])
protocol = json.loads((base / 'protocol.json').read_text())
for path, expected in ((original / 'run_campaign.sh', protocol['runner_sha256']),
                       (analyzer, protocol['analysis_sha256'])):
    assert hashlib.sha256(path.read_bytes()).hexdigest() == expected, path
audit = json.loads((original / 'analysis/audit-survey.json').read_text())
assert (audit['status'], audit['trial_count'], audit['reference_conditions'], audit['issues']) == ('pass', 18, 2, [])
PY
python3 "$analyzer" survey > "$base/survey-audit.log" 2>&1 || fail 'Linked survey evidence audit failed'
date -u +%FT%TZ > "$base/recovery-started.txt"

for folder in visible-ramp visible-curve; do
  if [[ $folder == visible-ramp ]]; then scene=visible_ramp_v1; else scene=visible_curve_v1; fi
  python3 /source/scripts/compare_survey_strategies.py "$base/$folder" \
    --budget 300 --repeats 3 \
    --start -1.4 1.55 -0.3 --start 6.5 -2 2.4 --scene "$scene" \
    --strategy nearest_view_v2 --strategy coverage_view_v2 --strategy surface_view_v2 \
    > "$base/$folder-run.log" 2>&1
  result=$?
  printf '%s\n' "$result" > "$base/$folder-exit-code.txt"
  [[ $result == 0 ]] || fail "$folder runner failed; inspect its trial records"
  python3 "$analyzer" "$scene" > "$base/$folder-audit.log" 2>&1 || fail "$folder evidence audit failed"
done

python3 "$analyzer" > "$base/final-analysis.log" 2>&1 || fail 'Final 54-trial analysis failed'
printf '%s\n' 0 > "$base/recovery-exit-code.txt"
