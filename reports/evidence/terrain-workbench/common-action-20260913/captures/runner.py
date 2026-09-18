import sys,json,hashlib,shutil,datetime
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from capture_slope_resampling import run
from compare_survey_strategies import request,wait_for
from survey_evaluation import write_json
root=Path('/evidence/common-action-20260913');root.mkdir(exist_ok=False)
trials=[]
for repeat,arms in [(3,('hold','wide_turn','advance','small_turn'))]:
    for arm in arms:trials.append(dict(folder=f'ramp-repeat{repeat}-{arm}',repeat=repeat,arm=arm,
        scene='visible_ramp_v1',start=[-1.4,1.55,-.3],goal=[-.65,1.3],next_goal=[.1,1.05],seconds=60))
files=['scripts/compare_observation_actions.py','scripts/capture_slope_resampling.py','scripts/trajectory_evaluation.py',
       'scripts/skip_probe.py','scripts/swept_resampling.py','scripts/survey_evaluation.py','scripts/observation_path.py',
       'scripts/resampling_dataset.py','scripts/autonomous_survey.py','scripts/experiment_motion.py',
       'ros2_ws/src/mooncar_terrain/scripts/terrain_map.py','tmp/evaluate_common_action.py','scripts/action_support.py','scripts/compare_survey_strategies.py']
write_json(root/'plan.json',dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),trials=trials,
    source_sha256={p:hashlib.sha256((Path('/source')/p).read_bytes()).hexdigest() for p in files},
    protocol=['Four new resets as a supplemental quartet; previous8-attempt protocol and failures remain unchanged. Hold, +.02rad, +.4rad, next Nav2 location then restore decision heading.',
    '60simseconds, >=5s pre-move baseline and frozen targets, same initial Nav2 move,5s arrival holding. All arms query the same alternative path.',
    'Pre-action sensor-only checkpoint <=5s old and >=22s remaining. Primary equal20simsecond window from command, INCLUDING movement/turn costs.',
    'ROI is fixed radius8m disk around initial XY intersected with50m world. Utility=new observed area + recovered previously deficient area; report each separately, plus loss of previously valid area.',
    'Retain shared-cohort comparisons: old deficits common to all4arms; novel cells absent from all4decision maps. Pre-move target versus outside-target separate. No per-arm subtraction of unequal eligible sets.',
    'Record failures and incomplete windows; never replace a failed run. Require4 valid windows and successful prescribed movement/headings for comparison completeness, not superiority.',
    'Common known-surface-v5 ray/voxel/fit model for all actions; predicted recovery minus predicted loss over20s. Unknown new-area prediction and total ranking are absent. No online admission.',
    'Report both original positive utility and loss-adjusted utility; shared loss cohort is the intersection of initially valid cells across all4arms.',
    'No parameter fitting on outcomes, no significance/accuracy/hardware claim; wide-turn .4rad and .75m onward displacement fixed in advance.']))
(root/'source').mkdir()
for p in files:shutil.copyfile(Path('/source')/p,root/'source'/Path(p).name)
shutil.copyfile(__file__,root/'runner.py')
try:
    for t in trials:
        print('START',t['folder'],flush=True)
        try:run(root/t['folder'],conditions=['approach'],goal=t['goal'],seconds=t['seconds'],start=t['start'],scene=t['scene'],
                nav2_plan=True,arrival_wait=5.,comparison_action=dict(arm=t['arm'],goal=t['next_goal']))
        except Exception as error:
            write_json(root/(t['folder']+'-failure.json'),dict(error=str(error)));request('stop',{})
        wait_for(lambda s:s['phase']=='stopped',90)
finally:request('stop',{})
write_json(root/'finished.json',dict(finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
print('FOUR SUPPLEMENTAL COMMON ACTION ATTEMPTS FINISHED',flush=True)
