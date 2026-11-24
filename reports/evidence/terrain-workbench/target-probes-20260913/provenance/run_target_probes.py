import sys,json,hashlib,shutil,datetime
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from capture_slope_resampling import run
from compare_survey_strategies import request,wait_for
from survey_evaluation import write_json
root=Path('/evidence/target-probes-20260913');root.mkdir(exist_ok=False)
trials=[]
for repeat in (1,2):
    for arm in (('no_turn','turn') if repeat==1 else ('turn','no_turn')):
        trials.append(dict(folder=f'ramp-repeat{repeat}-{arm}',pair=f'ramp-repeat{repeat}',arm=arm,repeat=repeat,
            scene='visible_ramp_v1',start=[-1.4,1.55,-.3],goal=[-.65,1.3],seconds=40,observation_yaw=0.))
files=['scripts/swept_resampling.py','scripts/skip_probe.py','scripts/capture_slope_resampling.py','scripts/trajectory_evaluation.py',
       'scripts/arrival_resampling.py','scripts/autonomous_survey.py','scripts/observation_path.py','scripts/experiment_motion.py',
       'ros2_ws/src/mooncar_terrain/scripts/terrain_map.py','tmp/evaluate_target_probes.py']
write_json(root/'plan.json',dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),trials=trials,deltas=[.02,-.02,.04,-.04,.08,-.08],
    source_sha256={p:hashlib.sha256((Path('/source')/p).read_bytes()).hexdigest() for p in files},
    scope='Pre-move deficient cells frozen before the Nav2 move; newly discovered deficits are separate future targets',
    protocol=['40simseconds; >=5s initial observation then freeze targets; Nav2 move;5s common arrival holding; select first nonempty target-negative candidate with v4.',
    'Maximum feature age5s, remaining>=12s. Nonzero turn executed despite skip; actual same direction, >=half requested, error<=.02+.25*requested, requested abs>=.01rad.',
    'Two independent ramp repeats counterbalanced with hold controls. Require2 actual target-negative actions and2 nonempty shared target cohorts, no target turn-only recovery over10s from command.',
    'Report global shared-deficit recovery separately, including gains outside initial targets. This is NOT validation of the earlier all-observed stopping rule or permission to stop global exploration.',
    'Existing captures used for development and candidate availability only. Fresh captures for this narrow target screen; no automatic online deployment.']))
(root/'source').mkdir()
for p in files:shutil.copyfile(Path('/source')/p,root/'source'/Path(p).name)
shutil.copyfile(__file__,root/'runner.py')
try:
    for t in trials:
        print('START',t['folder'],flush=True)
        try:
            run(root/t['folder'],conditions=['approach'],goal=t['goal'],seconds=40,start=t['start'],scene=t['scene'],nav2_plan=True,
                observe_turn=t['arm']=='turn',arrival_wait=5.,targeted_probe=True)
        except Exception as error:
            write_json(root/(t['folder']+'-failure.json'),dict(error=str(error)));request('stop',{})
        wait_for(lambda s:s['phase']=='stopped',90)
finally:request('stop',{})
write_json(root/'finished.json',dict(finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
print('FOUR TARGET PROBE ATTEMPTS FINISHED',flush=True)
