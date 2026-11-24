import sys,json,hashlib,datetime,shutil,math,yaml
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from simulation_experiment import survey_world,validate_task
from capture_slope_resampling import run
from compare_survey_strategies import request,wait_for
root=Path('/evidence/small-angle-validation-20260913');root.mkdir(exist_ok=False)
zones=yaml.safe_load(Path('/source/ros2_ws/src/mooncar_terrain/config/zones.yaml').read_text())['zones']
trials=[]
for repeat in (1,2):
    for scene,start,goal,delta in [('visible_ramp_v1',[-1.4,1.55,-.3],[-.65,1.3],.02),('visible_curve_v1',[-1.75,2.2,-.4],[-1.,1.8],.08)]:
        validate_task(dict(version=1,region=[-25,-25,25,25],start=start,scene=scene))
        for x,y in [start[:2],goal]:
            for z in zones:
                assert math.hypot(x-z['x'],y-z['y'])>z['radius']+math.hypot(.43,.36)+.1,'Prior-zone footprint conflict'
        arms=['no_turn','turn'] if (repeat==1)==('ramp' in scene) else ['turn','no_turn']
        for arm in arms:trials.append(dict(scene=scene,folder=f'{scene}-repeat{repeat}-{arm}',pair=f'{scene}-repeat{repeat}',repeat=repeat,arm=arm,start=start,goal=goal,seconds=35,observation_yaw=0.,observation_delta=delta))
files=['scripts/swept_resampling.py','scripts/arrival_resampling.py','scripts/resampling_dataset.py','scripts/trajectory_evaluation.py','scripts/capture_slope_resampling.py','scripts/observation_path.py','scripts/experiment_motion.py','scripts/simulation_experiment.py','scripts/autonomous_survey.py','ros2_ws/src/mooncar_terrain/scripts/terrain_map.py','ros2_ws/src/mooncar_terrain/scripts/terrain_geometry.py','ros2_ws/src/mooncar_terrain/config/zones.yaml','ros2_ws/src/mooncar_terrain/tests/test_swept_resampling.py']
plan=dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),trials=trials,gate_version='v3',arrival_wait=5.,source_sha256={p:hashlib.sha256((Path('/source')/p).read_bytes()).hexdigest() for p in files},admission=dict(min_precision=.5,min_recall=.5,no_false_skip=True,max_nonpositive_turn_pairs=1,both_residual_branches=True,all_eight_successful=True),limits=['New starts/goals; ramp +0.02rad and curve +0.08rad relative to measured decision heading, each arm twice. Counterbalanced.','35s budget, 5s baseline, 5s common settling to probe repeat-view saturation. Both arms resolve the same relative-angle rule before optional turn; both execute regardless of model.','V3 removes heading-tolerance veto only; support/ray model and thresholds remain V2. Skip means no modelled extra support relative to hold, never no recovery or completed exploration.','Hold/turn captured separately; primary shared decision-deficient cohort, initial-deficient subset separate; raw estimate jitter and executed angles reported.','Admission: same predeclared precision/recall >=0.5, no paired false skip, <=1 nonpositive recommended-turn pair, both residual branches and all 8 captures. For positive turns require signed actual delta >= half requested and error <=0.02+0.25*requested; failures retained.','Zero-angle synthetic controls are diagnostic only, not evidence of a real low-benefit turn. No post-outcome threshold changes or automatic deployment.'])
(root/'plan.json').write_text(json.dumps(plan,indent=2)+'\n');(root/'source').mkdir()
for name in files:shutil.copyfile(Path('/source')/name,root/'source'/Path(name).name)
shutil.copyfile(__file__,root/'runner.py')
for scene in ('visible_ramp_v1','visible_curve_v1'):survey_world(scene).write(root/(scene+'.sdf'))
try:
    for trial in trials:
        run(root/trial['folder'],conditions=['approach'],goal=trial['goal'],seconds=35,start=trial['start'],scene=trial['scene'],nav2_plan=True,observation_yaw=trial['observation_yaw'],observe_turn=trial['arm']=='turn',arrival_wait=5.,observation_delta=trial['observation_delta'])
        wait_for(lambda s:s['phase']=='stopped',90)
finally:request('stop',{})
(root/'finished.json').write_text(json.dumps(dict(finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())))
print('ALL SMALL-ANGLE VALIDATION CAPTURES FINISHED',flush=True)
