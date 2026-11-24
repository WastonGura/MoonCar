import sys,json,hashlib,datetime,shutil,math,yaml
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from simulation_experiment import survey_world,validate_task
from capture_slope_resampling import run
from compare_survey_strategies import request,wait_for
root=Path('/evidence/swept-validation-20260913');root.mkdir(exist_ok=False)
zones=yaml.safe_load(Path('/source/ros2_ws/src/mooncar_terrain/config/zones.yaml').read_text())['zones']
trials=[]
for repeat in (1,2):
    for scene,start,goal,yaw in [('visible_ramp_v1',[-1.3,1.3,-.4],[-.6,1.05],.25),('visible_curve_v1',[-1.8,2.,-.25],[-1.05,1.65],-.8)]:
        validate_task(dict(version=1,region=[-25,-25,25,25],start=start,scene=scene))
        for x,y in [start[:2],goal]:
            for z in zones:
                assert math.hypot(x-z['x'],y-z['y'])>z['radius']+math.hypot(.43,.36)+.1,'Prior-zone footprint conflict'
        arms=['no_turn','turn'] if (repeat==1)==('ramp' in scene) else ['turn','no_turn']
        for arm in arms:trials.append(dict(scene=scene,folder=f'{scene}-repeat{repeat}-{arm}',pair=f'{scene}-repeat{repeat}',repeat=repeat,arm=arm,start=start,goal=goal,seconds=35,observation_yaw=yaw))
files=['scripts/swept_resampling.py','scripts/arrival_resampling.py','scripts/resampling_dataset.py','scripts/trajectory_evaluation.py','scripts/capture_slope_resampling.py','scripts/observation_path.py','scripts/experiment_motion.py','scripts/simulation_experiment.py','scripts/autonomous_survey.py','ros2_ws/src/mooncar_terrain/scripts/terrain_map.py','ros2_ws/src/mooncar_terrain/scripts/terrain_geometry.py','ros2_ws/src/mooncar_terrain/config/zones.yaml','ros2_ws/src/mooncar_terrain/tests/test_swept_resampling.py']
plan=dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),trials=trials,source_sha256={p:hashlib.sha256((Path('/source')/p).read_bytes()).hexdigest() for p in files},admission=dict(min_precision=.5,min_recall=.5,no_false_skip=True,max_nonpositive_turn_pairs=1,both_residual_branches=True,all_eight_successful=True),limits=['Independent changed positions and target headings; 2 scenes x 2 arms x 2 repeats, counterbalanced order. Old arrival-decision batch is development only.','35 simulation seconds including baseline 5s and common arrival wait 1s; capture/drop timing retained. Both arms execute regardless of shadow suggestion.','Primary cohort: ALL observed deficient cells at decision, including new transit cells. Also report original baseline-deficient stratum separately; do not mix denominators.','Cell metrics for executed turn arms: support-union precision/recall >= 0.5 and recall improves over v1; this is an exploratory admission threshold, not statistical significance.','Paired shadow decision: no false skip of exclusive recovery; at most one suggested-turn pair without positive net; both try_observation and no_modelled_marginal_support must actually occur. Alignment/budget skips do not validate residual-stop logic.','Even a pass only permits further validation, not a claim of full autonomous strategy superiority. No threshold changes after outcomes. Failures retained.'])
(root/'plan.json').write_text(json.dumps(plan,indent=2)+'\n');(root/'source').mkdir()
for name in files:shutil.copyfile(Path('/source')/name,root/'source'/Path(name).name)
shutil.copyfile(__file__,root/'runner.py')
for scene in ('visible_ramp_v1','visible_curve_v1'):survey_world(scene).write(root/(scene+'.sdf'))
try:
    for trial in trials:
        run(root/trial['folder'],conditions=['approach'],goal=trial['goal'],seconds=35,start=trial['start'],scene=trial['scene'],nav2_plan=True,observation_yaw=trial['observation_yaw'],observe_turn=trial['arm']=='turn',arrival_wait=1.)
        wait_for(lambda s:s['phase']=='stopped',90)
finally:request('stop',{})
(root/'finished.json').write_text(json.dumps(dict(finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())))
print('ALL SWEPT VALIDATION CAPTURES FINISHED',flush=True)
