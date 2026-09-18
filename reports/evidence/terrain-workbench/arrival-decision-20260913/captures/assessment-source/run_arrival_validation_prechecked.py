import sys,json,hashlib,datetime,shutil,math,yaml
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from simulation_experiment import survey_world,validate_task
from capture_slope_resampling import run
from compare_survey_strategies import request,wait_for
root=Path('/evidence/arrival-validation-prechecked-20260913');root.mkdir(exist_ok=False)
zones=yaml.safe_load(Path('/source/ros2_ws/src/mooncar_terrain/config/zones.yaml').read_text())['zones']
trials=[]
for repeat in (1,2):
    for scene,start,goal in [('visible_ramp_v1',[-1.,1.2,-.15],[-.3,1.]),('visible_curve_v1',[-1.5,1.8,-.35],[-.8,1.6])]:
        validate_task(dict(version=1,region=[-25,-25,25,25],start=start,scene=scene))
        for x,y in [start[:2],goal]:
            for z in zones:
                assert math.hypot(x-z['x'],y-z['y'])>z['radius']+math.hypot(.43,.36)+.1,'Prior-zone footprint conflict'
        arms=['no_turn','turn'] if (repeat==1)==('ramp' in scene) else ['turn','no_turn']
        for arm in arms:trials.append(dict(scene=scene,folder=f'{scene}-repeat{repeat}-{arm}',pair=f'{scene}-repeat{repeat}',repeat=repeat,arm=arm,start=start,goal=goal,seconds=35,observation_yaw=0.))
files=['ros2_ws/src/mooncar_terrain/config/zones.yaml','scripts/arrival_resampling.py','scripts/observation_path.py','scripts/resampling_dataset.py','scripts/capture_slope_resampling.py','scripts/path_resampling_evaluation.py','scripts/experiment_motion.py','scripts/simulation_experiment.py','scripts/autonomous_survey.py','ros2_ws/src/mooncar_terrain/scripts/terrain_map.py']
plan=dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),trials=trials,conditions=['approach'],source_sha256={p:hashlib.sha256((Path('/source')/p).read_bytes()).hexdigest() for p in files},limits=['Prospective eight fresh captures: two farther target distances, changed start headings, two repeats per arm; no parameter changes after outcomes','35 simulation seconds including 5s baseline and 1s common settling interval after navigation; turn time counts within budget','Gate evaluated in shadow from observations no later than recorded decision time; experimental arm executed regardless of gate to reveal false skips','Primary: shared eligible decision-time cells, turn vs no-turn final recovery. Record net and paired exclusive recovery; pure target evaluator-only','Gate is not accepted online if a skipped turn arm has extra recovered common cells over its paired hold control, or recovery after decision contradicts zero-opportunity claims','Few controlled trajectories cannot establish overall strategy superiority; failures retained; no early integration or parameter tuning'])
(root/'plan.json').write_text(json.dumps(plan,indent=2)+'\n');(root/'source').mkdir()
for name in files:shutil.copyfile(Path('/source')/name,root/'source'/Path(name).name)
for scene in ('visible_ramp_v1','visible_curve_v1'):survey_world(scene).write(root/(scene+'.sdf'))
try:
    for trial in trials:
        run(root/trial['folder'],conditions=['approach'],goal=trial['goal'],seconds=35,start=trial['start'],scene=trial['scene'],nav2_plan=True,observation_yaw=0.,observe_turn=trial['arm']=='turn',arrival_wait=1.)
        wait_for(lambda s:s['phase']=='stopped',90)
finally:request('stop',{})
(root/'finished.json').write_text(json.dumps(dict(finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())))
print('ALL ARRIVAL VALIDATION CAPTURES FINISHED',flush=True)
