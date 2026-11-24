import sys,json,hashlib,datetime,shutil
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from simulation_experiment import survey_world,validate_task
from capture_slope_resampling import run
from compare_survey_strategies import request,wait_for
root=Path('/evidence/heading-control-20260913');root.mkdir(exist_ok=False)
trials=[]
for scene,start,goal,arms in [('visible_ramp_v1',[0,-1.,.463648],[.8,-.6],['no_turn','turn']),('visible_curve_v1',[0,1.,-.463648],[.8,.6],['turn','no_turn'])]:
    validate_task(dict(version=1,region=[-25,-25,25,25],start=start,scene=scene))
    for arm in arms:trials.append(dict(scene=scene,folder=scene+'-'+arm,arm=arm,start=start,goal=goal,seconds=35,observation_yaw=0.))
files=['scripts/observation_path.py','scripts/resampling_dataset.py','scripts/capture_slope_resampling.py','scripts/path_resampling_evaluation.py','scripts/experiment_motion.py','scripts/simulation_experiment.py','scripts/autonomous_survey.py','ros2_ws/src/mooncar_terrain/scripts/terrain_map.py']
plan=dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),trials=trials,conditions=['approach'],source_sha256={p:hashlib.sha256((Path('/source')/p).read_bytes()).hexdigest() for p in files},limits=['New starts, one fresh run per scene and arm; no statistical or strategy efficacy claim','35 simulation seconds per capture including 5 second baseline; no extra time for turn arm','Both arms query Nav2 before movement and share goal, navigation and sensor settings; turn arm alone executes final Spin to yaw zero','Primary paired outcome: recovered eligible cells in intersection of both initial baselines; pure target cells separately, exact initial state/point match diagnostic','Also record each full baseline, final valid area, missing labels, real capture windows, navigation/turn failures and time costs','No parameter fitting or threshold changes after outcomes; no-turn stays at arrival heading','Ground-truth geometry and poses evaluator-only, not features or movement control'])
(root/'plan.json').write_text(json.dumps(plan,indent=2)+'\n');(root/'source').mkdir()
for name in files:shutil.copyfile(Path('/source')/name,root/'source'/Path(name).name)
for scene in ('visible_ramp_v1','visible_curve_v1'):survey_world(scene).write(root/(scene+'.sdf'))
try:
    for trial in trials:
        run(root/trial['folder'],conditions=['approach'],goal=trial['goal'],seconds=trial['seconds'],start=trial['start'],scene=trial['scene'],nav2_plan=True,observation_yaw=trial['observation_yaw'],observe_turn=trial['arm']=='turn')
        wait_for(lambda s:s['phase']=='stopped',90)
finally:request('stop',{})
(root/'finished.json').write_text(json.dumps(dict(finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())))
print('ALL HEADING CONTROL CAPTURES FINISHED',flush=True)
