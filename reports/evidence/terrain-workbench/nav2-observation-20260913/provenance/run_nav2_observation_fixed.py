import sys,json,hashlib,datetime,shutil
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from simulation_experiment import survey_world
from capture_slope_resampling import run
from compare_survey_strategies import request,wait_for
root=Path('/evidence/nav2-observation-fixed-20260913');root.mkdir(exist_ok=False)
trials=[dict(scene=scene,folder=f'{scene}-repeat{repeat}',start=start,goal=goal,seconds=35,repeat=repeat,observation_yaw=0.) for repeat in (1,2) for scene,start,goal in [('visible_ramp_v1',[0,-.6,.463648],[.8,-.2]),('visible_curve_v1',[0,.6,-.463648],[.8,.2])]]
files=['scripts/observation_path.py','scripts/resampling_dataset.py','scripts/capture_slope_resampling.py','scripts/experiment_motion.py','scripts/simulation_experiment.py','scripts/autonomous_survey.py','ros2_ws/src/mooncar_terrain/scripts/terrain_map.py']
plan=dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),trials=trials,conditions=['approach'],source_sha256={p:hashlib.sha256((Path('/source')/p).read_bytes()).hexdigest() for p in files},limits=['Two repeated trajectories per independent world, no training or strategy efficacy claim','ComputePathToPose snapshot saved before NavigateToPose; navigation may replan','Intermediate headings assume path tangents; final observation heading explicitly executed with Nav2 Spin','Goal yaw tolerance .1 radians, goal position diagnostic; no relaxation after outcomes','True poses/world geometry evaluator-only; no future or truth features'])
(root/'plan.json').write_text(json.dumps(plan,indent=2)+'\n');(root/'source').mkdir()
for name in files:shutil.copyfile(Path('/source')/name,root/'source'/Path(name).name)
for scene in ('visible_ramp_v1','visible_curve_v1'):survey_world(scene).write(root/(scene+'.sdf'))
try:
    for trial in trials:
        run(root/trial['folder'],conditions=['approach'],goal=trial['goal'],seconds=trial['seconds'],start=trial['start'],scene=trial['scene'],nav2_plan=True,observation_yaw=trial['observation_yaw'])
        wait_for(lambda s:s['phase']=='stopped',90)
finally:request('stop',{})
print('ALL NAV2 OBSERVATION CAPTURES FINISHED',flush=True)

