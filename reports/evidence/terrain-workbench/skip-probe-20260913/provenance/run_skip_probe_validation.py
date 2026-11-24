import sys,json,hashlib,datetime,shutil
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from capture_slope_resampling import run
from compare_survey_strategies import request,wait_for
from survey_evaluation import write_json
from skip_probe import DELTAS

root=Path('/evidence/skip-probe-validation-20260913');root.mkdir(exist_ok=False)
trials=[]
for repeat in (1,2):
    for scene,start,goal in [('visible_ramp_v1',[-1.4,1.55,-.3],[-.65,1.3]),('visible_curve_v1',[-1.75,2.2,-.4],[-1.,1.8])]:
        arms=['no_turn','turn'] if (repeat==1)==('ramp' in scene) else ['turn','no_turn']
        for arm in arms:trials.append(dict(scene=scene,folder=f'{scene}-repeat{repeat}-{arm}',pair=f'{scene}-repeat{repeat}',repeat=repeat,arm=arm,start=start,goal=goal,seconds=60,observation_yaw=0.))
files=['scripts/skip_probe.py','scripts/swept_resampling.py','scripts/arrival_resampling.py','scripts/resampling_dataset.py','scripts/trajectory_evaluation.py','scripts/capture_slope_resampling.py','scripts/observation_path.py','scripts/experiment_motion.py','scripts/simulation_experiment.py','scripts/autonomous_survey.py','ros2_ws/src/mooncar_terrain/scripts/terrain_map.py','ros2_ws/src/mooncar_terrain/scripts/terrain_geometry.py','ros2_ws/src/mooncar_terrain/config/zones.yaml','ros2_ws/src/mooncar_terrain/tests/test_skip_probe.py','tmp/assess_skip_probe_validation.py']
write_json(root/'plan.json',dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),trials=trials,arrival_wait=5.,gate_version='v3',deltas=DELTAS,
    source_sha256={p:hashlib.sha256((Path('/source')/p).read_bytes()).hexdigest() for p in files},
    admission=dict(min_executed_negative_probes=4,min_pairs=4,no_paired_extra_recovery=True),
    limits=['Freeze all rules before capture. First nonempty deficient-cell snapshot candidate with zero modelled marginal support; test deltas in listed order. Both arms use same selection procedure. Hold arm never rotates.',
    'Two scenes, two repeats, counterbalanced arm order; independently reset each run. 60 simulation seconds including 5 baseline and 5 arrival holding. Fixed candidate order; no retries chosen after outcomes.',
    'Sensor-only checkpoint written before selection/action. Maximum selection age 5 simulation seconds and at least 8 remaining. No candidate, stale/low-budget, failed execution remain in denominator.',
    'Admit only four actual nonzero executed skip probes with paired controls, no shared turn-only recovery. Executed direction same, >=half requested, error <=.02+.25*abs(requested). This is a conservative engineering screen, not significance.',
    'Scene repeatability reported separately: positive net in both repeats is a repeatable warning. Adaptive view choices and independent trajectories limit causal attribution.',
    'Truth never used in selection. Model, thresholds and online strategy unchanged. Never equate no modelled support with no hold recovery or completed exploration.']))
(root/'source').mkdir()
for p in files:shutil.copyfile(Path('/source')/p,root/'source'/Path(p).name)
shutil.copyfile(__file__,root/'runner.py')
try:
    for trial in trials:
        print('START '+trial['folder'],flush=True)
        try:
            run(root/trial['folder'],conditions=['approach'],goal=trial['goal'],seconds=60,start=trial['start'],scene=trial['scene'],nav2_plan=True,observe_turn=trial['arm']=='turn',arrival_wait=5.,skip_probe=True)
        except Exception as error:
            write_json(root/(trial['folder']+'-failure.json'),dict(error=str(error)))
            request('stop',{})
        wait_for(lambda s:s['phase']=='stopped',90)
finally:request('stop',{})
write_json(root/'finished.json',dict(finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
print('ALL SKIP PROBE ATTEMPTS FINISHED',flush=True)
