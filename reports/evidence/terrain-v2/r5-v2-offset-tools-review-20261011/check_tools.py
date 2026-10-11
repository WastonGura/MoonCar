"""Read-only delivery review plus isolated CLI probes; never calls Docker/Gazebo."""
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
PREP=ROOT.parent/'r5-v2-offset-diagnosis-prep-20261011'
FIXTURES=ROOT.parent/'r5-v2-offset-prep-codex-review-20261011'
BASE=ROOT.parent/'r5-turn-gate-validation-20261011/campaign/14-pure_turn-ramp-1000us-left'


def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def main():
    evaluator=module(PREP/'evaluate_diagnosis_run.py','gemini_updated_evaluator')
    runner=module(PREP/'run_diagnosis_campaign.py','gemini_updated_runner')
    result={'scope':'offline review only; no new simulation or container',
            'synthetic_contact_probes':{},
            'runner_dry_run':runner.dry_run_check()}
    for mode in ['complete','missing','explicit_empty','unknown']:
        r=evaluator.evaluate_run(FIXTURES/('synthetic-'+mode))
        result['synthetic_contact_probes'][mode]={k:r.get(k) for k in ['qualified','qualification_error','fr_zero_contact_steps_count']}
    code=(PREP/'run_diagnosis_campaign.py').read_text()
    tree=ast.parse(code)
    result['runner_ast_function_calls']=sorted({ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n,ast.Call)})
    # No runner execution with authorization flags: it has no subprocess calls.
    # The dry-run function only computes metadata/strings; no shell evaluation.
    original=(BASE/'world.sdf').read_text()
    result['world_diff_checks']={}
    for filename,name,x in [('world_shift_x05.sdf','test_shift_x_plus_5cm',.05),('world_restore_x00.sdf','restore_x_0',0.)]:
        text=(PREP/'worlds'/filename).read_text()
        normalized=text.replace(name,'14-pure_turn-ramp-1000us-left')
        if x:normalized=normalized.replace('<pose>0.05 0 .03 0 0 0</pose>','<pose>0 0 .03 0 0 0</pose>')
        result['world_diff_checks'][filename]={'only_declared_changes':normalized==original,
            'sha256':hashlib.sha256((PREP/'worlds'/filename).read_bytes()).hexdigest()}
    manifest=json.loads((PREP/'manifest.json').read_text())
    result['manifest_verified']=all(hashlib.sha256((PREP/k).read_bytes()).hexdigest()==v['sha256'] for k,v in manifest.items())
    # Probe overwrite only against a newly-created disposable marker in this review.
    marker=ROOT/'overwrite-probe.json';marker.write_text('{"preserve_this_marker": true}\n')
    probe=subprocess.run([sys.executable,'-B',str(PREP/'evaluate_diagnosis_run.py'),
                          '--run-dir',str(FIXTURES/'synthetic-complete'),'--output',str(marker)],
                         capture_output=True,text=True,timeout=30)
    result['cli_overwrite_probe']={'exit_code':probe.returncode,
        'existing_marker_overwritten':'preserve_this_marker' not in marker.read_text(),
        'stdout':probe.stdout,'stderr':probe.stderr}
    baseline=evaluator.evaluate_run(BASE)
    result['baseline_evaluator']={k:baseline.get(k) for k in ['qualified','final_displacement_mm','critical_events_count','net_horizontal_impulse_Ns','actual_com_dv_mm_s']}
    result['baseline_saved_event_iterations']=[r['iteration'] for r in baseline.get('sample_critical_events',[])]
    result['baseline_target_17019_saved']=17019 in result['baseline_saved_event_iterations']
    result['baseline_has_grid_edge_windows']=any('edge' in k for k in baseline)
    result['executed_files_sha256']={n:hashlib.sha256((PREP/n).read_bytes()).hexdigest() for n in ['run_diagnosis_campaign.py','evaluate_diagnosis_run.py','campaign_plan.json']}
    (ROOT/'review-results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2,allow_nan=False))


if __name__=='__main__':main()
