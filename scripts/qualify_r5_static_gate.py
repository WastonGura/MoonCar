"""Explain the measured static bias before granting formal campaign eligibility."""
import argparse
import hashlib
import json
from pathlib import Path


def qualify(analysis):
    rows=analysis['results'];ledger=analysis['ledger'];checks={}
    checks['complete_new_batch']=len(rows)==12 and ledger['status']=='completed' and ledger['freeze_unchanged']
    checks['cleanup_complete']=all(r['cleanup_passed'] and not r['failure'] for r in ledger['attempts'])
    checks['single_joint_mechanics']=analysis['native_bench_passed']
    checks['fine_step_axial_balance']=analysis['fine_axial_balance_passed']
    vehicles=[r for r in rows if r['specification']['kind']=='vehicle']
    checks['static_support']=len(vehicles)==4 and all(r['body_tail_speed_max_m_s']<.001 and
        all(c['empty_fraction']<.01 for c in r['contact'].values()) and
        all(j['mechanics_passed'] for j in r['joints'].values()) for r in vehicles)
    checks['native_constraint_explains_bias']=bool(vehicles) and all(
        abs(j['formula']['pre_constraint_implicit_force_mean_N']+j['measured']['transmitted_fz_mean_N'])
        <=max(1e-8,.00001*abs(j['measured']['transmitted_fz_mean_N']))
        for r in vehicles for j in r['joints'].values())
    precision=analysis.get('physics_profile',{}).get('name')=='r5_precision_v1'
    on=next((r for r in vehicles if r['specification']['drive_enabled'] and r['specification']['step']==(.00025 if precision else .001)),None)
    off=next((r for r in vehicles if not r['specification']['drive_enabled']),None)
    differences={k:abs(on['joints'][k]['measured']['q_mean_m']-off['joints'][k]['measured']['q_mean_m'])
                 for k in on['joints']} if on and off else {}
    checks['drive_ablation']=bool(differences) and off['drive_commands_absent'] and max(differences.values())<1e-6
    ordered=sorted([r for r in vehicles if r['specification']['drive_enabled']],key=lambda r:r['specification']['step'])
    repeat_differences={}
    if precision:
        fine=[r for r in vehicles if r['specification']['step']==.00025]
        repeats=[r for r in fine if r['specification']['drive_enabled']]
        checks['precision_static_maxima']=len(fine)==3 and all(
            j['formula']['axial_compression_error_max_m']<=.0003 and
            j['formula']['axial_force_balance_error_max_fraction']<=.05 for r in fine for j in r['joints'].values())
        repeat_differences={k:abs(repeats[0]['joints'][k]['measured']['q_mean_m']-repeats[1]['joints'][k]['measured']['q_mean_m']) for k in repeats[0]['joints']} if len(repeats)==2 else {}
        checks['repeat_static_positions']=bool(repeat_differences) and max(repeat_differences.values())<=.00001
        checks['bias_converges_with_step']=len(ordered)==3 and all(
            ordered[i]['joints'][k]['formula']['axial_compression_error_fraction']<ordered[-1]['joints'][k]['formula']['axial_compression_error_fraction']
            for i in (0,1) for k in ordered[0]['joints'])
    else:
        checks['bias_converges_with_step']=len(ordered)==3 and all(
            ordered[0]['joints'][k]['formula']['axial_compression_error_fraction'] <
            ordered[1]['joints'][k]['formula']['axial_compression_error_fraction'] <
            ordered[2]['joints'][k]['formula']['axial_compression_error_fraction'] for k in ordered[0]['joints'])
    return dict(physics_profile=analysis.get('physics_profile'),passed=all(checks.values()),checks=checks,drive_ablation_q_difference_m=differences,
        repeat_static_q_difference_m=repeat_differences,
        explanation='Finite-step implicit spring/damping evaluated before contact velocity correction',
        equation='F_axis = k*(q-ref) + (c*h+k*h*h)*(a_post-delta_v_constraint/h)',
        formula_values_are_computed=True,native_values_are_measured=True,
        parameter_tuning=False,physics_engine_modified=False,production_changed=False,hardware_calibrated=False)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--diagnostic',required=True);args=parser.parse_args()
    root=Path(args.diagnostic);path=root/'analysis.json';analysis=json.loads(path.read_text())
    result=qualify(analysis);result['analysis_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    result['diagnostic_freeze_sha256']=hashlib.sha256((root/'freeze.json').read_bytes()).hexdigest()
    target=root/'gate.json'
    if target.exists():raise FileExistsError('Refuse gate overwrite')
    target.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
