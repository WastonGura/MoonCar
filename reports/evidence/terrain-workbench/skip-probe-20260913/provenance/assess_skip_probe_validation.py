import sys,json,hashlib,math
from pathlib import Path
sys.path.insert(0,'/source/scripts')
import numpy as np
from survey_evaluation import write_json

def assess(captures,output):
    plan=json.loads((captures/'plan.json').read_text());summary=json.loads((output/'summary.json').read_text())
    execution=[];rejections=[];ids=set()
    for trial in plan['trials']:
        folder=captures/trial['folder'];path=folder/'approach-control.json'
        if not path.exists():rejections.append('missing_capture:'+trial['folder']);continue
        control=json.loads(path.read_text());ids.add(control['experiment'])
        selection=control.get('skip_probe',{});chosen=selection.get('selected')
        if not chosen or not selection.get('executable'):
            rejections.append('no_executable_candidate:'+trial['folder']);continue
        checkpoint=folder/'approach/checkpoint.npz'
        assert hashlib.sha256(checkpoint.read_bytes()).hexdigest()==selection['checkpoint_sha256']
        with np.load(checkpoint,allow_pickle=False) as prefix,np.load(folder/'approach/samples.npz',allow_pickle=False) as full:
            assert set(prefix.files)=={'points','offsets','stamp','estimated_camera','estimated_robot'}
            for key in prefix.files:assert np.array_equal(prefix[key],full[key][:len(prefix[key])]),key
        assert selection['feature_stamp']<=selection['completed_sim_stamp']<=control['observation_decision_sim_stamp']
        replay=next((t for t in summary['trials'] if t['folder']==trial['folder'] and t['status']=='evaluated'),None)
        if replay is None:rejections.append('incomplete_evaluation:'+trial['folder']);continue
        assert replay['gate']['reason']=='no_modelled_marginal_support'
        assert replay['last_feature_stamp']==selection['feature_stamp']
        if trial['arm']=='no_turn':
            assert 'observe_turn_terminal' not in control
            continue
        requested=control.get('observation_requested_delta');actual=control.get('observation_actual_delta')
        good=control.get('observation_reached') and requested is not None and actual is not None and abs(requested)>=.01 and actual*requested>0 and abs(actual)>=abs(requested)/2 and abs(actual-requested)<=.02+.25*abs(requested)
        age=control['observation_decision_sim_stamp']-selection['feature_stamp']
        good=bool(good and age<=5)
        execution.append(dict(folder=trial['folder'],verified=good,requested=requested,actual=actual,feature_age_seconds=age,
            turn_sim_seconds=replay['turn_sim_seconds'],selected_delta=chosen['delta']))
    pairs=[p for p in summary['pairs'] if p.get('scope')=='all_decision_deficient' and p['status']=='evaluated']
    if len(ids)!=8:rejections.append('not_eight_distinct_attempts')
    if sum(r['verified'] for r in execution)<plan['admission']['min_executed_negative_probes']:rejections.append('insufficient_actual_negative_probes')
    if len(pairs)<plan['admission']['min_pairs']:rejections.append('insufficient_matched_pairs')
    if any(p['turn_only']>0 for p in pairs):rejections.append('paired_recovery_missed_by_skip_rule')
    repeated=[]
    for scene in ('visible_ramp_v1','visible_curve_v1'):
        selected=[p for p in pairs if p['pair'].startswith(scene)]
        if len(selected)==2 and all(p['net_recovered']>0 for p in selected):repeated.append(scene)
    result=dict(stop_validation_passed=not rejections,rejections=rejections,execution=execution,pairs=pairs,
        repeated_positive_net_scenes=repeated,online_policy_changed=False,online_ready=not rejections,
        limits='Engineering admission, not statistical significance. Hold snapshot/trajectory differ. Turn-only recovery is a warning, not cellwise causal truth.')
    write_json(output/'assessment.json',result);print(json.dumps(result,indent=2),flush=True)

if __name__=='__main__':assess(Path(sys.argv[1]),Path(sys.argv[2]))
