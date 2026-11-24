import sys,json,hashlib
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from survey_evaluation import write_json

def assess(captures,analysis):
    plan=json.loads((captures/'plan.json').read_text()); result=json.loads((analysis/'summary.json').read_text())
    reasons=[];rules=plan['admission']
    for name,digest in plan['source_sha256'].items():
        assert hashlib.sha256((captures/'source'/Path(name).name).read_bytes()).hexdigest()==digest
    assert hashlib.sha256((analysis/'swept_resampling.py').read_bytes()).hexdigest()==plan['source_sha256']['scripts/swept_resampling.py']
    trials=[t for t in result['trials'] if t['status']=='evaluated']
    if len(trials)!=8:reasons.append('incomplete_successful_execution')
    for t in trials:
        assert t['last_feature_stamp']<=t['decision_stamp']
    selected=[g for g in result['metrics'] if g['scope']=='all_decision_deficient' and g['arm']=='turn']
    model=next(g for g in selected if g['rule']=='swept_prediction');old=next(g for g in selected if g['rule']=='v1_prediction')
    if model['precision'] is None or model['precision']<rules['min_precision']:reasons.append('insufficient_precision')
    if model['recall'] is None or model['recall']<rules['min_recall']:reasons.append('insufficient_recall')
    if model['recall'] is None or old['recall'] is None or model['recall']<=old['recall']:reasons.append('recall_did_not_improve')
    pairs=[p for p in result['pairs'] if p.get('scope')=='all_decision_deficient' and p['status']=='evaluated']
    if any(p['false_skip'] for p in pairs):reasons.append('false_skip_of_paired_recovery')
    if sum(p['no_positive_net'] for p in pairs)>rules['max_nonpositive_turn_pairs']:reasons.append('too_many_nonpositive_turn_pairs')
    branch={reason:sum(t['gate']['reason']==reason for t in trials) for reason in ('try_observation','no_modelled_marginal_support')}
    if not all(branch.values()):reasons.append('residual_stop_and_turn_not_both_validated')
    write_json(analysis/'assessment.json',dict(shadow_check_passed=not reasons,rejections=reasons,branches=branch,
        model_metrics=model,old_metrics=old,pairs=pairs,online_policy_changed=False,
        meaning='Predeclared exploratory admission check, not statistical significance or overall autonomous strategy acceptance'))
    print(json.dumps(dict(rejections=reasons,branches=branch,model_metrics=model,pairs=pairs),indent=2))

if __name__=='__main__':assess(Path(sys.argv[1]),Path(sys.argv[2]))
