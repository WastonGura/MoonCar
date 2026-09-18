"""Evidence sufficiency audit, separate from the frozen exploratory admission check."""
import sys,json
from pathlib import Path

def audit(trials):
    probes=[t for t in trials if t['status']=='evaluated' and t['arm']=='turn' and t['gate']['reason']=='no_modelled_marginal_support' and t['gate']['angle_rad']>1e-6]
    unprobed=[t['folder'] for t in trials if t['status']=='evaluated' and t['arm']=='no_turn' and t['gate']['reason']=='no_modelled_marginal_support']
    return dict(executed_nonzero_skip_probes=[t['folder'] for t in probes],unprobed_skip_suggestions=unprobed,
        stop_validation_sufficient=False,online_ready=False,
        reasons=['no_executed_nonzero_skip_probe'] if not probes else ['negative_probe_outcome_review_required'],
        meaning='Passing branch counts across different arms does not test the skipped action. Same-condition paired runs can produce different decisions. Do not modify the frozen admission verdict.')

if __name__=='__main__':
    sample=dict(folder='hold-only',status='evaluated',arm='no_turn',gate=dict(reason='no_modelled_marginal_support',angle_rad=.02))
    assert audit([sample])['reasons']==['no_executed_nonzero_skip_probe']
    root=Path(sys.argv[1]);result=json.loads((root/'summary.json').read_text())
    report=audit(result['trials']);(root/'stop-coverage-audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
