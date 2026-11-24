"""Preserve actual completion and independently verify original workbench."""
import json,sys
from pathlib import Path
p=Path(sys.argv[1])
def read(name):return json.loads((p/name).read_text(encoding='utf-8-sig'))
before=read('workbench-before.json')[0];after=read('workbench-after.json')[0]
container=read('container-final.json')[0];campaign=read('campaign/summary.json');ledger=read('campaign/ledger.json')
checks=read('checks-summary.json')
completed_pairs=read('completed-pairs.json')
summary=dict(status=campaign['status'],attempts=len(ledger),qualified=sum(bool(x.get('qualified')) for x in ledger),
    failed=sum(not bool(x.get('qualified')) for x in ledger),remaining_matrix_not_run=campaign['limit']-len(ledger),
    stop_reason=campaign.get('reason'),frozen_execution_unchanged=campaign['freeze_unchanged'],
    gated_side_effect_removed=campaign.get('gated_side_effect_removed',False),
    completed_pairs_passed=completed_pairs['all_completed_pairs_passed'],
    final_restore_prediction_verified=False,
    step_comparisons=campaign.get('step_comparisons'),input_pairs=campaign.get('input_pairs'),
    production_model_and_defaults_changed=False,whole_vehicle_accepted=False,hardware_calibrated=False,
    regressions_passed=checks['passed'],
    own_container=dict(id=container['Id'],status=container['State']['Status'],running=container['State']['Running'],
        oom_killed=container['State']['OOMKilled'],exit_code=container['State']['ExitCode']),
    original_workbench=dict(id=before['Id'],id_unchanged=before['Id']==after['Id'],
        state_unchanged=before['State']==after['State'],config_unchanged=before['Config']==after['Config']))
assert campaign['status']!='running' and not container['State']['Running']
assert all(summary['original_workbench'][k] for k in ('id_unchanged','state_unchanged','config_unchanged'))
(p/'closure.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary))
