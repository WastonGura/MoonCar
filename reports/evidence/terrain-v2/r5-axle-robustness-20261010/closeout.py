"""Preserve stopped phase status and verify the original workbench."""
import json,sys
from pathlib import Path
p=Path(sys.argv[1])
before=json.loads((p/'workbench-before.json').read_text(encoding='utf-8-sig'))[0]
after=json.loads((p/'workbench-after.json').read_text(encoding='utf-8-sig'))[0]
container=json.loads((p/'container-final.json').read_text(encoding='utf-8-sig'))[0]
campaign=json.loads((p/'campaign/summary.json').read_text())
ledger=json.loads((p/'campaign/ledger.json').read_text())
checks=json.loads((p/'checks-summary.json').read_text())
summary=dict(status=campaign['status'],attempts=len(ledger),qualified=sum(bool(x.get('qualified')) for x in ledger),
    failed=sum(not bool(x.get('qualified')) for x in ledger),remaining_matrix_not_run=campaign['limit']-len(ledger),
    stop_reason=campaign['reason'],frozen_execution_unchanged=campaign['freeze_unchanged'],
    step_comparison_passed=all(x['passed'] for x in campaign['step_comparisons']),
    production_model_and_defaults_changed=False,whole_vehicle_accepted=False,
    v2_robustness_verified=False,regressions_passed=checks['passed'],
    own_container=dict(id=container['Id'],status=container['State']['Status'],running=container['State']['Running'],
        oom_killed=container['State']['OOMKilled'],exit_code=container['State']['ExitCode']),
    original_workbench=dict(id=before['Id'],id_unchanged=before['Id']==after['Id'],
        state_unchanged=before['State']==after['State'],config_unchanged=before['Config']==after['Config']))
assert campaign['status']=='stopped' and len(ledger)==15 and not container['State']['Running']
assert all(summary['original_workbench'][k] for k in ('id_unchanged','state_unchanged','config_unchanged'))
(p/'closure.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary))
