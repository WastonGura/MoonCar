"""Host-only closeout: only this container; old failures remain unchanged."""
import ast,hashlib,json,re,subprocess,sys
from pathlib import Path
root=Path(sys.argv[1]);p=Path(sys.argv[2]);docker=sys.argv[3];node=sys.argv[4]
def digest(file):return hashlib.sha256(file.read_bytes()).hexdigest()
def inspect(name):return json.loads(subprocess.check_output([docker,'inspect',name],text=True))[0]
def save(name,value):(p/name).write_text(json.dumps(value,indent=2)+'\n')
summary=json.loads((p/'campaign/summary.json').read_text());ledger=json.loads((p/'campaign/ledger.json').read_text())
assert summary['status']=='stopped' and len(ledger)==1
target=inspect('mooncar-r5-contact-geometry-20261011')
assert target['Id']=='a659f7d9938d64f83450e1cbae6f2fc9f6464ccbf9b1255d55089fad3a92923e'
if target['State']['Running']:
    completed=subprocess.run([docker,'stop','--time','5',target['Id']],capture_output=True,text=True,timeout=30)
    save('container-stop.json',dict(returncode=completed.returncode,stdout=completed.stdout,stderr=completed.stderr))
    assert completed.returncode==0
target=inspect(target['Id']);save('container-final.json',target)
workbench=inspect('mooncar-terrain-validation');save('workbench-after.json',workbench)
before=json.loads((p/'workbench-before.json').read_text(encoding='utf-8-sig'))[0]
same={key:workbench[key]==before[key] for key in ('Id','State','Config')}
old=root/'reports/evidence/terrain-v2/r5-turn-gate-validation-20261011'
protected=json.loads((old/'preservation-before.json').read_text())
original_changes=[k for k,v in protected.items() if digest(root/k)!=v]
oldrecords=json.loads((old/'archive-manifest.json').read_text())['files']
oldfail=[k for k,v in oldrecords.items() if digest(old/Path(k.replace('\\','/')))!=v['sha256']]
frozen=json.loads((p/'campaign/freeze.json').read_text())
snapshot_failures=[k for k,v in frozen['source'].items() if digest(p/'frozen-source-snapshot'/k)!=v]
log=(p/'software-regression.log').read_text();counts=[int(n) for n in re.findall(r'Ran (\d+) tests',log)]
py=sorted((root/'scripts').glob('*.py'))+sorted((root/'ros2_ws/src').rglob('*.py'))+sorted((root/'containers').rglob('*.py'))
astfail=[]
for file in py:
    try:ast.parse(file.read_text(encoding='utf-8-sig'))
    except Exception as error:astfail.append(dict(file=str(file.relative_to(root)),error=repr(error)))
js=sorted((root/'web').glob('*.js'))+sorted((root/'web').glob('*.mjs'));jsfail=[]
for file in js:
    r=subprocess.run([node,'--check',str(file)],capture_output=True,text=True)
    if r.returncode:jsfail.append(dict(file=str(file.relative_to(root)),error=r.stderr))
front=subprocess.run([node,'--test','web/replay_cache.test.js','web/session_order.test.mjs'],cwd=root,capture_output=True)
(p/'frontend-regression.log').write_bytes(front.stdout+front.stderr)
checks=dict(package_tests=counts,total=sum(counts),software_passed='Software regression checks passed.' in log and 'FAILED' not in log,
            firmware_strict_c11_ubsan='firmware core:' in log,python_files=len(py),python_ast_failures=astfail,
            frontend_tests_passed=front.returncode==0 and b'pass 3' in front.stdout,frontend_syntax_files=len(js),frontend_syntax_failures=jsfail)
checks['passed']=checks['software_passed'] and checks['firmware_strict_c11_ubsan'] and not astfail and not jsfail and checks['frontend_tests_passed']
save('checks-summary.json',checks)
r=json.loads((p/'campaign/01-plate-left/result.json').read_text())
d=json.loads((p/'campaign/01-plate-left/diagnostic-result.json').read_text())
receipt=json.loads((p/'campaign/01-plate-left/actual-diagnostic-libraries.json').read_text())
metrics=dict(actual_attempts=1,qualified=0,failed=1,remaining_not_run=7,stop_reason=summary['reason'],
             physics_source_unchanged_at_stop=summary['freeze_unchanged'],original_round_passed=r['passed'],
             capture_input_and_primitive_gates=d['eligible'],maximum_displacement_m=d['maximum_displacement_m'],
             incomplete_receipt_passed=receipt['passed'],observer_in_first_receipt=any('origin-observer' in x for x in receipt['libraries']),
             later_step_rows=r['step_observation'],metadata=d['metadata'],run_wall_s=r['wall_seconds'],
             complete_attempt_wall_s=ledger[0]['wall_seconds'],performance=r['formal']['performance'],
             simulation_signature_not_reused_after_repair=True,receipt_fix_physics_verified=False,
             geometry_effect_confirmed=False,repair_verified=False,whole_vehicle_accepted=False,goal_achieved=False,production_changed=False)
save('round-conclusion.json',metrics)
closure=dict(status='stopped',attempts=1,qualified=0,failed=1,not_run=7,own_container_running=target['State']['Running'],
    original_workbench_preserved=same,original_files_count=len(protected),original_file_changes=original_changes,
    old_archive_files=len(oldrecords),old_archive_failures=oldfail,frozen_source_files=len(frozen['source']),frozen_snapshot_failures=snapshot_failures,
    poststop_software_repair=True,new_simulations_after_repair=0,regressions_passed=checks['passed'],goal_achieved=False)
closure['passed']=not target['State']['Running'] and all(same.values()) and not original_changes and not oldfail and not snapshot_failures and checks['passed']
save('closure.json',closure);print(json.dumps(dict(closure=closure,checks=checks)))
if not closure['passed']:raise SystemExit(1)
