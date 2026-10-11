"""Archive and close this campaign only; preserve original files/workbench."""
import ast,hashlib,json,re,subprocess
from pathlib import Path
p=Path(__file__).resolve().parent;root=p.parents[3]
docker=r'C:\Users\jiker\AppData\Local\Programs\DockerDesktop\resources\bin\docker.exe'
node=r'C:\Users\jiker\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe'
name='mooncar-r5-contact-geometry-20261011-02'
def digest(f):
    h=hashlib.sha256()
    with f.open('rb') as s:
        for block in iter(lambda:s.read(1024*1024),b''):h.update(block)
    return h.hexdigest()
def save(n,v):(p/n).write_text(json.dumps(v,indent=2)+'\n')
def inspect(n):return json.loads(subprocess.check_output([docker,'inspect',n],text=True))[0]
target=inspect(name);before_target=json.loads((p/'container-start.json').read_text());assert target['Id']==before_target['Id']
if target['State']['Running']:
    stop=subprocess.run([docker,'stop','--time','5',target['Id']],capture_output=True,text=True,timeout=30)
    save('container-stop.json',dict(returncode=stop.returncode,stdout=stop.stdout,stderr=stop.stderr));assert stop.returncode==0
target=inspect(name);save('container-final.json',target)
workbench=inspect('mooncar-terrain-validation');save('workbench-after.json',workbench)
before=json.loads((p/'workbench-before.json').read_text());workbench_same={k:before[k]==workbench[k] for k in ('Id','State','Config')}
protected=json.loads((p/'preservation-before.json').read_text());changes=[n for n,h in protected.items() if digest(root/n)!=h]
allowed={'scripts/prepare_r5_contact_geometry.py','scripts/prepare_r5_contact_geometry.sh','scripts/run_r5_contact_geometry.py','scripts/r5_contact_geometry.py','ros2_ws/src/mooncar_gazebo/tests/test_contact_geometry.py'}
unexpected=[n for n in changes if n.replace('\\','/') not in allowed]
old_results=[]
for oldname in ('r5-turn-gate-validation-20261011','r5-contact-geometry-20261011'):
    old=root/'reports/evidence/terrain-v2'/oldname;manifest=json.loads((old/'archive-manifest.json').read_text())['files']
    failures=[n for n,v in manifest.items() if digest(old/n.replace('\\','/'))!=v['sha256']]
    old_results.append(dict(name=oldname,files=len(manifest),failures=failures))
frozen=json.loads((p/'campaign/freeze.json').read_text());snapshot_fail=[n for n,h in frozen['source'].items() if digest(p/'frozen-source-snapshot'/n)!=h]
log=(p/'software-regression.log').read_text();counts=[int(n) for n in re.findall(r'Ran (\d+) tests',log)]
pythonfiles=list((root/'scripts').glob('*.py'))+list((root/'ros2_ws/src').rglob('*.py'))+list((root/'containers').rglob('*.py'))
errors=[]
for f in pythonfiles:
    try:ast.parse(f.read_text(encoding='utf-8-sig'))
    except Exception as error:errors.append(dict(file=str(f.relative_to(root)),error=repr(error)))
js=list((root/'web').glob('*.js'))+list((root/'web').glob('*.mjs'));jserrors=[]
for f in js:
    result=subprocess.run([node,'--check',str(f)],capture_output=True,text=True)
    if result.returncode:jserrors.append(dict(file=str(f.relative_to(root)),error=result.stderr))
front=subprocess.run([node,'--test','web/replay_cache.test.js','web/session_order.test.mjs'],cwd=root,capture_output=True)
(p/'frontend-regression.log').write_bytes(front.stdout+front.stderr)
checks=dict(software_tests=counts,total=sum(counts),software_passed='Software regression checks passed.' in log and 'FAILED' not in log,
    firmware_passed='firmware core:' in log,frontend_passed=front.returncode==0,python_files=len(pythonfiles),python_errors=errors,js_files=len(js),js_errors=jserrors)
checks['passed']=checks['software_passed'] and checks['firmware_passed'] and checks['frontend_passed'] and not errors and not jserrors;save('checks-summary.json',checks)
summary=json.loads((p/'campaign/summary.json').read_text());ledger=json.loads((p/'campaign/ledger.json').read_text())
closure=dict(status=summary['status'],attempts=len(ledger),qualified=sum(e.get('qualified',False) for e in ledger),
    failed=sum(e.get('qualified') is False for e in ledger),not_run=8-len(ledger),freeze_unchanged=summary['freeze_unchanged'],
    own_container_running=target['State']['Running'],original_workbench_preserved=workbench_same,original_files_count=len(protected),authorized_tool_changes=changes,unexpected_file_changes=unexpected,
    old_archives=old_results,frozen_snapshot_files=len(frozen['source']),snapshot_failures=snapshot_fail,regressions_passed=checks['passed'],production_changed=False,whole_vehicle_accepted=False)
closure['passed']=not target['State']['Running'] and all(workbench_same.values()) and not unexpected and not snapshot_fail and not any(x['failures'] for x in old_results) and checks['passed'];save('closure.json',closure)
excluded={'archive-manifest.json','archive-verification.json'};files={}
for f in sorted(p.rglob('*')):
    if f.is_file() and f.name not in excluded and '__pycache__' not in f.parts:
        files[str(f.relative_to(p)).replace('\\','/')]=dict(size=f.stat().st_size,sha256=digest(f))
save('archive-manifest.json',dict(files=files,count=len(files),bytes=sum(v['size'] for v in files.values())))
fail=[n for n,v in files.items() if digest(p/n)!=v['sha256']]
save('archive-verification.json',dict(passed=not fail,files=len(files),bytes=sum(v['size'] for v in files.values()),failures=fail))
print(json.dumps(dict(closure=closure,checks=checks,archive_files=len(files),archive_failures=fail)))
if not closure['passed'] or fail:raise SystemExit(1)
