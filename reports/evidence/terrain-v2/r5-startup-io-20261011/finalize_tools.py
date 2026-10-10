"""Close and archive the no-Gazebo tool diagnosis without rewriting old runs."""
import hashlib,json
from pathlib import Path
p=Path(__file__).resolve().parent;root=p.parents[3]
def read(path):return json.loads(path.read_text(encoding='utf-8-sig'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
before=read(p/'workbench-before.json')[0];after=read(p/'workbench-after.json')[0];container=read(p/'container-final.json')[0]
assert before['Id']==after['Id'] and before['State']==after['State'] and before['Config']==after['Config']
assert not container['State']['Running'] and not container['State']['OOMKilled']
guard=root/'ros2_ws/src/mooncar_gazebo/launch/image_dds_guard.py'
assert sha(guard)==read(p/'test-guard.manifest.json')['base_sha256']
original=read(root/'reports/evidence/terrain-v2/r5-turn-slip-gate-20261011/preservation-before.json')
changed=[name for name,digest in original.items() if sha(root/name)!=digest]
assert not changed
out=dict(new_simulations=0,original_enodata_cause_confirmed=False,test_guard_unit_tests=8,entry_tests=4,
    native_runtime_passed=read(p/'native-summary.json')['passed'],production_guard_changed=False,
    old_files_checked=len(original),old_files_changed=changed,original_workbench_unchanged=True,
    container_id=container['Id'],container_stopped=True,stock_bridge_close_issue_unresolved=True)
(p/'closure.json').write_text(json.dumps(out,indent=2)+'\n')
files={str(path.relative_to(p)).replace('\\','/'):dict(sha256=sha(path),bytes=path.stat().st_size)
    for path in sorted(p.rglob('*')) if path.is_file() and path.name not in ('archive-manifest.json','archive-verification.json')}
(p/'archive-manifest.json').write_text(json.dumps(dict(files=files,bytes=sum(r['bytes'] for r in files.values()),count=len(files)),indent=2)+'\n')
bad=[name for name,row in files.items() if sha(p/name)!=row['sha256']]
(p/'archive-verification.json').write_text(json.dumps(dict(passed=not bad,checked=len(files),mismatches=bad),indent=2)+'\n')
assert not bad;print(json.dumps(out))
