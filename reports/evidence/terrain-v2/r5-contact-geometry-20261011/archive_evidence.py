"""Archive stopped evidence and separate the poststop source signature."""
import hashlib,json,sys
from pathlib import Path
root=Path(sys.argv[1]);p=Path(sys.argv[2])
def digest(file):return hashlib.sha256(file.read_bytes()).hexdigest()
source=['containers/r5_contact_metadata/CMakeLists.txt','containers/r5_contact_metadata/ContactMetadata.cc',
        'scripts/r5_contact_geometry.py','scripts/check_r5_contact_geometry.py','scripts/run_r5_contact_geometry.py',
        'scripts/preflight_r5_contact_geometry.py','scripts/r5_library_readiness.py','scripts/prepare_r5_contact_geometry.sh',
        'scripts/execute_r5_contact_geometry.sh','ros2_ws/src/mooncar_gazebo/tests/test_contact_geometry.py',
        'ros2_ws/src/mooncar_gazebo/tests/test_library_readiness.py']
current={n:digest(root/n) for n in source}
f=json.loads((p/'campaign/freeze.json').read_text())
proof=dict(poststop_source_sha256=current,simulated_source_sha256=f['source'],new_simulations_after_repair=0,
           old_simulation_results_not_reclassified=True,requires_new_signature_for_execution=True)
(p/'source-signatures.json').write_text(json.dumps(proof,indent=2)+'\n')
records={str(x.relative_to(p)):dict(sha256=digest(x),bytes=x.stat().st_size) for x in sorted(p.rglob('*'))
         if x.is_file() and x.name not in ('archive-manifest.json','archive-verification.json')}
total=sum(v['bytes'] for v in records.values())
(p/'archive-manifest.json').write_text(json.dumps(dict(files=records,file_count=len(records),bytes=total,
    attempts=1,qualified=0,failed=1,not_run=7,original_runtime_readonly_root='/evidence: r5-turn-gate-validation-20261011',
    raw_data_preserved_locally=True,large_raw_data_committed=False,goal_achieved=False),indent=2)+'\n')
fail=[k for k,v in records.items() if digest(p/k)!=v['sha256']]
(p/'archive-verification.json').write_text(json.dumps(dict(files=len(records),bytes=total,failures=fail,passed=not fail),indent=2)+'\n')
print(json.dumps(dict(files=len(records),bytes=total,passed=not fail)))
if fail:raise SystemExit(1)
