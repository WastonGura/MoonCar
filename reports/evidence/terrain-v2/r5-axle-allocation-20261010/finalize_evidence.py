"""Read-only evidence hashing and baseline preservation, after physical runs."""
import hashlib,json,shutil,sys
from pathlib import Path
root=Path(sys.argv[1]) if len(sys.argv)>1 else Path('/source')
p=Path(sys.argv[2]) if len(sys.argv)>2 else Path('/evidence')
def sha(x):return hashlib.sha256(x.read_bytes()).hexdigest()
freeze=json.loads((p/'campaign/freeze.json').read_text())
dest=p/'source-snapshot';dest.mkdir(exist_ok=False)
for name,value in freeze['source'].items():
    source=root/name
    if sha(source)!=value:raise RuntimeError('Frozen source changed '+name)
    target=dest/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
for folder in ('r5_axle_allocation','r5_slip_observer'):
    for source in (root/'containers'/folder).iterdir():
        if source.is_file():
            target=dest/'containers'/folder/source.name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
before=json.loads((p/'preservation-before.json').read_text(encoding='utf-8-sig'))
changed=[n for n,h in before.items() if not (root/n).is_file() or sha(root/n)!=h]
(p/'preservation.json').write_text(json.dumps(dict(checked_files=len(before),changed=changed,passed=not changed),indent=2)+'\n')
files={str(x.relative_to(p)):dict(sha256=sha(x),bytes=x.stat().st_size) for x in sorted(p.rglob('*')) if x.is_file() and x.name not in ('archive-manifest.json','archive-verification.json')}
manifest=dict(files=files,total_bytes=sum(x['bytes'] for x in files.values()),entry_count=len(files))
(p/'archive-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
bad=[n for n,h in files.items() if sha(p/n)!=h['sha256']]
(p/'archive-verification.json').write_text(json.dumps(dict(checked=len(files),mismatches=bad,passed=not bad),indent=2)+'\n')
print(json.dumps(dict(preservation_passed=not changed,entries=len(files),bytes=manifest['total_bytes'],archive_verified=not bad)))
if changed or bad:raise SystemExit(1)
