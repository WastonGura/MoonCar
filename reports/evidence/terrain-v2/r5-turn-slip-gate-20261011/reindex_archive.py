"""Refresh archive hashes after report-only whitespace edits; never alter data."""
import hashlib,json
from pathlib import Path
p=Path(__file__).resolve().parent
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
files={str(path.relative_to(p)).replace('\\','/'):dict(sha256=sha(path),bytes=path.stat().st_size)
    for path in sorted(p.rglob('*')) if path.is_file() and path.name not in ('archive-manifest.json','archive-verification.json')}
manifest=dict(files=files,total_bytes=sum(x['bytes'] for x in files.values()),entry_count=len(files))
(p/'archive-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
bad=[name for name,row in files.items() if sha(p/name)!=row['sha256']]
(p/'archive-verification.json').write_text(json.dumps(dict(checked=len(files),mismatches=bad,passed=not bad),indent=2)+'\n')
print(json.dumps(dict(entries=len(files),bytes=manifest['total_bytes'],verified=not bad)))
if bad:raise SystemExit(1)
