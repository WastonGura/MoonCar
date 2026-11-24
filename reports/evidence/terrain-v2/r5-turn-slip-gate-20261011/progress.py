"""Read-only compact progress, never modifies simulation evidence."""
import json
from pathlib import Path
p=Path(__file__).resolve().parent/'campaign'
s=json.loads((p/'summary.json').read_text());ledger=json.loads((p/'ledger.json').read_text())
out=dict(summary=s,completed=sum('qualified' in e for e in ledger),last=ledger[-1])
done=[e for e in ledger if 'qualified' in e]
if done:
    e=done[-1];f=p/e['name']/'result.json'
    if f.is_file():
        r=json.loads(f.read_text());q=r.get('turn_slip_probe',{})
        out['last_completed']=dict(name=e['name'],qualified=e['qualified'],error=e.get('error'),
            endpoint=q.get('endpoint_xy_m'),trajectory=q.get('trajectory'),gate=q.get('gate'),
            performance=r.get('formal',{}).get('performance'))
print(json.dumps(out,indent=2))
