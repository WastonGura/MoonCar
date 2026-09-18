from pathlib import Path
import csv,json,hashlib
base=Path('/source/reports/evidence/terrain-workbench/swept-support-20260913/analysis');new=Path('/evidence/small-angle-development-20260913')
def rows(path):
    with path.open(encoding='utf-8-sig',newline='') as f:return {(r['folder'],r['ix'],r['iy']):r for r in csv.DictReader(f)}
a,b=rows(base/'cells.csv'),rows(new/'cells.csv');assert a==b
old=json.loads((base/'summary.json').read_text());current=json.loads((new/'summary.json').read_text())
changes=[]
for t in current['trials']:
    prior=next(p for p in old['trials'] if p['folder']==t['folder'])
    if prior['gate']['turn']!=t['gate']['turn']:changes.append(dict(folder=t['folder'],before=prior['gate']['reason'],after=t['gate']['reason']))
result=dict(identical_cell_rows=len(a),changed_decisions=changes,development_only=True,inputs={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (base/'cells.csv',new/'cells.csv')})
(new/'gate-only-change-check.json').write_text(json.dumps(result,indent=2));print(result)
