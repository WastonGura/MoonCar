"""Independent accounting checks for the frozen visible-surface diagnostic."""
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np

root=Path(__file__).resolve().parent
for file in root.rglob('hashes.json'):
    for name,digest in json.loads(file.read_text()).items():
        p=file.parent/name
        assert p.resolve().is_relative_to(file.parent.resolve())
        assert hashlib.sha256(p.read_bytes()).hexdigest()==digest,str(p)
folder=root/'analysis';summary=json.loads((folder/'summary.json').read_text())
source=root.parent/'projection-20260913/fixed-motion'
for name,digest in summary['input_sha256'].items():assert hashlib.sha256((source/name).read_bytes()).hexdigest()==digest
assert not summary['accepted_accuracy'] and summary['frames']==147
assert summary['ray_matched']+summary['ray_missing']==summary['observed_points']
assert sum(summary['point_categories'].values())==summary['ray_matched']
rows=list(csv.DictReader((folder/'cells.csv').open(encoding='utf-8-sig',newline='')))
assert len({(r['ix'],r['iy']) for r in rows})==3228
for r in rows:
    paired=r['paired']=='True'
    assert paired==(r['valid']=='True' and r['reference_valid']=='True')
    assert bool(r['height_error_m'])==paired and bool(r['slope_error_deg'])==paired
    assert (r['single_surface_pair']=='True')==(paired and r['category'] in ('ground','sloped_top','level_top','curved_top'))
for group in summary['groups']:
    selected=[r for r in rows if group['category']=='all' or r['category']==group['category']]
    observed=[r for r in selected if r['observed']=='True'];pairs=[r for r in selected if r['paired']=='True']
    assert (len(selected),len(observed),len(pairs))==(group['cells'],group['observed'],group['paired'])
    assert sum(r['valid']=='False' for r in observed)==group['invalid']
    if pairs:
        error=math.sqrt(sum(float(r['height_error_m'])**2 for r in pairs)/len(pairs))
        assert math.isclose(error,group['height_rmse_m'],abs_tol=1e-12)
bins=list(csv.DictReader((folder/'quality-bins.csv').open(encoding='utf-8-sig',newline='')))
for b in bins:
    lo,hi=float(b['q_min']),float(b['q_max'])
    selected=[r for r in rows if r['quality'] and lo<=float(r['quality']) and (float(r['quality'])<hi or hi==1 and float(r['quality'])==1) and (b['category']=='all' or r['category']==b['category'])]
    assert len(selected)==int(b['observed'])
    assert sum(r['valid']=='False' for r in selected)==int(b['invalid'])
high=[r for r in rows if r['category']=='mixed' and float(r['quality'] or 0)>=.8]
assert len(high)==27 and sum(r['valid']=='False' for r in high)==16
slope=[r for r in rows if r['category']=='sloped_top']
assert len(slope)==36 and sum(r['fit_status']=='too_few_voxels' for r in slope)==28
with np.load(folder/'measured-true-pose.npz') as a,np.load(folder/'visible-reference.npz') as b:
    np.testing.assert_array_equal(a['stamps'],b['stamps']);assert len(a['stamps'])==147
    np.testing.assert_allclose(a['translations'],b['translations']);np.testing.assert_allclose(a['quaternions'],b['quaternions'])
    assert len(a['points'])==summary['observed_points'] and len(b['points'])==summary['ray_matched']
print('PASS: frozen inputs, identical rays/poses frame accounting, retained invalid cells, Q bins and mixed-surface counterexamples')
