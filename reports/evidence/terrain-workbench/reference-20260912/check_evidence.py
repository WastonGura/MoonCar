"""Validate delivered diagnostics independently of the plotting implementation."""
import csv
import hashlib
import json
import math
from pathlib import Path

root=Path(__file__).resolve().parent
runs=sorted(p for p in root.glob('autonomy-*') if p.is_dir())
assert len(runs)==3
for folder in runs:
    summary=json.loads((folder/'summary.json').read_text(encoding='utf-8'))
    assert summary['status']=='diagnostic_complete' and summary['accepted_accuracy'] is False
    assert summary['alignment_status']=='nominal_unverified'
    for filename,digest in json.loads((folder/'hashes.json').read_text()).items():
        assert hashlib.sha256((folder/filename).read_bytes()).hexdigest()==digest,(folder.name,filename)
    source=root.parent/'continuous-20260912'/folder.name
    for filename,digest in summary['input_sha256'].items():
        assert hashlib.sha256((source/filename).read_bytes()).hexdigest()==digest,(folder.name,'source',filename)
    with (folder/'cells.csv').open(encoding='utf-8-sig',newline='') as handle: cells=list(csv.DictReader(handle))
    assert len(cells)==5024 and len({(c['ix'],c['iy']) for c in cells})==5024
    assert math.isclose(len(cells)*summary['resolution']**2,summary['roi_area_m2'])
    for group in summary['groups']:
        selected=[c for c in cells if group['category']=='all' or c['category']==group['category']]
        assert len(selected)==group['reference_cells']
        assert group['missing_cells']+group['invalid_observed_cells']+group['valid_observed_cells']==len(selected)
        paired=[c for c in selected if c['height_error_m']!='']
        assert len(paired)==group['matched_cells']
        if paired:
            rmse=math.sqrt(sum((float(c['observed_z'])-float(c['z']))**2 for c in paired)/len(paired))
            assert math.isclose(rmse,group['height_rmse_m'],abs_tol=1e-12)
            mae=sum(abs(float(c['observed_slope'])-float(c['slope'])) for c in paired)/len(paired)
            assert math.isclose(mae,group['slope_mae_deg'],abs_tol=1e-12)
        else: assert group['height_rmse_m'] is None
    with (folder/'quality-bins.csv').open(encoding='utf-8-sig',newline='') as handle: bins=list(csv.DictReader(handle))
    assert sum(int(b['observed_cells']) for b in bins if b['category']=='all')==summary['groups'][0]['observed_cells']
    assert json.loads((source/'summary.json').read_text())['height_rmse_m'] is None
    for filename in ('surface-comparison','quality-error','coverage-status'):
        assert (folder/(filename+'.png')).read_bytes().startswith(b'\x89PNG')
        assert (folder/(filename+'.pdf')).read_bytes().startswith(b'%PDF')
alignment=json.loads((root/'alignment-parked.json').read_text())
assert len(alignment['samples'])==3 and alignment['fitted_correction'] is False
for row in alignment['samples']:
    a,b=row['gazebo_model']['position'],row['map_base_footprint']
    assert math.isclose(math.hypot(a['x']-b['x'],a['y']-b['y']),row['xy_error_m'])
print('PASS: three separate diagnostic packs, unchanged input hashes, fixed denominators, recomputed errors, Q bins and parked alignment evidence')
