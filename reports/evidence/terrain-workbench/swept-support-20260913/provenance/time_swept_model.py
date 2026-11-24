import sys,json,csv,time
from pathlib import Path
import numpy as np
sys.path.insert(0,'/source/scripts')
from swept_resampling import predict
root=Path('/evidence/swept-development-scoped-20260913')
folder='visible_curve_v1-repeat2-turn'
with (root/'cells.csv').open(encoding='utf-8-sig') as f:cells=[dict(ix=int(r['ix']),iy=int(r['iy']),fit_state=r['decision_state']) for r in csv.DictReader(f) if r['folder']==folder]
capture=Path('/source/reports/evidence/terrain-workbench/arrival-decision-20260913/captures')/folder/'approach'
info=json.loads((capture/'camera-info.json').read_text())
with np.load(root/(folder+'-decision.npz'),allow_pickle=False) as data:
    start=time.perf_counter();rows,gate=predict(dict(resolution=.25,cells=cells),data['cloud'],data['camera'],data['robot'],0.,info,.5,20)
    result=dict(cells=len(cells),seconds=time.perf_counter()-start,sampled_poses=gate['sampled_poses'],notes='One local timing under running Gazebo workload; no runtime guarantee, excludes map reconstruction.')
(root/'timing.json').write_text(json.dumps(result,indent=2));print(result)
