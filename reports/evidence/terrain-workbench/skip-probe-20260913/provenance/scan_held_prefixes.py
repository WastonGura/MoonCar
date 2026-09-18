"""Development only: does longer holding produce negative candidates?"""
import sys,json,math
from pathlib import Path
import numpy as np
sys.path.insert(0,'/source/scripts')
from swept_resampling import snapshot
from skip_probe import select
from survey_evaluation import write_json
folder=Path(sys.argv[1]);output=Path(sys.argv[2]);capture=folder/'approach'
control=json.loads((folder/'approach-control.json').read_text())
meta=json.loads((capture/'manifest.json').read_text());info=json.loads((capture/'camera-info.json').read_text())
arrival=control['move_terminal']['sim_stamp'];rows=[]
with np.load(capture/'samples.npz',allow_pickle=False) as data:stamps=data['stamp']
for wait in (5,10,15,20,25,30):
    if arrival+wait>stamps[-1]:continue
    cut=int(np.searchsorted(stamps,arrival+wait,side='right'));terrain,camera,robot=snapshot(capture,cut)
    selected,candidates=select(terrain.quality(True),terrain.cloud(),camera,robot,info,meta['projection_pixel_offset'],30)
    rows.append(dict(arrival_wait=wait,feature_stamp=float(stamps[cut-1]),selected=selected,candidates=candidates))
write_json(output,dict(development_only=True,source=str(folder),rows=rows))
print([(r['arrival_wait'],r['selected']['delta'] if r['selected'] else None) for r in rows])
