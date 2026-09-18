"""Development-only angle scan of an already saved pre-action sensor prefix."""
import sys,json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,'/source/scripts')
from resampling_dataset import TerrainMap
from swept_resampling import predict
from survey_evaluation import write_json
capture=Path(sys.argv[1]);output=Path(sys.argv[2]);meta=json.loads((capture/'checkpoint.json').read_text())
terrain=TerrainMap()
with np.load(capture/'checkpoint.npz',allow_pickle=False) as data:
    for i,stamp in enumerate(data['stamp']):
        camera=data['estimated_camera'][i]
        terrain.add_frame(data['points'][data['offsets'][i]:data['offsets'][i+1]],float(stamp),camera[:3,3],Rotation.from_matrix(camera[:3,:3]).as_quat())
    robot=data['estimated_robot'][-1]
quality=terrain.quality(True);cloud=terrain.cloud();base=math.atan2(robot[1,0],robot[0,0]);rows=[]
for delta in [sign*angle for angle in (.005,.01,.015,.025,.03,.05,.06,.1,.12,.15,.2,.3,.4,.6,.8,1.,1.5,2.) for sign in (1,-1)]:
    yaw=math.atan2(math.sin(base+delta),math.cos(base+delta))
    values,gate=predict(quality,cloud,camera,robot,yaw,meta['camera_info'],meta['pixel_offset'],30,gate_version='v3')
    rows.append(dict(delta=delta,marginal=sum(r['marginal_prediction'] for r in values),reason=gate['reason']))
write_json(output,dict(development_only=True,checkpoint_sha256=meta['sha256'],rows=rows))
print(json.dumps(rows,indent=2))
