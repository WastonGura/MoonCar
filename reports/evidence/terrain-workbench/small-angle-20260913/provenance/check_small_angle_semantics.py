import sys
from pathlib import Path
sys.path.insert(0,'/source/scripts')
import numpy as np
from swept_resampling import predict
cloud=np.array([[2.02,.01,0],[2.08,.01,0],[2.14,.01,0]])
robot=np.eye(4);camera=np.eye(4);camera[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];camera[:3,3]=[.435,0,.235]
quality=dict(resolution=.25,cells=[dict(ix=8,iy=0,fit_state='too_few_voxels')])
info=dict(width=320,height=240,k=[277.191,0,160,0,277.191,120,0,0,1])
rows,gate=predict(quality,cloud,camera,robot,0.,info,.5,20,gate_version='v3')
assert rows[0]['hold_prediction'] and rows[0]['swept_prediction'] and not rows[0]['marginal_prediction']
assert not gate['turn'] and gate['reason']=='no_modelled_marginal_support'
print('PASS: same-view marginal stop does not assert zero recovery while holding')
