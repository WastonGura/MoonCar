"""Candidate features cannot read post-action data; same-bearing approach is explicit."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
import tempfile
import numpy as np
from resampling_dataset import features,extract


def test_features():
    cell=dict(ix=0,iy=0,points=3,fit_state='too_few_voxels',bearing_mask=255)
    # Cell centre (.5,.5), same bearing; halve range to expose distance-only opportunity.
    value=features(cell,1.,[-3.5,.5],[-1.5,.5],0.)
    assert value['point_count_proxy']==12 and not value['direction_prediction']
    assert value['distance_direction_prediction'] and value['bearing_change_deg']==0
    assert not features(cell,1.,[-3.5,.5],[-3.5,.5],0.)['distance_direction_prediction']
    assert not features(cell,1.,[-3.5,.5],[-1.5,.5],3.141592653589793)['distance_direction_prediction']
    for state in ('valid','steep_fit','nonplanar'):
        cell['fit_state']=state
        assert not features(cell,1.,[-3.5,.5],[-1.5,.5],0.)['distance_direction_prediction']


def test_no_future_leakage():
    before=np.array([[1,0,0],[1.06,0,0],[1.12,0,0]])
    control=dict(status='completed',condition='stationary',experiment='synthetic',action_started_sim_stamp=2.)
    with tempfile.TemporaryDirectory() as directory:
        results=[]
        for suffix in (before,before+[0,.06,0]):
            np.savez(Path(directory)/'samples.npz',points=np.vstack((before,suffix)),offsets=[0,3,6],
                     estimated_camera=np.repeat(np.eye(4)[None],2,axis=0),stamp=[1.,3.])
            rows,_=extract(Path(directory),control,None,'test');results.append(rows)
        assert results[0][0]['final_points']!=results[1][0]['final_points']
        excluded={'final_state','final_points','recovered'}
        assert [{k:v for k,v in row.items() if k not in excluded} for row in results[0]]==[
               {k:v for k,v in row.items() if k not in excluded} for row in results[1]]


if __name__=='__main__':
    test_features();test_no_future_leakage()
    print('PASS: same-bearing range, unchanged views, FOV/conflict exclusions, no post-action feature leakage')
