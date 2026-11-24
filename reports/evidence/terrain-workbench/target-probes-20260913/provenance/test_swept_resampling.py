from pathlib import Path
import json
import math
import sys
import tempfile
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
import numpy as np
from scipy.spatial.transform import Rotation
from swept_resampling import swept_poses, predict, support, snapshot, decide


if __name__=='__main__':
    assert not decide([{'marginal_prediction':True}],.05,10,'v2')['turn']
    assert decide([{'marginal_prediction':True}],.05,10,'v3')['turn']
    assert decide([{'marginal_prediction':False}],.05,10,'v3')['reason']=='no_modelled_marginal_support'
    assert not decide([{'marginal_prediction':True}],.05,1,'v3')['turn']
    from capture_slope_resampling import run
    for delta,nav in [(float('nan'),True),(.1,False),(4.,True)]:
        try:run(Path('must-not-create'),nav2_plan=nav,observation_delta=delta)
        except ValueError:pass
        else:raise AssertionError('Invalid relative observation request accepted')
    robot=np.eye(4);robot[:3,:3]=Rotation.from_euler('z',math.radians(179)).as_matrix()
    camera=robot.copy();camera[:3,3]=robot[:3,:3]@np.array([.435,0,.235])
    poses,angle=swept_poses(camera,robot,math.radians(-179))
    assert abs(angle-math.radians(2))<1e-8 and len(poses)==2
    assert np.allclose(poses[-1][:3,3],Rotation.from_euler('z',math.radians(-179)).apply([.435,0,.235]))
    cloud=np.array([[2.02,.01,0],[2.08,.01,0],[2.14,.01,0]])
    assert support(cloud,.25,.05)==support(np.repeat(cloud,5,axis=0),.25,.05)
    assert not support(cloud,.25,.05)['can_fit']
    expanded=np.vstack([cloud,cloud+[0,.08,0]])
    assert support(expanded,.25,.05)['can_fit']
    robot=np.eye(4);camera=np.eye(4);camera[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];camera[:3,3]=[.435,0,.235]
    quality=dict(resolution=.25,cells=[dict(ix=8,iy=0,fit_state='too_few_voxels')])
    info=dict(width=320,height=240,k=[277.191,0,160,0,277.191,120,0,0,1])
    rows,gate=predict(quality,cloud,camera,robot,1.,info,.5,20)
    assert rows[0]['swept_visible'] and not rows[0]['final_visible']
    assert rows[0]['swept_points']>=rows[0]['hold_points']
    assert predict(quality,cloud,camera,robot,1.,info,.5,1)[1]['reason']=='insufficient_budget'
    assert predict(quality,cloud,camera,robot,0.,info,.5,20)[1]['reason']=='already_aligned'
    # A single horizontal proxy may fabricate points; identical measured views must not.
    zero,gate=predict(quality,cloud,camera,robot,0.,info,.5,20,gate_version='v3',model_version='v4')
    assert zero[0]['hold_points']==zero[0]['swept_points']==len(cloud)
    assert not zero[0]['marginal_prediction'] and not gate['turn']
    moved,_=predict(quality,cloud,camera,robot,1.,info,.5,20,gate_version='v3',model_version='v4')
    assert moved[0]['hold_points']==len(cloud)
    assert moved[0]['swept_points']>=moved[0]['hold_points']
    assert support(expanded,.25,.05,center=np.array([2.125,.125]))['can_fit']
    quality['cells'][0]['fit_state']='nonplanar'
    assert predict(quality,cloud,camera,robot,1.,info,.5,20)[0]==[]
    try:swept_poses(camera,robot,float('nan'))
    except ValueError:pass
    else:raise AssertionError('Accepted nonfinite yaw')
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory);results=[]
        for changed in (False,True):
            clouds=[cloud,expanded if changed else cloud]
            np.savez(path/'samples.npz',points=np.vstack(clouds),offsets=[0,len(cloud),sum(map(len,clouds))],
                stamp=[1.,2.],estimated_camera=[np.eye(4)]*2,estimated_robot=[robot]*2)
            terrain,pose,base=snapshot(path,1)
            results.append((terrain.cloud(),terrain.quality(True)))
        assert np.array_equal(results[0][0],results[1][0]) and results[0][1]==results[1][1]
    print('PASS: shortest arc, camera lever arm, swept-only visibility, support rank, duplicate saturation, guards, no future samples')
