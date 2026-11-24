"""Unknown/occupied route segments must not masquerade as executable observation paths."""
import base64
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from observation_path import check_path


def check():
    grid=dict(width=4,height=4,resolution=1.,origin=[0,0],data=base64.b64encode(bytes(16)).decode())
    path=[[.5,.5,0],[3.5,.5,0]]
    assert check_path(path,[3.5,.5],grid)==path
    # Check the segment interior, not merely free endpoints.
    for bad in (100,255):
        costs=bytearray(16);costs[1]=bad;grid['data']=base64.b64encode(costs).decode()
        try:check_path(path,[3.5,.5],grid)
        except ValueError:pass
        else:raise AssertionError('Unsafe segment accepted')
    grid['data']=base64.b64encode(bytes(16)).decode()
    for points,goal in [([[.5,.5,0],[5,.5,0]],[5,.5]),(path,[2,.5]),([[.5,.5,0],[float('nan'),.5,0]],[3.5,.5])]:
        try:check_path(points,goal,grid)
        except ValueError:pass
        else:raise AssertionError('Invalid path accepted')
    print('PASS: free route, unknown/occupied segment interiors, out-of-map, endpoint mismatch and non-finite poses')


if __name__=='__main__':
    check()
    try:import rclpy
    except ImportError:print('SKIP: ROS planner import check requires sourced ROS')
    else:
        from observation_path import compute_path
        state=dict(task=dict(region=[-5,-5,5,5]),motion=dict(active=True,ready=False,pose={}))
        try:compute_path(None,[1.,0.],state,0.)
        except ValueError as error:assert 'stationary' in str(error)
        else:raise AssertionError('Moving experiment accepted a new observation query')
        print('PASS: ROS imports and stationary precondition')
