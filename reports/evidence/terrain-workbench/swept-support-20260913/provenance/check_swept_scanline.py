import runpy
values=runpy.run_path('/source/ros2_ws/src/mooncar_terrain/tests/test_swept_resampling.py',run_name='__main__')
np=values['np'];rotate=values['Rotation'].from_euler('z',1.).as_matrix()
robot=np.eye(4);robot[:3,:3]=rotate
camera=values['camera'].copy();camera[:3,:3]=rotate@camera[:3,:3];camera[:3,3]=rotate@camera[:3,3]
quality=dict(resolution=.25,cells=[dict(ix=8,iy=0,fit_state='too_few_voxels')])
rows,gate=values['predict'](quality,values['cloud'],camera,robot,0.,values['info'],.5,20)
assert rows[0]['swept_prediction'] and not rows[0]['hold_prediction'] and gate['turn']
print('PASS: scanline outside initial view gains enough hypothesized support during rotation')
cloud=values['cloud']+[0,1.,0]
quality=dict(resolution=.25,cells=[dict(ix=8,iy=4,fit_state='too_few_voxels')])
rows,_=values['predict'](quality,cloud,values['camera'],np.eye(4),1.,values['info'],.5,20)
import math
assert math.atan2(1.125,2.125-.435)>math.pi/6
assert rows[0]['hold_visible']
print('PASS: partial-cell ray visibility survives an out-of-FOV cell centre')
