from pathlib import Path
import sys
sys.path.insert(0,'/source/scripts')
from observation_path import compute_path
try:compute_path(None,[1.,0.],dict(task=dict(region=[-5,-5,5,5]),motion=dict(active=True,ready=False,pose={})),0.)
except ValueError as error:assert 'stationary' in str(error)
else:raise AssertionError('Active motion must reject planning')
print('PASS: ROS planner imports and stationary precondition')
