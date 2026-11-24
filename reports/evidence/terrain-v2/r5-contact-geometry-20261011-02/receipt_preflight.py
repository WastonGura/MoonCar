"""Actual delayed library loading, no Gazebo world or ROS initialization."""
import ctypes,json,time,sys
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from run_r5_contact_geometry import receipt
from r5_library_readiness import Readiness
p=Path('/probe/receipt-preflight');p.mkdir()
loaded=[]
for lib in ('/evidence/contact-observer/libmooncar-contact-observer.so','/evidence/allocator/libmooncar-axle-allocation.so',
            '/evidence/turn-slip/libmooncar-turn-slip-control.so','/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim-wheel-slip-system.so'):
    loaded.append(ctypes.CDLL(lib))
gate=Readiness(p/'history.jsonl');first=receipt(p);assert first and not first['passed'];assert gate.observe(first) is None
time.sleep(.7)
loaded.append(ctypes.CDLL('/evidence/observer/libmooncar-origin-observer.so'))
second=receipt(p);assert second and second['passed'];assert gate.observe(second)['passed'];gate.close()
(p/'result.json').write_text(json.dumps(dict(passed=True,first_incomplete=True,second_complete=True,new_simulations=0,no_world=True,no_ros_init=True),indent=2)+'\n')
print('Live library receipt preflight passed.')
