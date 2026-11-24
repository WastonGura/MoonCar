"""No Gazebo/world/ROS participant: only delayed dlopen of real observer libraries."""
import ctypes,json,sys,time
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from run_r5_turn_slip import receipt
from receipt_readiness import Readiness

root=Path('/probe/receipt-preflight');root.mkdir(exist_ok=False)
handles=[]
for path in ('/evidence/contact-observer/libmooncar-contact-observer.so',
             '/evidence/allocator/libmooncar-axle-allocation.so',
             '/evidence/turn-slip/libmooncar-turn-slip-control.so',
             '/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim8-wheel-slip-system.so.8.15.0'):
    handles.append(ctypes.CDLL(path))
r=Readiness(root/'receipt-history.jsonl')
first=receipt(root);assert first and not first['passed'];assert r.observe(first) is None
time.sleep(.7)
handles.append(ctypes.CDLL('/evidence/observer/libmooncar-origin-observer.so'))
second=receipt(root);assert second and second['passed'];assert r.observe(second)['passed'];r.close()
proof=dict(passed=True,first_incomplete=not first['passed'],first_origin_observer_present=any('origin-observer' in k for k in first['libraries']),
           second_complete=second['passed'],second_origin_observer_present=any('origin-observer' in k for k in second['libraries']),
           simulated_startup_delay_wall_s=.7,new_gazebo_simulations=0,no_ros_init=True,no_world=True)
(root/'result.json').write_text(json.dumps(proof,indent=2)+'\n');print(json.dumps(proof))
