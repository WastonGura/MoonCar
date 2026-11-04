import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/validation'))
from feedback_turn import FeedbackTurn, swept_clear


def pose(x=0.,y=0.,yaw=0.):return dict(x=x,y=y,yaw=yaw,z=0.)


def grid():
    return NS(info=NS(resolution=.1,width=80,height=80,origin=NS(position=NS(x=-4.,y=-4.))),data=np.zeros(6400,dtype=int))


def test_feedback_uses_position_and_preserves_acceleration_limits():
    c=FeedbackTurn(pose(),math.pi/2,0.)
    r=c.step(pose(x=-.1),.1)
    assert 0<r['linear']<=.02+1e-9
    assert 0<r['angular']<=.04+1e-9


def test_heading_success_alone_does_not_complete_displaced_turn():
    c=FeedbackTurn(pose(),math.pi/2,0.)
    r=c.step(pose(x=-.3,yaw=math.pi/2),.1)
    assert r['phase']=='return_to_anchor' and r['status']=='running'
    c.phase='final_turn'
    r=c.step(pose(x=-.3,yaw=math.pi/2),.2)
    assert r['status']=='failed'


def test_clock_and_displacement_faults_fail_closed():
    c=FeedbackTurn(pose(),0.,1.)
    with pytest.raises(ValueError):c.step(pose(),.9)
    with pytest.raises(ValueError):c.step(pose(),1.6)
    assert c.step(pose(x=.7),1.1)['status']=='failed'


def test_reverse_and_rotating_sweep_detect_midpath_obstacle():
    g=grid();region=[-4.,-4.,4.,4.]
    assert swept_clear(g,region,pose(),-.12,.3)[0]
    # Clear start and endpoint, obstacle intersects the middle of a longer sweep.
    g.data[40*80+54]=100
    assert swept_clear(g,region,pose(),0.,0.)[0]
    assert swept_clear(g,region,pose(x=2.4),0.,0.)[0]
    assert not swept_clear(g,region,pose(),.3,0.,horizon=8.)[0]


def test_unknown_and_boundary_block_before_motion():
    g=grid();g.data[40*80+40]=-1
    assert swept_clear(g,[-4.,-4.,4.,4.],pose(),0.,0.)[1]=='predicted_unknown_footprint'
    assert swept_clear(grid(),[-1.,-1.,1.,1.],pose(x=.5),.1,0.)[1]=='predicted_region_boundary'


def test_resource_witness_distinguishes_server_from_gui():
    from primary_runtime import is_gazebo_server
    assert is_gazebo_server(['gz sim -r -s -v 2 --render-engine ogre2 /tmp/world.sdf'])
    assert is_gazebo_server(['/usr/bin/gz','sim','--server-only','/tmp/world.sdf'])
    assert not is_gazebo_server(['gz sim -g -v 3 --gui-config /tmp/gui.config'])
    assert not is_gazebo_server(['gz','sim','--gui-only'])
