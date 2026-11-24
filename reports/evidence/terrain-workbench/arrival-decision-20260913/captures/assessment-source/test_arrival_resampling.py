from pathlib import Path
import sys,tempfile,json
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
import numpy as np
from arrival_resampling import recheck,replay,transitions
from assess_arrival_gate import paired


if __name__=='__main__':
    cell=dict(ix=8,iy=0,fit_state='too_few_voxels',points=3,bearing_mask=0)
    q=dict(resolution=.25,cells=[cell])
    assert recheck(q,[0,.3,.235],[0,0,1],0,10)['turn']
    assert not recheck(q,[0,.3,.235],[0,0,1],0,1)['turn']
    assert recheck(q,[0,.3,.235],[0,0,0],0,10)['reason']=='already_aligned'
    for state in ('valid','nonplanar','steep_fit'):
        cell['fit_state']=state
        assert recheck(q,[0,.3,.235],[0,0,1],0,10)['remaining_deficient']==0
    try:recheck(q,[0,0,float('nan')],[0,0,1],0,10)
    except ValueError:pass
    else:raise AssertionError('Nonfinite input accepted')
    x=transitions({(0,0):{'fit_state':'valid'}},{(1,0):{'fit_state':'valid'}},{(0,0),(1,0)})
    assert x['gained']==x['lost']==1 and x['net']==0
    def row(x,decision,final):return dict(ix=x,iy=0,decision=decision,final=final,baseline_surface='sloped_top')
    pair=paired([row(0,'too_few_voxels','valid'),row(1,'valid','valid'),row(2,'too_few_voxels','missing')],
                [row(0,'too_few_voxels','nonplanar'),row(1,'too_few_voxels','valid'),row(2,'too_few_voxels','valid')])
    assert pair['common_remaining']==2 and pair['missing_pairs']==1 and pair['no_turn_only']==1 and pair['net_recovered']==-1
    control=dict(status='completed',action_started_sim_stamp=3.,move_terminal=dict(phase='succeeded',sim_stamp=5.2),
                 observation_decision_sim_stamp=6.2,observe_turn_terminal=dict(sim_stamp=7.2))
    with tempfile.TemporaryDirectory() as directory:
        folder=Path(directory);(folder/'manifest.json').write_text(json.dumps(dict(seconds=35)))
        outputs=[]
        for changed in (False,True):
            clouds=[np.array([[1,0,0],[1.06,0,0],[1.12,0,0]])+([0,.06,0] if changed and i>=7 else [0,0,0]) for i in range(10)]
            np.savez(folder/'samples.npz',points=np.vstack(clouds),offsets=np.arange(11)*3,
                     estimated_camera=np.repeat(np.eye(4)[None],10,axis=0),estimated_robot=np.repeat(np.eye(4)[None],10,axis=0),stamp=np.arange(10,dtype=float))
            outputs.append(replay(folder,control,1.))
        assert outputs[0][0]['decision']==outputs[1][0]['decision']
        assert outputs[0][1]['decisions']==outputs[1][1]['decisions']
        assert outputs[0][0]['final']!=outputs[1][0]['final']
        assert outputs[0][1]['stage_last_stamps']['decision']==6.
    print('PASS: residual-only gate, budget/alignment guards, gains and losses, event cutoffs, no post-decision feature leakage')
