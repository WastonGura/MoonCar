"""Controlled action-value experiments; never replaces the autonomous policy."""
import json
import math
from pathlib import Path
import numpy as np
from skip_probe import request_prefix
from observation_path import compute_path
from compare_survey_strategies import request
from survey_evaluation import write_json, gain

ARMS=('hold','small_turn','wide_turn','advance')
WINDOW=20.


def validate_action(action):
    if not isinstance(action,dict) or set(action)!={'arm','goal'} or action['arm'] not in ARMS:
        raise ValueError('Known comparison arm and one next XY goal required')
    if not isinstance(action['goal'],(list,tuple)) or len(action['goal'])!=2 or not all(type(v) in (int,float) and math.isfinite(v) for v in action['goal']):
        raise ValueError('Finite next XY goal required')


def prepare(capture,node,action,seconds):
    validate_action(action)
    _,_,_,meta=request_prefix(capture,node,'checkpoint')
    # Query the same alternative in every arm, so query overhead is shared.
    state=request();base=state['motion']['pose']['yaw']
    path=compute_path(node,action['goal'],state,base)
    write_json(capture/'alternative-nav2-plan.json',path)
    state=request();pose=state['motion']['pose'];now=state['motion']['diagnostics']['sim_stamp']
    if not 0<=now-meta['last_stamp']<=5 or meta['first_stamp']+seconds-now<WINDOW+2:
        raise ValueError('Stale comparison checkpoint or insufficient equal observation budget')
    delta={'hold':0.,'small_turn':.02,'wide_turn':.4,'advance':0.}[action['arm']]
    yaw=math.atan2(math.sin(pose['yaw']+delta),math.cos(pose['yaw']+delta))
    value=dict(**action,pose=pose,yaw=yaw,requested_delta=delta,feature_stamp=meta['last_stamp'],
               decision_stamp=now,window_seconds=WINDOW,checkpoint_sha256=meta['sha256'],
               path_file='alternative-nav2-plan.json')
    write_json(capture/'comparison.json',value)
    return value


def outcome(before,after,targets,origin,radius=8.):
    """Reuse the online metric definition; area is projected coverage, not accuracy."""
    values=gain(before,after,origin,radius)
    target=gain(before,after,origin,radius,targets)
    area=before['resolution']**2
    old={(c['ix'],c['iy']) for c in before['cells'] if c['fit_state']=='valid' and math.hypot((c['ix']+.5)*before['resolution']-origin[0],(c['iy']+.5)*before['resolution']-origin[1])<=radius}
    final={(c['ix'],c['iy']) for c in after['cells'] if c['fit_state']=='valid'}
    values.update(target_recovered_cells=target['resampling_recovered_cells'],
                  outside_target_recovered_cells=values['resampling_recovered_cells']-target['resampling_recovered_cells'],
                  lost_valid_surface_m2=len(old-final)*area)
    values['utility_m2']=values['new_observed_m2']+values['recovered_surface_m2']
    values['utility_m2_per_sim_second']=values['utility_m2']/WINDOW
    return values


if __name__=='__main__':
    # A recovered old cell and a newly seen valid cell must never count twice.
    def q(cells):return dict(resolution=.25,cells=[dict(ix=x,iy=0,quality=.5,fit_state=s) for x,s in cells])
    result=outcome(q([(0,'too_few_voxels'),(2,'valid')]),q([(0,'valid'),(1,'valid'),(2,'too_few_voxels')]),{(0,0)},[0,0])
    assert result['utility_m2']==.125 and result['target_recovered_cells']==1
    assert result['outside_target_recovered_cells']==0 and result['lost_valid_surface_m2']==.0625
    assert result['utility_m2_per_sim_second']==.00625
    for bad in ({'arm':'bad','goal':[0,0]},{'arm':'hold','goal':[float('nan'),0]},{'arm':'hold','goal':[True,0]}):
        try:validate_action(bad)
        except ValueError:pass
        else:raise AssertionError('Invalid action admitted')
    print('PASS comparison action validation, disjoint gains, lost validity and equal-time rate')
