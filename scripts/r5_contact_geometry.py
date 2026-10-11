"""Qualification for the approved eight-round geometry diagnostic, not acceptance."""
import copy
import csv
import math
from pathlib import Path
from run_r5_origin_campaign import pair_inputs

PROFILES=('plate','coplanar_mesh','mirrored_mesh','restore_plate')
MATRIX=tuple((p,case,angular) for p in PROFILES for case,angular in (('left',.2),('right',-.2)))
SCREEN_ERROR="ValueError('Existing 1 cm turn screen failed')"

def mesh_receipt(proof):
    """Bind native loader evidence to both actual fixture paths."""
    if proof.get('passed') is not True or proof.get('new_simulations')!=0:return False
    rows=proof.get('meshes',[])
    expected={'/probe/fixtures/coplanar_mesh.obj','/probe/fixtures/mirrored_mesh.obj'}
    if len(rows)!=2 or {r.get('path') for r in rows}!=expected:return False
    return all(r.get('passed') is True and isinstance(r.get('vertices'),int) and r['vertices']>0
        and r.get('normals')==r['vertices'] and isinstance(r.get('indices'),int) and r['indices']>0
        and r['indices']%3==0 for r in rows)

def qualify(result,code):
    """Only the declared geometry output may fail; original result is preserved."""
    p=result.get('turn_slip_probe',{})
    checks={k:bool(result.get(k)) for k in ('data_complete','ground_contact_gate','stop_zero',
        'observer_flush_confirmed','receiver_verified','raw_fixture_passed')}
    for key in ('parked','step_observation','capture_analysis','image_trace_analysis','dds',
                'formal','origin_qualification','image_content','lifecycle_capture'):
        checks[key]=result.get(key,{}).get('passed') is True
    checks['capture_drained']=result.get('capture',{}).get('drained') is True
    checks['gate']=p.get('gate',{}).get('passed') is True
    checks['allocation']=p.get('allocation',{}).get('passed') is True
    checks['contact_source']=p.get('source',{}).get('proof',{}).get('passed') is True
    checks['no_other_errors']=not any(result.get(k) for k in ('error','production_error','origin_error','secondary_collection_error')) and not result.get('errors')
    checks['tilt_and_stop']=math.isfinite(result.get('max_tilt_rad',math.inf)) and result.get('max_tilt_rad',math.inf)<=.1 and result.get('stop_drift_last_second_m',math.inf)<=.01
    displacement=p.get('trajectory',{}).get('maximum_displacement_m',math.inf)
    checks['finite_geometry']=math.isfinite(displacement)
    screen=displacement<=.01
    checks['original_exit']=bool(code==0 and result.get('passed') and not result.get('turn_slip_error')) if screen else bool(code==1 and result.get('passed') is False and result.get('turn_slip_error')==SCREEN_ERROR)
    return dict(eligible=all(checks.values()),checks=checks,original_passed=result.get('passed'),
        geometric_screen_passed=screen,maximum_displacement_m=displacement,
        vehicle_accepted=False,goal_achieved=False,production_changed=False)

def inputs(a,b):
    # Use the existing exact input criteria, without changing either old result.
    ra,rb=copy.deepcopy(a),copy.deepcopy(b)
    ra['passed']=qualify(a,0 if a.get('passed') else 1)['eligible']
    rb['passed']=qualify(b,0 if b.get('passed') else 1)['eligible']
    return pair_inputs(ra,rb)

def metadata(path,other_ids):
    with Path(path).open() as stream:rows=list(csv.DictReader(stream))
    names={int(r['entity']):r['value'] for r in rows if r['kind']=='collision'}
    detector=[r['value'] for r in rows if r['kind']=='collision_detector']
    if len(detector)!=1 or not names or any(int(i) not in names for i in other_ids):
        raise ValueError('Incomplete collision identity metadata')
    return dict(collision_detector=detector[0] if detector[0]!='MISSING' else None,
        collision_detector_available=detector[0]!='MISSING',
        other_collisions={str(int(i)):names[int(i)] for i in other_ids},collision_registry=names,
        no_state_changes=True)
