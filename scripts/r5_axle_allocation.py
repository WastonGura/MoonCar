"""Fixed-contact, test-only actuator allocation and independent raw qualification."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from r5_csv import read_numeric_csv
from r5_load_diagnostic import semantic
from r5_d0_analysis import wheel_budget as legacy_budget, WHEELS


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def factors(front, rear):
    if not np.isfinite([front,rear]).all() or min(front,rear)<=0:raise ValueError('Invalid normal estimates')
    ff=2*front/(front+rear);rr=2-ff
    if max(abs(ff-1),abs(rr-1))>.05:raise ValueError('Outside fixed 5% allocation scope')
    return dict(front=ff,rear=rr)


def allocate(native, ratios, enabled):
    native=np.asarray(native,float)
    if native.ndim!=2 or native.shape[1]!=4 or not np.isfinite(native).all():raise ValueError('Missing native targets')
    if np.max(abs(native[:,0]-native[:,2]))>1e-12 or np.max(abs(native[:,1]-native[:,3]))>1e-12:
        raise ValueError('Unexpected native axle allocation')
    pure=(abs(native[:,0]+native[:,1])<=1e-12)&(abs(native[:,0]-native[:,1])>1e-12)
    applied=pure & bool(enabled)
    out=native.copy()
    out[applied,:2]*=ratios['front'];out[applied,2:]*=ratios['rear']
    return out,applied


def generate(source, output):
    source=Path(source);output=Path(output);output.mkdir()
    if sha(source)!='eda2d3e04a669d3aa5b2ee0633fe9c8bd5daac9f638ee1364158426bc384c81a':
        raise ValueError('Previously verified fixed contact model changed: '+sha(source))
    base=ET.parse(source);slip=base.find("model/plugin[@name='gz::sim::systems::WheelSlip']")
    if slip is None or base.find("model/plugin[@name='mooncar::AxleAllocation']") is not None:raise ValueError('Wrong baseline systems')
    loads={w:float(slip.find(f"wheel[@link_name='{w}_wheel_link']/wheel_normal_force").text) for w in WHEELS}
    if loads['front_left']!=loads['front_right'] or loads['rear_left']!=loads['rear_right']:raise ValueError('Asymmetric estimate')
    ratio=factors(loads['front_left'],loads['rear_left'])
    original=deepcopy(base)
    p=ET.SubElement(base.find('model'),'plugin',filename='/evidence/allocator/libmooncar-axle-allocation.so',name='mooncar::AxleAllocation')
    for key,value in [('enabled','false'),('front_factor',format(ratio['front'],'.17g')),('rear_factor',format(ratio['rear'],'.17g')),('output','UNSET')]:
        ET.SubElement(p,key).text=value
    base.write(output/'baseline.sdf');base.write(output/'restore.sdf')
    candidate=deepcopy(base);candidate.find("model/plugin[@name='mooncar::AxleAllocation']/enabled").text='true'
    candidate.write(output/'candidate.sdf')
    restored=deepcopy(candidate);restored.find("model/plugin[@name='mooncar::AxleAllocation']/enabled").text='false'
    if semantic(restored.getroot())!=semantic(base.getroot()):raise ValueError('Candidate changed more than control enable')
    base.find('model').remove(p)
    if semantic(base.getroot())!=semantic(original.getroot()):raise ValueError('Physical/contact definitions changed')
    for n in ('baseline','candidate','restore'):(output/(n+'.urdf')).write_bytes(source.with_suffix('.urdf').read_bytes())
    manifest=dict(ratios=ratio,normal_estimates_N=loads,physical_and_contact_parameters_unchanged=True,
        single_candidate_change='Test actuator module enabled false -> true',production_changed=False,
        conditional_prediction='Equal lateral compliances at equal side target; steady ICR X approaches zero, not a measured outcome',
        engineering_turn_screen_m=.01,yaw_loss_limit_rad=.05,models={n:sha(output/(n+'.sdf')) for n in ('baseline','candidate','restore')})
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');return manifest


def qualify_budget(active, path, linear, angular, enabled, ratios):
    """Both real pre-allocation and actual post-allocation targets stay in evidence."""
    rows=read_numeric_csv(path,delimiter=',',names=True)
    fields=['iteration','t','dt','enabled','applied']+[w+s for s in ('_native','_allocated') for w in WHEELS]
    if len(rows)<2 or any(k not in rows.dtype.names or not np.isfinite(rows[k]).all() for k in fields):
        return dict(passed=False,reason='Missing/nonfinite allocation records')
    if not (np.diff(rows['iteration'])==1).all():return dict(passed=False,reason='Allocation steps missing')
    index=np.searchsorted(rows['iteration'],active['iteration'])
    if (index>=len(rows)).any() or not np.array_equal(rows['iteration'][index],active['iteration']):
        return dict(passed=False,reason='No exact allocator/observer iteration pairing')
    rows=rows[index]
    if not np.array_equal(np.rint(rows['t']*1e9),np.rint(active['t']*1e9)) or not np.array_equal(rows['dt'],active['dt']):
        return dict(passed=False,reason='Allocator time grid mismatch')
    native=np.column_stack([rows[w+'_native'] for w in WHEELS])
    expected,applied=allocate(native,ratios,enabled)
    logged=np.column_stack([rows[w+'_allocated'] for w in WHEELS])
    actual=np.column_stack([active[w+'_wheel_joint_velocity_cmd'] for w in WHEELS])
    recorded=bool(np.isfinite(actual).all() and np.max(abs(logged-expected))<=1e-12 and
        np.max(abs(actual-logged))<=1e-12 and (rows['enabled']==int(enabled)).all() and np.array_equal(rows['applied'],applied.astype(int)))
    # This copy holds ACTUALLY RECORDED native commands, not inferred/debiased commands.
    raw=active.copy()
    for i,w in enumerate(WHEELS):raw[w+'_wheel_joint_velocity_cmd']=native[:,i]
    if linear or angular:
        native_budget=legacy_budget(raw,linear,angular)
    else:
        native_budget=dict(passed=bool(np.max(abs(native))<=1e-12),linear_integral_m=0.,angular_integral_rad=0.)
    left=(actual[:,0]+actual[:,2])/2;right=(actual[:,1]+actual[:,3])/2
    v=(left+right)*.076/2;omega=(right-left)*.076/.3311
    before_left=(native[:,0]+native[:,2])/2;before_right=(native[:,1]+native[:,3])/2
    means_equal=bool(np.max(abs(left-before_left))<=1e-12 and np.max(abs(right-before_right))<=1e-12)
    tail=bool(np.max(abs(actual[-1]))<=1e-12)
    return dict(passed=bool(recorded and means_equal and tail and native_budget['passed']),
        driver_allocation_verified=recorded,mean_side_targets_preserved=means_equal,native_target_budget=native_budget,
        front_rear_targets_match=bool(np.max(abs(actual[:,:2]-actual[:,2:]))<=1e-10),
        direction_and_acceleration_valid=native_budget.get('direction_and_acceleration_valid',True),
        continuous=native_budget.get('continuous',True),zero_tail=tail,linear_integral_m=float(np.sum(v*active['dt'])),
        angular_integral_rad=float(np.sum(omega*active['dt'])),enabled=enabled,applied_steps=int(applied.sum()),
        allocation_path=str(path),raw_targets_preserved=True)
