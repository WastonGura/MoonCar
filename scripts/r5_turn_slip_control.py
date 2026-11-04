"""Test-only fixed contact gating; no body truth enters the control predicate."""
import copy,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from r5_csv import read_numeric_csv
from r5_contact_source import sha,WHEELS
from r5_load_diagnostic import semantic

def pure_turn(targets):
    a=np.asarray(targets,float)
    if a.ndim!=2 or a.shape[1]!=4 or not np.isfinite(a).all():raise ValueError('Missing/nonfinite four-wheel targets')
    left=(a[:,0]+a[:,2])*.5;right=(a[:,1]+a[:,3])*.5
    return (abs(left+right)<=1e-12)&(abs(right-left)>1e-12)

def generate(source,output):
    if sha(source)!='d765ca1c9c93f9aa837ae128321b9fb14b43b9ecc9e58567121be391eec3d1c4':raise ValueError('Fixed candidate changed')
    p=Path(output);p.mkdir();tree=ET.parse(source);base=copy.deepcopy(tree)
    plugin=ET.SubElement(base.find('model'),'plugin',name='mooncar::TurnSlipControl',filename='/evidence/turn-slip/libmooncar-turn-slip-control.so')
    ET.SubElement(plugin,'mode').text='always';ET.SubElement(plugin,'output').text='UNSET'
    (p/'always.sdf').write_bytes(ET.tostring(base.getroot()))
    candidate=copy.deepcopy(base);candidate.find("model/plugin[@name='mooncar::TurnSlipControl']/mode").text='pure_turn'
    (p/'pure_turn.sdf').write_bytes(ET.tostring(candidate.getroot()))
    (p/'restore.sdf').write_bytes((p/'always.sdf').read_bytes())
    check=copy.deepcopy(candidate);check.find("model/plugin[@name='mooncar::TurnSlipControl']/mode").text='always'
    if semantic(check.getroot())!=semantic(base.getroot()):raise ValueError('Unexpected candidate difference')
    check.find('model').remove(check.find("model/plugin[@name='mooncar::TurnSlipControl']"))
    if semantic(check.getroot())!=semantic(tree.getroot()):raise ValueError('Existing physical/control definitions changed')
    for name in ('always','pure_turn','restore'):(p/(name+'.urdf')).write_bytes(Path(source).with_suffix('.urdf').read_bytes())
    result=dict(models={name:sha(p/(name+'.sdf')) for name in ('always','pure_turn','restore')},
        single_difference='Slip controller always -> pure_turn; native fixed WheelSlip10/0 and axle ratios unchanged',production_changed=False)
    (p/'manifest.json').write_text(json.dumps(result,indent=2)+'\n');return result

def qualify(physics,result,path,mode,parameters):
    rows=read_numeric_csv(path,delimiter=',',names=True);rows=np.atleast_1d(rows)
    if len(rows)<2 or not np.all(np.diff(rows['iteration'])==1):raise ValueError('Slip controller grid missing or duplicated')
    active=physics[(physics['t']>=result['sim_started'])&(physics['t']<=result['sim_ended'])]
    index=np.searchsorted(rows['iteration'],active['iteration'])
    if len(rows)<2 or np.any(index>=len(rows)) or not np.array_equal(rows['iteration'][index],active['iteration']):raise ValueError('Slip controller steps missing')
    selected=rows[index]
    if not np.array_equal(np.rint(selected['t']*1e9),np.rint(active['t']*1e9)) or not np.array_equal(selected['dt'],active['dt']):raise ValueError('Slip controller time mismatch')
    targets=np.column_stack([selected[w+'_target'] for w in WHEELS]);pure=pure_turn(targets)
    if not np.array_equal(selected['pure_turn'],pure) or not np.all(selected['control_mode']==(mode=='pure_turn')):raise ValueError('Actual gate mode/predicate mismatch')
    expected_targets=np.column_stack([active[w+'_wheel_joint_velocity_cmd'] for w in WHEELS])
    if not np.array_equal(targets,expected_targets):raise ValueError('Slip controller changed or misrecorded wheel targets')
    errors={}
    for w in WHEELS:
        setting=parameters[w]
        native=abs(active[w+'_spin_before'])*setting['wheel_radius']/setting['wheel_normal_force']*setting['slip_compliance_lateral']
        expected=native.copy()
        if mode=='pure_turn':expected[~pure]=0.
        observed=active[w+'_slip1_cmd'];fields=(w+'_before1',w+'_before2',w+'_after1',w+'_after2')
        if not all(np.isfinite(selected[k]).all() for k in fields) or not np.isfinite(native).all() or not np.isfinite(observed).all():raise ValueError('Missing actual compliance records')
        error=float(max(np.max(abs(selected[w+'_before1']-native)),np.max(abs(selected[w+'_after1']-expected)),np.max(abs(observed-expected))))
        if error>1e-12 or not np.all(selected[w+'_present']==1) or not np.all(active[w+'_slip_present']==1):raise ValueError('Actual contact command/profile mismatch')
        if np.any(selected[w+'_before2']) or np.any(selected[w+'_after2']) or np.any(active[w+'_slip2_cmd']):raise ValueError('Longitudinal command changed')
        errors[w]=error
    marker=Path(str(path)+'.paused').read_text().split(',');boundary=result['schedule']['stop']['after']
    if int(marker[0])!=boundary['iteration'] or abs(float(marker[1])-boundary['sim'])>1e-9:raise ValueError('Slip controller pause/flush mismatch')
    return dict(passed=True,mode=mode,pure_turn_steps=int(pure.sum()),other_steps=int((~pure).sum()),maximum_errors=errors,
        wheel_targets_unchanged=True,longitudinal_zero_preserved=True,actual_observer_commands_match=True)
