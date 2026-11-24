"""Read-only support, settling, displacement and step-convergence audit."""
import argparse,json
from pathlib import Path
import numpy as np
from r5_csv import read_numeric_csv
from r5_suspension import WHEELS

def metrics(folder,spec):
    r=json.loads((folder/'result.json').read_text());p=read_numeric_csv(folder/'physics-steps.csv',usecols=('iteration','t','x','y','z','qw','qx','qy','qz','com_world_x','com_world_y','com_world_z'))
    start=r['schedule']['motion']['before']['sim'];end=r['schedule']['motion']['after']['sim']
    move=(p['t']>=start-1e-9)&(p['t']<=end+1e-9);steady=(p['t']>=start+3)&(p['t']<=end-1)
    xy=np.column_stack([p[k][move] for k in ('x','y')]);com=np.column_stack([p['com_world_'+k][move] for k in 'xyz'])
    w,x,y,z=(p[k][move] for k in ('qw','qx','qy','qz'))
    yaw=np.unwrap(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))
    c=read_numeric_csv(next(folder.glob('*-contact-steps.csv')),delimiter=',',names=True,
                   usecols=('iteration','t_ns','wheel','state','f1z'))
    parked_contact=c[(c['t_ns']>=round((start-1)*1e9))&(c['t_ns']<=round(start*1e9))]
    parked_empty={}
    for index,wheel in enumerate(WHEELS):
        a=parked_contact[parked_contact['wheel']==index];_,ix=np.unique(a['iteration'],return_index=True)
        parked_empty[wheel]=float((a['state'][ix]==0).mean())
    lo=round((start+3)*1e9);hi=round((end-1)*1e9);c=c[(c['t_ns']>=lo)&(c['t_ns']<=hi)]
    empty={};force={}
    for index,wheel in enumerate(WHEELS):
        a=c[c['wheel']==index];_,ix=np.unique(a['iteration'],return_index=True)
        empty[wheel]=float((a['state'][ix]==0).mean())
        force[wheel]=float(a['f1z'][a['state']==1].sum()/len(ix))
    tail=p['t']>=p['t'][-1]-1;velocity=np.linalg.norm(np.diff(np.column_stack([p[k][tail] for k in ('x','y','z')]),axis=0),axis=1)/spec['step']
    before=(p['t']>=start-1)&(p['t']<=start)
    parked_velocity=np.linalg.norm(np.diff(np.column_stack([p[k][before] for k in ('x','y','z')]),axis=0),axis=1)/spec['step']
    result=dict(name=spec['name'],specification=spec,reference_max_mm=float(np.max(np.linalg.norm(xy-xy[0],axis=1))*1000),
        com_max_mm=float(np.max(np.linalg.norm((com-com[0])[:,:2],axis=1))*1000),
        endpoint_xy_mm=((xy[-1]-xy[0])*1000).tolist(),com_endpoint_mm=((com[-1]-com[0])*1000).tolist(),
        actual_turn_deg=float(np.degrees(yaw[-1]-yaw[0])),empty_fraction=empty,mean_vertical_contact_N=force,
        parked_speed_max_m_s=float(parked_velocity.max()),parked_empty_fraction=parked_empty,
        stopped_last_second_speed_max_m_s=float(velocity.max()),
        trajectory=r.get('turn_slip_probe',{}).get('trajectory'),diagnostic_qualified=json.loads((folder/'suspension-qualification.json').read_text())['diagnostic_qualified'])
    if not spec['rigid']:
        s=read_numeric_csv(folder/'suspension.csv')
        assert np.array_equal(s['iteration'],p['iteration']) and np.array_equal(s['t'],p['t'])
        result['ecm_pose_difference_parked_speed_max_m_s']=result['parked_speed_max_m_s']
        result['ecm_pose_difference_stopped_speed_max_m_s']=result['stopped_last_second_speed_max_m_s']
        body_speed=np.linalg.norm(np.column_stack([s[WHEELS[0]+'_wheel_suspension_joint_parent_world_v'+k] for k in 'xyz']),axis=1)
        result['parked_speed_max_m_s']=float(body_speed[before].max())
        result['stopped_last_second_speed_max_m_s']=float(body_speed[tail].max())
        result['body_speed_source']='Physics link WorldLinearVelocity component; cached pose differences retained separately'
        result['travel']={}
        for wheel in WHEELS:
            q=s[wheel+'_wheel_suspension_joint_q'];v=s[wheel+'_wheel_suspension_joint_v']
            result['travel'][wheel]=dict(min_m=float(q[steady].min()),max_m=float(q[steady].max()),
                full_min_m=float(q.min()),full_max_m=float(q.max()),
                full_limit_fraction=float((abs(q)>=.02-1e-5).mean()),
                parked_mean_m=float(q[before].mean()),parked_velocity_max=float(abs(v[before]).max()),
                steady_limit_fraction=float((abs(q[steady])>=.02-1e-5).mean()),
                all_steps_within_limits=bool(abs(q).max()<=.02001))
        result['support_target_passed']=bool(all(f<=.05 for f in empty.values()))
        result['flat_static_passed']=bool(spec['scene']=='flat' and
            result['parked_speed_max_m_s']<.001 and all(f<.01 for f in parked_empty.values()))
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--prepared',required=True);p.add_argument('--campaign');p.add_argument('--static');p.add_argument('--output',required=True);a=p.parse_args()
    out=Path(a.output);out.mkdir(parents=True,exist_ok=False);m=json.loads((Path(a.prepared)/'manifest.json').read_text())
    static=metrics(Path(a.static)/m['static_bench']['name'],m['static_bench']) if a.static else None
    rows=[];ledger=None
    if a.campaign:
        ledger=json.loads((Path(a.campaign)/'campaign.json').read_text())
        for attempt in ledger['attempts']:
            if attempt.get('qualification',{}).get('diagnostic_qualified'):
                spec=next(s for s in m['rounds'] if s['name']==attempt['name']);rows.append(metrics(Path(a.campaign)/attempt['name'],spec))
    convergence=[]
    for rigid in (True,False):
        names={m['rounds'][i]['name'] for i in ((10,11) if rigid else (12,13))}
        selected={r['specification']['step']:r for r in rows if r['name'] in names}
        if .0005 not in selected or .00025 not in selected:continue
        b,c=selected[.0005],selected[.00025];limit=max(2,.05*max(b['reference_max_mm'],c['reference_max_mm']))
        displacement=abs(b['reference_max_mm']-c['reference_max_mm']);angle=abs(b['actual_turn_deg']-c['actual_turn_deg'])/max(abs(b['actual_turn_deg']),abs(c['actual_turn_deg']))
        travel=max(abs(b['travel'][w][k]-c['travel'][w][k]) for w in WHEELS for k in ('full_min_m','full_max_m')) if not rigid else 0.
        convergence.append(dict(rigid=rigid,displacement_difference_mm=displacement,displacement_limit_mm=limit,
            turn_fraction_difference=angle,travel_difference_m=travel,passed=bool(displacement<=limit and angle<=.02 and travel<=.001)))
    if static is None:static=next((r for r in rows if r['specification']['scene']=='flat' and not r['specification']['rigid']),None)
    from r5_suspension_comparison import compare_campaign
    comparison=compare_campaign(Path(a.campaign),m,rows,out) if a.campaign else {}
    complete=bool(ledger and ledger['status']=='completed' and len(rows)==17 and ledger['freeze_unchanged'])
    precision_gate=bool(ledger and ledger.get('static_gate',{}).get('passed') and ledger['static_gate'].get('physics_profile')==m.get('physics_profile'))
    engineering=bool(complete and precision_gate and static and static['flat_static_passed'] and len(convergence)==2 and
        all(r['passed'] for r in convergence) and comparison.get('passed') and
        all(r['stopped_last_second_speed_max_m_s']<.001 for r in rows) and
        all(r['flat_static_passed'] for r in rows if r['specification']['scene']=='flat') and
        all(r['support_target_passed'] and all(x['all_steps_within_limits'] for x in r['travel'].values()) for r in rows if not r['specification']['rigid']))
    result=dict(physics_profile=m.get('physics_profile'),static=static,rounds=rows,convergence=convergence,campaign=ledger,comparison=comparison,
                engineering_passed=engineering,label='支撑模型修复候选' if engineering else '验收未通过',
                production_changed=False,hardware_calibrated=False)
    (out/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(engineering_passed=engineering,rounds=len(rows),convergence=convergence),indent=2))
    return 0 if engineering else 1
if __name__=='__main__':raise SystemExit(main())
