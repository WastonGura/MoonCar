"""Readonly, synchronous native/ECM/frame/force checks of the 12-trial batch."""
import argparse
import json
from pathlib import Path
import numpy as np
from r5_suspension import WHEELS


def relative_z(data,name):
    prefix=name+'_' if name else ''
    delta=np.column_stack([data[prefix+'child_'+x]-data[prefix+'parent_'+x] for x in 'xyz'])
    w,x,y,z=[data[prefix+'parent_'+x] for x in ('qw','qx','qy','qz')]
    axis=np.column_stack([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
    return (delta*axis).sum(axis=1)


def metrics(trial):
    folder=Path(trial['world']).parent
    data=np.genfromtxt(folder/'suspension.csv',delimiter=',',names=True)
    native=np.genfromtxt(folder/'native.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
    count=trial['iterations'];step=trial['step'];tail=data['t']>=5
    assert len(data)==count and np.array_equal(data['iteration'],np.arange(1,count+1))
    assert np.allclose(data['t'],np.arange(1,count+1)*step,atol=1e-12,rtol=0)
    assert all(np.isfinite(data[n]).all() for n in data.dtype.names if not n.endswith('_cmd'))
    names=['spring'] if trial['kind']=='joint' else [w+'_wheel_suspension_joint' for w in WHEELS]
    result=dict(name=trial['name'],specification=trial,steps=count,joints={})
    for name in names:
        n=native[native['joint']==name]
        assert len(n)==count, f'Native acquisition count mismatch {name}: {len(n)}'
        assert all(np.isfinite(n[k]).all() for k in n.dtype.names if k!='joint')
        q=data[name+'_q'];v=data[name+'_v'];g=relative_z(data,name)
        native_geometry=relative_z(n,'')
        assert np.isfinite(q).all() and np.isfinite(v).all()
        measured=dict(q_mean_m=float(q[tail].mean()),q_min_m=float(q.min()),q_max_m=float(q.max()),
            velocity_tail_max_m_s=float(abs(v[tail]).max()),native_ecm_q_max_difference_m=float(abs(q-n['q']).max()),
            geometry_q_max_difference_m=float(abs((g-q)-(g[0]-q[0])).max()),
            native_geometry_q_max_difference_m=float(abs((native_geometry-q)-(native_geometry[0]-q[0])).max()),
            transmitted_fz_mean_N=float(data[name+'_fz'][tail].mean()),
            transmitted_torque_mean_Nm=[float(data[name+'_'+k][tail].mean()) for k in ('tx','ty','tz')],
            native_body_fz_mean_N=float(n['fz_body'][tail].mean()),
            native_acceleration_mean_m_s2=float(n['a'][tail].mean()),
            native_velocity_change_mean_m_s=float(n['velocity_change_native'][tail].mean()),
            native_force_mean_N=float(n['force_native'][tail].mean()),
            native_constraint_impulse_mean_Ns=float(n['constraint_impulse_native'][tail].mean()))
        parameters=dict(k_N_m=float(n['k_native'][0]),c_N_s_m=float(n['c_native'][0]),reference_m=float(n['ref_native'][0]))
        for imported,native_key in [('k_imported','k_native'),('c_imported','c_native'),('ref_imported','ref_native')]:
            assert np.array_equal(data[name+'_'+imported],n[native_key]),'Consumed spring parameter mismatch'
        spring=n['k_native']*(n['q']-n['ref_native'])
        damping=n['c_native']*n['v']
        implicit=spring+n['k_native']*step*n['v']+n['k_native']*step*step*n['a']+damping+n['c_native']*step*n['a']
        free_acceleration=n['a']-n['velocity_change_native']/step
        pre_constraint=spring+(n['k_native']*step*step+n['c_native']*step)*free_acceleration
        formula=dict(elastic_force_mean_N=float(spring[tail].mean()),damping_force_mean_N=float(damping[tail].mean()),
                     implicit_spring_damping_force_mean_N=float(implicit[tail].mean()),
                     pre_constraint_acceleration_mean_m_s2=float(free_acceleration[tail].mean()),
                     pre_constraint_implicit_force_mean_N=float(pre_constraint[tail].mean()))
        if trial['kind']=='joint':
            target=trial['proof']['expected_q'];compression=abs(target-parameters['reference_m']) or .01
            error=abs(measured['q_mean_m']-target)
            envelope=[float(abs(q[(data['t']>=i)&(data['t']<i+1)]-target).max()) for i in range(6)]
            passed=bool(error/compression<=.05 and measured['q_min_m']>=-.02001 and measured['q_max_m']<=.02001
                and measured['velocity_tail_max_m_s']<.001 and measured['geometry_q_max_difference_m']<3e-6
                and measured['native_geometry_q_max_difference_m']<1e-9
                and measured['native_ecm_q_max_difference_m']<1e-9
                and (trial['case']!='decay' or envelope[-1]<.01*envelope[0]))
            if trial['case']=='load':passed=passed and measured['transmitted_fz_mean_N']*trial['proof']['gravity_z']<0
            formula.update(expected_q_m=target,error_m=error,relative_compression_error=error/compression,envelope_m=envelope)
        else:
            transmitted=abs(measured['transmitted_fz_mean_N'])
            elastic=abs(formula['elastic_force_mean_N'])
            formula.update(axial_elastic_balance_fraction=abs(elastic-transmitted)/max(transmitted,1e-12),
                axial_equilibrium_q_m=parameters['reference_m']+transmitted/parameters['k_N_m'],
                axial_compression_error_fraction=abs(elastic-transmitted)/max(transmitted,1e-12))
            axial=np.abs(data[name+'_fz'])
            predicted=n['ref_native']+axial/n['k_native']
            formula.update(axial_compression_error_max_m=float(np.abs(n['q'][tail]-predicted[tail]).max()),
                axial_force_balance_error_max_fraction=float((np.abs(np.abs(spring[tail])-axial[tail])/np.maximum(axial[tail],1e-12)).max()))
            passed=bool(measured['velocity_tail_max_m_s']<.001 and measured['q_min_m']>=-.02001
                and measured['q_max_m']<=.02001 and measured['geometry_q_max_difference_m']<3e-6
                and measured['native_geometry_q_max_difference_m']<1e-9
                and measured['native_ecm_q_max_difference_m']<1e-9)
        result['joints'][name]=dict(measured=measured,native_parameters=parameters,formula=formula,mechanics_passed=passed)
    if trial['kind']=='vehicle':
        physics=np.genfromtxt(folder/'physics-steps.csv',delimiter=',',names=True)
        assert np.array_equal(data['iteration'],physics['iteration']) and np.array_equal(data['t'],physics['t'])
        xyz=np.column_stack([physics[k][tail] for k in 'xyz'])
        result['ecm_pose_difference_tail_speed_max_m_s']=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).max()/step)
        body=native[native['joint']==names[0]]
        native_xyz=np.column_stack([body['parent_'+k][tail] for k in 'xyz'])
        result['body_tail_speed_max_m_s']=float(np.linalg.norm(np.diff(native_xyz,axis=0),axis=1).max()/step)
        result['body_speed_source']='Native DART parent-link positions at each complete physical step'
        contacts=np.genfromtxt(folder/'contacts.csv',delimiter=',',names=True)
        contacts=contacts[contacts['t_ns']>=5e9]
        result['contact']={}
        for index,wheel in enumerate(WHEELS):
            c=contacts[contacts['wheel']==index];iterations,ix=np.unique(c['iteration'],return_index=True)
            assert np.array_equal(iterations,data['iteration'][tail]),'Contact acquisition missing step'
            result['contact'][wheel]=dict(empty_fraction=float((c['state'][ix]==0).mean()),
                vertical_force_mean_N=float(c['f1z'][c['state']==1].sum()/len(ix)))
        if not trial['drive_enabled']:
            assert all(np.isnan(data[name+'_drive_velocity_cmd']).all() for name in names),'Ablation created drive commands'
            policy=np.genfromtxt(folder/'idle-slip.csv',delimiter=',',names=True)
            assert (policy['lateral_after']==0).all(),'Idle contact policy differs from active zero input'
            result['drive_commands_absent']=True
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--diagnostic',required=True);args=parser.parse_args()
    out=Path(args.diagnostic);manifest=json.loads((out/'manifest.json').read_text());ledger=json.loads((out/'ledger.json').read_text())
    results=[]
    for attempt in ledger['attempts']:
        trial=next(t for t in manifest['trials'] if t['name']==attempt['name'])
        if attempt.get('failure'):continue
        results.append(metrics(trial))
    native_passed=len(results)>=8 and all(j['mechanics_passed'] for r in results[:8] for j in r['joints'].values())
    fine=next((r for r in results if r['specification']['kind']=='vehicle' and r['specification'].get('step')==.00025),None)
    fine_balance_passed=bool(fine and all(j['formula']['axial_elastic_balance_fraction']<=.05 for j in fine['joints'].values()))
    result=dict(physics_profile=manifest.get('physics_profile'),results=results,ledger=ledger,native_bench_passed=native_passed,fine_axial_balance_passed=fine_balance_passed,
                bias_explanation_approved=False,formal_matrix_allowed=False,production_changed=False,hardware_calibrated=False)
    (out/'analysis.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(trials=len(results),native_bench_passed=native_passed,fine_axial_balance_passed=fine_balance_passed)))
if __name__=='__main__':main()
