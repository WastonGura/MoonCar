"""Full ROS fixture for a predeclared suspension intervention."""
import argparse,copy,hashlib,json,shutil,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from r5_csv import aligned_observations
from r5_robot_description import validate_bound_description,qualify_feedback

def qualify_observation(data,physical):
    return bool(np.array_equal(data['iteration'],physical['iteration']) and np.array_equal(data['t'],physical['t']) and
        all(np.isfinite(data[n]).all() for n in data.dtype.names if not n.endswith('_cmd')))

def main():
    p=argparse.ArgumentParser();p.add_argument('--prepared',required=True);p.add_argument('--output',required=True);p.add_argument('--name',required=True);a=p.parse_args()
    m=json.loads((Path(a.prepared)/'manifest.json').read_text());s=next(r for r in m['rounds']+[m['static_bench']] if r['name']==a.name)
    assert hashlib.sha256(Path(s['world']).read_bytes()).hexdigest()==s['world_sha256']
    description_proof=validate_bound_description(s)
    import check_r5_runtime as runtime
    original_runtime_run=runtime.run
    def bound_run(args):
        args.robot_description_urdf=s['robot_description_urdf']
        args.robot_state_publish_frequency_hz=s['robot_state_publish_frequency_hz']
        args.ground_reference_profile=s['ground_reference_profile']
        return original_runtime_run(args)
    runtime.run=bound_run
    import r5_axle_robustness
    validate=r5_axle_robustness.validate
    def declared_step(scene,step):
        if step!=s['step']:raise ValueError('Undeclared integration step')
        validate(scene,step)
    r5_axle_robustness.validate=declared_step
    import check_r5_coplanar_round
    original_motion=check_r5_coplanar_round.motion_qualification
    def motion(diagnostic,result,linear,angular):
        if linear==0 and angular==0:
            checks=dict(diagnostic['checks'])
            # The archived raw drive fixture requires one metre of forward
            # travel. Preserve its failure; independently qualify the declared
            # zero-input bench using all existing zero/static/capture gates.
            zero=result.get('formal',{}).get('command_window',{}).get('zero_input')
            passed=bool(zero and result.get('passed') and result.get('parked',{}).get('passed') and
                        all(v for k,v in checks.items() if k!='raw_fixture_passed'))
            return dict(passed=passed,kind='zero',raw_forward_distance_failure_retained=True,checks=checks)
        return original_motion(diagnostic,result,linear,angular)
    check_r5_coplanar_round.motion_qualification=motion
    original_write=ET.ElementTree.write
    def observation_path(tree,target,*args,**kwargs):
        plugin=tree.find("world/model[@name='mooncar']/plugin[@name='mooncar::SuspensionObserver']")
        if plugin is not None:
            plugin.find('output').text=str(Path(a.output)/a.name/'suspension.csv')
        return original_write(tree,target,*args,**kwargs)
    ET.ElementTree.write=observation_path
    old=sys.argv
    try:
        sys.argv=['round','--output',a.output,'--name',a.name,'--world',s['world'],
            '--ode-profile','candidate','--candidate-runtime','/opt/mooncar/turn/r5-v2',
            '--manifest','/opt/mooncar/turn/r5-v2/manifest.json','--round-fixture',m['fixture'],
            '--angular',str(s['angular']),'--linear',str(s['linear'])]
        code=check_r5_coplanar_round.main()
    finally:
        sys.argv=old;r5_axle_robustness.validate=validate
        check_r5_coplanar_round.motion_qualification=original_motion
        ET.ElementTree.write=original_write
        runtime.run=original_runtime_run
    folder=Path(a.output)/a.name
    def semantic(e):return e.tag,sorted(e.attrib.items()),(e.text or '').strip(),tuple(semantic(c) for c in e)
    def normalized(path):
        root=ET.parse(path).getroot();model=root.find("world/model[@name='mooncar']")
        for plugin in model.findall('plugin'):
            if plugin.get('name') in ('mooncar::StepObserver','mooncar::ContactObserver'):model.remove(plugin)
            else:
                for e in plugin.findall('output'):e.text='OBSERVATION_PATH'
        return semantic(root)
    declared=normalized(s['world'])==normalized(folder/'world.sdf')
    loader=json.loads((Path(a.output)/(a.name+'-loader.json')).read_text())
    zero_qualified=bool(s['linear']==0 and s['angular']==0 and code==1 and
        loader['fixture_exit']==1 and not loader['error'] and loader['library_verified'] and
        loader['motion_qualification'].get('kind')=='zero' and loader['motion_qualification']['passed'])
    if not ((code==0 and loader['passed']) or zero_qualified):
        (folder/'suspension-qualification.json').write_text(json.dumps(dict(specification=s,
            diagnostic_qualified=False,library_verified=loader['library_verified'],
            exact_declared_world_consumed=declared,loader=loader),indent=2)+'\n')
        return 1
    observation=True
    feedback=qualify_feedback(folder,s['robot_description_urdf'])
    if not s['rigid']:
        src=folder/'suspension.csv'
        observation=aligned_observations(src,folder/'physics-steps.csv')
    result=dict(specification=s,diagnostic_qualified=bool(((code==0 and loader['passed']) or zero_qualified) and declared and observation and feedback['passed']),
        robot_description_qualification=description_proof,
        joint_state_tf_qualification=feedback,
        exact_declared_world_consumed=declared,suspension_observation_complete=observation,
        zero_input_qualified_separately=zero_qualified,historical_raw_forward_gate_retained=True,
        library_verified=loader['library_verified'],physical_model_accepted=False,production_changed=False)
    (folder/'suspension-qualification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    return 0 if result['diagnostic_qualified'] else 1
if __name__=='__main__':raise SystemExit(main())
