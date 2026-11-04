"""Declare exactly twelve mechanics trials, separately from the 17-round matrix."""
import argparse
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from check_r5_suspension_bench import ENGINE,OBSERVER
from r5_flat_scene import validate_flat
from prepare_r5_suspension import sha
from r5_monitoring import validate_monitoring


def bench_world(parameter,case,output,step=.00025):
    # Same native spring parameters as the rover. Mirrored axial loading puts
    # equilibrium inside the travel range, away from a constraint reaction.
    gravity={'load':3.24,'decay':0,'lower':-16.2,'upper':16.2}[case]
    mass=parameter['force_N']/1.62
    root=ET.Element('sdf',version='1.10');world=ET.SubElement(root,'world',name='bench')
    plugin=ET.SubElement(world,'plugin',filename='gz-sim-physics-system',name='gz::sim::systems::Physics')
    ET.SubElement(ET.SubElement(plugin,'engine'),'filename').text=ENGINE
    physics=ET.SubElement(world,'physics',name='physics',type='ode')
    ET.SubElement(physics,'max_step_size').text=str(step);ET.SubElement(physics,'real_time_factor').text='0'
    ET.SubElement(world,'gravity').text=f'0 0 {gravity}'
    model=ET.SubElement(world,'model',name='bench')
    for name,m in [('anchor',1),('load',mass)]:
        link=ET.SubElement(model,'link',name=name);ET.SubElement(link,'pose').text='0 0 1 0 0 0'
        inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'mass').text=str(m)
        tensor=ET.SubElement(inertial,'inertia')
        for field in ('ixx','iyy','izz','ixy','ixz','iyz'):
            ET.SubElement(tensor,field).text='.01' if field in ('ixx','iyy','izz') else '0'
    joint=ET.SubElement(model,'joint',name='fixed',type='fixed')
    ET.SubElement(joint,'parent').text='world';ET.SubElement(joint,'child').text='anchor'
    joint=ET.SubElement(model,'joint',name='spring',type='prismatic')
    ET.SubElement(joint,'parent').text='anchor';ET.SubElement(joint,'child').text='load'
    axis=ET.SubElement(joint,'axis');ET.SubElement(axis,'xyz').text='0 0 1'
    dynamics=ET.SubElement(axis,'dynamics')
    for key,value in [('spring_stiffness',parameter['k_N_m']),('damping',parameter['c_N_s_m']),
                       ('spring_reference',parameter['reference_m'])]:ET.SubElement(dynamics,key).text=str(value)
    limits=ET.SubElement(axis,'limit')
    ET.SubElement(limits,'lower').text=str(parameter['lower_m']);ET.SubElement(limits,'upper').text=str(parameter['upper_m'])
    ET.SubElement(limits,'effort').text='50';ET.SubElement(limits,'velocity').text='.5'
    plugin=ET.SubElement(model,'plugin',name='mooncar::SuspensionObserver',filename=OBSERVER)
    ET.SubElement(plugin,'output').text=str(output);ET.SubElement(plugin,'joint').text='spring'
    target=parameter['reference_m']+mass*gravity/parameter['k_N_m']
    target=max(parameter['lower_m'],min(parameter['upper_m'],target))
    return ET.ElementTree(root),dict(gravity_z=gravity,load_mass_kg=mass,expected_q=target,parameters=parameter)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--prepared',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    campaign=json.loads((Path(args.prepared)/'manifest.json').read_text());trials=[]
    validate_monitoring(campaign)
    precision=campaign['physics_profile']['name']=='r5_precision_v1'
    bench_step=.00025 if precision else .001
    for axle in ('front','rear'):
        parameter=campaign['physical_parameters']['parameters'][axle+'_left']
        for case in ('load','decay','lower','upper'):
            name=f'{len(trials)+1:02d}-{axle}-{case}';folder=out/name;folder.mkdir()
            tree,proof=bench_world(parameter,case,folder/'suspension.csv',bench_step)
            path=folder/'world.sdf';tree.write(path)
            trials.append(dict(name=name,kind='joint',case=case,world=str(path),world_name='bench',step=bench_step,
                               iterations=round(6/bench_step),proof=proof,world_sha256=sha(path)))
    vehicle_cases=[(.001,True),(.00025,True),(.00025,True),(.00025,False)] if precision else [(.001,True),(.0005,True),(.00025,True),(.001,False)]
    for step,drive in vehicle_cases:
        name=f'{len(trials)+1:02d}-vehicle-{step:g}-drive-{int(drive)}';folder=out/name;folder.mkdir()
        tree=ET.parse(campaign['static_bench']['world']);world=tree.find('world');world.set('name',f'static_{len(trials)+1}')
        world.find('physics/max_step_size').text=str(step)
        model=world.find("model[@name='mooncar']");removed=[]
        for plugin in list(model.findall('plugin')):
            if not drive and plugin.get('name') in ('gz::sim::systems::DiffDrive','mooncar::AxleAllocation','mooncar::TurnSlipControl'):
                removed.append(plugin.get('name'));model.remove(plugin);continue
            for output in plugin.findall('output'):
                output.text=str(folder/({'mooncar::SuspensionObserver':'suspension.csv',
                    'mooncar::StepObserver':'physics-steps.csv','mooncar::ContactObserver':'contacts.csv',
                    'mooncar::TurnSlipControl':'slip.csv','mooncar::AxleAllocation':'allocation.csv'}[plugin.get('name')]))
        if not drive:
            policy=ET.SubElement(model,'plugin',name='mooncar::IdleSlipPolicy',
                filename='/work/suspension-observer/libmooncar-idle-slip-policy.so')
            ET.SubElement(policy,'output').text=str(folder/'idle-slip.csv')
        qualification=validate_flat(tree,2)
        path=folder/'world.sdf';tree.write(path)
        trials.append(dict(name=name,kind='vehicle',world=str(path),world_name=world.get('name'),step=step,
                           iterations=round(6/step),drive_enabled=drive,removed_execution_plugins=removed,
                           idle_contact_policy_preserved=not drive,
                           scene_qualification=qualification,world_sha256=sha(path)))
    assert len(trials)==12
    manifest=dict(physics_profile=campaign['physics_profile'],monitoring_profile=campaign['monitoring_profile'],maximum_attempts=12,trials=trials,formal_matrix_attempts=0,
                  parameters=campaign['physical_parameters'],hardware_calibrated=False,production_changed=False)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(declared_trials=len(trials),formal_rounds=len(campaign['rounds']))))
if __name__=='__main__':main()
