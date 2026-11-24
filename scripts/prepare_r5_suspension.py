"""Predeclare the 17-round suspension experiment, preserving fixed actuators."""
import argparse,copy,hashlib,json,os,shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from r5_suspension import suspended,WHEELS
from audit_r5_mass import read_sdf,read_urdf,aggregate
from r5_flat_scene import flat_world,validate_spec
from r5_physics_profiles import PROFILES,main_step,profile_record
from r5_suspension_protocol import declared_matrix
from r5_monitoring import monitoring_profile

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def mass(model):
    s=ET.Element('sdf',version='1.10');s.append(copy.deepcopy(model));return aggregate(read_sdf(s))
def compare(a,b):
    assert abs(a['total_mass_kg']-b['total_mass_kg'])<1e-9
    assert np.max(abs(np.array(a['com_m'])-b['com_m']))<1e-9
    assert np.max(abs(np.array(a['inertia_kg_m2'])-b['inertia_kg_m2']))<1e-10

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True)
    p.add_argument('--physics-profile',choices=tuple(PROFILES),default='legacy_1ms');a=p.parse_args()
    out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    observer='/work/suspension-observer/libmooncar-suspension-observer.so'
    originals={k:ET.parse('/oldcause/runtime/prepared/'+n+'/curve-0.001/world.sdf') for k,n in
        [('curve','01-curve-left'),('plane','03-plane-left'),('plane_normal','04-plane-normal-gravity')]}
    # Keep exact plane assets; rewrite archived paths to their read-only mount.
    for tree in originals.values():
        for uri in tree.findall('.//uri'):
            if uri.text.startswith('/diagnosis/prepared/plane-assets/'):
                uri.text=uri.text.replace('/diagnosis/prepared','/oldcause/runtime/prepared',1)
    originals['flat']=flat_world(ET.parse('/oldclosure/prepared-d0/ramp-0.001/world.sdf'),out/'flat-assets')
    specs=declared_matrix(a.physics_profile)
    rounds=[];proof=None
    descriptions={}
    for rigid,name in [(True,'rigid'),(False,'suspended')]:
        path=out/(name+'.urdf');shutil.copyfile('/diagnosis/after/'+name+'.urdf',path)
        descriptions[rigid]=str(path)
    original=originals['curve'].find("world/model[@name='mooncar']")
    neutral,proof=suspended(original,observer,'AUDIT')
    compare(mass(original),mass(neutral))
    before=aggregate(read_urdf(ET.parse('/diagnosis/before/r5.urdf').getroot())[0])
    after=aggregate(read_urdf(ET.parse('/diagnosis/after/suspended.urdf').getroot())[0])
    compare(before,after);compare(before,mass(neutral))
    # Cross-check public Xacro springs against independently derived frozen loads.
    generated=ET.parse('/diagnosis/after/suspended.sdf').find('model')
    for w in WHEELS:
        j=generated.find(f"joint[@name='{w}_wheel_suspension_joint']")
        assert abs(float(j.findtext('axis/dynamics/spring_stiffness'))-proof['parameters'][w]['k_N_m'])<1e-9
        assert abs(float(j.findtext('axis/dynamics/damping'))-proof['parameters'][w]['c_N_s_m'])<1e-9
    specs=[('flat',False,0,main_step(a.physics_profile),0)]+specs
    for index,(scene,rigid,angular,dt,linear) in enumerate(specs):
        name=f'{index:02d}-{scene}-'+('rigid' if rigid else 'suspended')+f'-{angular:g}-{dt:g}'
        if index==0:name='00-static-'+out.name
        else:name=out.name+'-'+name
        root=copy.deepcopy(originals[scene]);world=root.find('world')
        world.find('physics/max_step_size').text=str(dt)
        old=world.find("model[@name='mooncar']")
        if not rigid:
            new,_=suspended(old,observer,f'/work/{name}-suspension.csv')
            world.remove(old);world.append(new)
        folder=out/name/f'{"ramp" if scene=="flat" else "curve"}-{dt:g}';folder.mkdir(parents=True)
        path=folder/'world.sdf';root.write(path)
        rounds.append(dict(name=name,scene=scene,rigid=rigid,angular=angular,linear=linear,step=dt,
            robot_description_urdf=descriptions[rigid],robot_description_sha256=sha(descriptions[rigid]),
            robot_state_publish_frequency_hz=30.,
            ground_reference_profile='qualified_flat_mesh_v1' if scene=='flat' else 'legacy_scene_grid',
            world=str(path),world_sha256=sha(path),slip_compliance_lateral=10.,allocation_enabled=True))
        rounds[-1]['scene_qualification']=validate_spec(rounds[-1])
    fixture=out/'round_fixture.py';text=Path('/oldclosure/prepared-d0/round_fixture.py').read_text()
    fixture.write_text(text.replace('/closure/prepared-d0/formal.sdf','/oldclosure/prepared-d0/formal.sdf'))
    manifest=dict(physics_profile=profile_record(a.physics_profile),monitoring_profile=monitoring_profile(),maximum_attempts=17,rounds=rounds[1:],static_bench=rounds[0],fixture=str(fixture),observer=observer,observer_sha256=sha(observer),
        execution_environment={key:os.environ.get(key,'') for key in ('ROS_DOMAIN_ID','ROS_AUTOMATIC_DISCOVERY_RANGE',
            'GZ_PARTITION','RMW_IMPLEMENTATION','MOONCAR_IMAGE_DDS_RUNTIME','MOONCAR_TURN_RUNTIME','MOONCAR_DART_PLUGIN_ROOT','LIBGL_ALWAYS_SOFTWARE')},
        physical_parameters=proof,mass_neutral_audit=dict(before=before,after=after,passed=True),
        production_changed=False,hardware_calibrated=False)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(formal_rounds=len(rounds)-1,unexecuted_static_template=1,physics_profile=manifest['physics_profile'],mass_neutral=True,parameters=proof)))
if __name__=='__main__':main()
