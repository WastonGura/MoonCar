"""No physics steps: fresh public model and historical runtime consistency."""
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
from audit_r5_mass import read_urdf,aggregate
from prepare_r5_suspension import compare
from r5_suspension_protocol import validate_campaign,validate_environment

def main():
    root=Path('/diagnosis');out=root/'current-models';out.mkdir(exist_ok=False)
    urdf='/source/ros2_ws/src/mooncar_description/urdf/'
    for name,source,args in [('rigid','mooncar_r5.urdf.xacro',[]),('legacy','mooncar.urdf.xacro',[]),
                             ('suspended','mooncar_r5.urdf.xacro',['suspension_profile:=independent_linear_v1'])]:
        result=subprocess.run(['xacro',urdf+source,*args],check=True,capture_output=True)
        (out/(name+'.urdf')).write_bytes(result.stdout)
    checks={}
    for name,old in [('rigid','r5'),('legacy','legacy')]:
        checks[name+'_byte_identical']=(out/(name+'.urdf')).read_bytes()==(root/'before'/(old+'.urdf')).read_bytes()
    for path in Path(urdf).rglob('*'):
        if path.is_file():
            installed=Path('/work/install/share/mooncar_description/urdf')/path.relative_to(urdf)
            if not installed.exists() or installed.read_bytes()!=path.read_bytes():raise ValueError('Installed public Xacro mismatch')
    checks['installed_public_models_identical']=True
    compare(aggregate(read_urdf(ET.parse(out/'rigid.urdf').getroot())[0]),aggregate(read_urdf(ET.parse(out/'suspended.urdf').getroot())[0]))
    checks['neutral_mass_com_inertia_identical']=True
    checks['public_suspended_model_identical']=(out/'suspended.urdf').read_bytes()==(root/'after/suspended.urdf').read_bytes()
    manifest=json.loads((root/'prepared/manifest.json').read_text())
    checks['frozen_seventeen_round_matrix']=validate_campaign(manifest)['passed']
    checks['isolated_init_environment']=validate_environment(manifest['execution_environment'])['passed']
    expected={
        '/opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so':'cfe9b7c6bf72c204645df81f1be7bc646ffbd342acea52c0684a1ad24848274c',
        '/opt/ros/jazzy/opt/gz_dartsim_vendor/lib/libdart.so.6.13.2':'cb4f008188d1ea13cba9b43470c9d46cf819e8df0e4fdc818fa1e4f81b396426',
        '/opt/mooncar/turn/r5-v2/libode.so.8.0.2':'313dbad175a34e0fd0387792bc922766b5e7727c3883ede5b84322cd055140c7'}
    hashes={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in expected}
    checks['historical_runtime_hashes']=hashes==expected
    production={str(p):float(ET.parse(p).findtext('world/physics/max_step_size')) for p in Path('/source/ros2_ws/src/mooncar_gazebo/worlds').glob('mooncar_*_v2.sdf')}
    checks['production_worlds_keep_1ms']=bool(production) and all(h==.001 for h in production.values())
    result=dict(passed=all(checks.values()),checks=checks,library_hashes=hashes,production_worlds=production,
        physics_profile=manifest['physics_profile'],monitoring_profile=manifest['monitoring_profile'],production_changed=False,hardware_calibrated=False)
    (root/'preflight.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
