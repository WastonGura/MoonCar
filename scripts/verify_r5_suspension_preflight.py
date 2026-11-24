"""Verify source/install/default exports and neutral experimental model integrity."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from audit_r5_mass import aggregate,read_urdf
from prepare_r5_suspension import mass,compare
from r5_suspension import WHEELS
from r5_suspension_protocol import validate_campaign


def main():
    current=Path('/diagnosis/current-models');before=Path('/diagnosis/before');checks={}
    for current_name,old_name in [('rigid','r5'),('legacy','legacy')]:
        checks[current_name+'_byte_identical']=(current/(current_name+'.urdf')).read_bytes()==(before/(old_name+'.urdf')).read_bytes()
    source=Path('/source/ros2_ws/src/mooncar_description/urdf');installed=Path('/work/install/share/mooncar_description/urdf')
    checks['installed_public_models_identical']=all((installed/p.relative_to(source)).read_bytes()==p.read_bytes() for p in source.rglob('*') if p.is_file())
    manifest=json.loads(Path('/diagnosis/prepared/manifest.json').read_text());validate_campaign(manifest)
    old=aggregate(read_urdf(ET.parse(before/'r5.urdf').getroot())[0])
    candidate=aggregate(read_urdf(ET.parse(current/'suspended.urdf').getroot())[0])
    generated=ET.parse(current/'suspended.sdf').find('model');compare(old,candidate);compare(old,mass(generated))
    checks['neutral_mass_com_inertia_identical']=True
    for wheel in WHEELS:
        joint=generated.find(f"joint[@name='{wheel}_wheel_suspension_joint']")
        parameter=manifest['physical_parameters']['parameters'][wheel]
        checks[wheel+'_public_sdf_parameters_match']=all(abs(float(joint.findtext('axis/'+field))-parameter[key])<1e-10
            for key,field in [('k_N_m','dynamics/spring_stiffness'),('c_N_s_m','dynamics/damping'),
                              ('reference_m','dynamics/spring_reference'),('lower_m','limit/lower'),('upper_m','limit/upper')])
    result=dict(passed=all(checks.values()),checks=checks,formal_declared_rounds=17,
        source_model_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in current.glob('*')},
        mass_neutral=candidate,production_changed=False,hardware_calibrated=False)
    path=Path('/diagnosis/preflight.json')
    if path.exists():raise FileExistsError('Refuse preflight overwrite')
    path.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
