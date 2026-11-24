"""Actual runtime and fixture checks without starting Gazebo."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from audit_r5_ramp_contact import digest,plate_top,read_obj
from r5_contact_geometry import qualify
from run_r5_contact_geometry import signature

def main():
    reference=json.loads(Path('/evidence/campaign/freeze.json').read_text())
    actual=signature();checks={}
    for key in ('resources','libraries','model','runtime','allocator_models','allocator_library','turn_models','turn_library','contact_library'):
        checks['unchanged_'+key]=actual[key]==reference[key]
    for name in ('03-pure_turn-flat-1000us-left-1','14-pure_turn-ramp-1000us-left'):
        result=json.loads(Path('/evidence/campaign',name,'result.json').read_text())
        proof=qualify(result,0 if result['passed'] else 1)
        checks['qualification_replay_'+name]=proof['eligible']
        if name.startswith('14'):checks['old_failure_not_relabelled']=not proof['original_passed'] and not proof['geometric_screen_passed']
    worlds={n:ET.parse('/probe/fixtures/'+n+'.sdf').getroot() for n in ('plate','coplanar_mesh','mirrored_mesh')}
    plate=worlds['plate'].find("world/model[@name='plate']")
    checks['original_plate_top']=abs(plate_top(plate))<1e-12
    model=ET.tostring(worlds['plate'].find("world/model[@name='mooncar']"))
    for name in ('coplanar_mesh','mirrored_mesh'):
        checks['identical_vehicle_'+name]=ET.tostring(worlds[name].find("world/model[@name='mooncar']"))==model
        v,f=read_obj('/probe/fixtures/'+name+'.obj')
        checks['coplanar_height_'+name]=bool((v[:,2]==.05).all())
    manifest=json.loads(Path('/evidence/test-startup/image_dds_guard.manifest.json').read_text())
    checks['startup_guard_hash']=manifest['test_sha256']==digest('/evidence/test-startup/image_dds_guard.py')
    checks['no_production_change']=not manifest['production_changed'] and not manifest['timeout_changed']
    result=dict(passed=all(checks.values()),checks=checks,new_simulations=0,production_changed=False)
    Path('/probe/preflight.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    if not result['passed']:raise SystemExit(1)

if __name__=='__main__':main()
