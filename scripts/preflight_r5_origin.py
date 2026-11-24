#!/usr/bin/env python3
"""No Gazebo: exact candidate/model, binary dependencies and installed-file gate."""
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
from image_dds_validation import semantic,installed_files

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def run():
    e=Path('/evidence');previous=Path('/source/reports/evidence/terrain-v2/r5-dds-compatible-20261009')
    ready=json.loads((e/'derived-preflight-ready/result.json').read_text())
    if not ready['passed'] or ready['installed']!=installed_files():raise RuntimeError('Installed/model preflight missing')
    tree=ET.parse(e/'r5-before.sdf');original=ET.parse(e/'r5-before.sdf')
    expected={'p_gain':20.,'i_gain':.1,'d_gain':.5,'i_max':1.,'i_min':-1.,'cmd_max':5.,'cmd_min':-5.}
    controllers=tree.findall("./model/plugin[@name='gz::sim::systems::JointPositionController']")
    if len(controllers)!=2 or any(float(c.findtext(k))!=v for c in controllers for k,v in expected.items()):raise RuntimeError('Wrong controller baseline')
    for c in controllers:c.find('d_gain').text='0.0'
    model=e/'r5-d0.sdf';tree.write(model);(e/'r5-d0.urdf').write_bytes((e/'r5-before.urdf').read_bytes())
    if sha(model)!=sha(previous/'campaign/r5-d0.sdf'):raise RuntimeError('D0 candidate differs from previous tested candidate')
    for c in controllers:c.find('d_gain').text='0.5'
    if semantic(tree.getroot())!=semantic(original.getroot()):raise RuntimeError('Model changed beyond D')
    for link in original.findall('./model/link'):
        if '_wheel_link' not in link.get('name',''):continue
        collisions=link.findall('collision')
        if len(collisions)!=1:raise RuntimeError('Ambiguous wheel collision')
        friction=collisions[0].find('surface/friction/ode')
        if friction is None or list(map(float,friction.findtext('fdir1').split()))!=[0.,0.,1.]:raise RuntimeError('Observer friction basis assumption changed')
        if float(friction.findtext('mu'))!=.2 or float(friction.findtext('mu2'))!=.85:raise RuntimeError('Wheel friction changed')
    binary=e/'observer/libmooncar-origin-observer.so'
    ldd=subprocess.check_output(['ldd',str(binary)],text=True)
    if 'not found' in ldd:raise RuntimeError('Observer dependency missing')
    (e/'observer-dependencies.txt').write_text(ldd)
    # Source verification: backend contact contract explicitly defines the
    # first-body force and position in world coordinates.
    contract=(e/'source-audit/GetContacts.hh').read_text()
    if 'in the world frame' not in contract:raise RuntimeError('Unknown contact frame contract')
    source=(e/'source-audit/SimulationFeatures.cc').read_text()
    for token in ('extraContactData.force = _contact.force','_contact.point'):
        if token not in source:raise RuntimeError('Backend contact conversion changed')
    result=dict(passed=True,model_matches_previous=True,only_d_changed=True,installed=installed_files(),
        observer_sha256=sha(binary),candidate_sha256=sha(model),contact_contract='GetContacts: first-body force, point and normal in world frame',
        source_audit={p.name:sha(p) for p in (e/'source-audit').iterdir() if p.is_file()},
        preparation_failures='apt package names, temporary PYTHONPATH replacement, baseline file staging; preserved logs; zero Gazebo attempts')
    (e/'origin-preflight.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='installed'},indent=2))

if __name__=='__main__':run()
