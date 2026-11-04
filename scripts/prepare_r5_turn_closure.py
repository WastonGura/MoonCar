"""Expand actual formal profiles and create observation-only acceptance worlds."""
import argparse
from argparse import Namespace
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def canonical(e):return (e.tag,sorted(e.attrib.items()),(e.text or '').strip(),tuple(canonical(c) for c in e))

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
    out=Path(a.output);out.mkdir(exist_ok=False)
    package=Path('/work/install/share/mooncar_gazebo')
    launch=package/'launch';sys.path.insert(0,str(launch))
    from turn_guard import verify,augment_urdf
    root='/opt/mooncar/turn/r5-v2';manifest=verify(root)
    if manifest.get('allocation_profile') == 'independent_laser_feedback_v1':
        raise RuntimeError('Fixed-allocation closure fixture retired; qualify independent feedback with check_r5_adaptive_turn_round.py first')
    description=Path('/work/install/share/mooncar_description/urdf')
    for model in ('mooncar.urdf.xacro','mooncar_r5.urdf.xacro'):
        original=subprocess.check_output(['xacro',str(description/model),'use_sim_time:=true'],text=True)
        legacy=subprocess.check_output([sys.executable,str(launch/'turn_model.py'),'--model',str(description/model),
            '--profile','legacy','--root',root],text=True)
        if original!=legacy:raise RuntimeError('Legacy xacro output changed')
        (out/(model+'.legacy.urdf')).write_text(original)
        if model.startswith('mooncar_r5'):
            changed=augment_urdf(original,root)
            clean=ET.fromstring(changed);clean.remove(clean[-1])
            for controller in clean.findall("gazebo/plugin[@name='gz::sim::systems::JointPositionController']"):
                controller.find('d_gain').text='0.5'
            if canonical(clean)!=canonical(ET.fromstring(original)):raise RuntimeError('Vehicle/gimbal altered')
            path=out/'formal.urdf';path.write_text(changed)
            (out/'formal.sdf').write_text(subprocess.check_output(['gz','sdf','-p',str(path)],text=True))
    from check_r5_physics import prepare
    for scene in ('ramp','curve','survey'):
        for step in (.001,.0005):
            folder=out/(scene+'-'+str(step));folder.mkdir()
            world=prepare(Namespace(world=package/f'worlds/mooncar_{scene}_v2.sdf',package=package,
                model=out/'formal.sdf',plugin=manifest['dart_path'],backend='dart',step=step,
                disable_gimbal=False,disable_drive=False),folder)
            tree=ET.parse(world);model=tree.find("world/model[@name='mooncar']")
            for name in ('mooncar::AxleAllocation','mooncar::TurnSlipControl'):
                ET.SubElement(model.find(f"plugin[@name='{name}']"),'output').text='UNSET'
            # Read-only observers, also used by earlier qualified campaigns.
            for name,library in (('mooncar::StepObserver','/evidence/observer/libmooncar-origin-observer.so'),
                                 ('mooncar::ContactObserver','/evidence/contact-observer/libmooncar-contact-observer.so')):
                plugin=ET.SubElement(model,'plugin',name=name,filename=library)
                ET.SubElement(plugin,'output').text='UNSET'
            tree.write(world)
    # Isolated fixture derived from preserved diagnostic entry. New world determines
    # scene and step; no legacy evidence file is changed or resumed.
    text=Path('/evidence/check_r5_v2_offset_round.py').read_text()
    text=text.replace('path = original_prepare(run_args, out)',
        "run_args.model = '"+str(out/'formal.sdf')+"'\n        path = original_prepare(run_args, out)")
    text=text.replace("'--scene', 'ramp'", "'--scene', world_src.parent.name.split('-')[0]")
    text=text.replace("'--step', '0.001'", "'--step', ET.parse(world_src).findtext('world/physics/max_step_size')")
    (out/'round_fixture.py').write_text(text)
    (out/'preflight.json').write_text(json.dumps(dict(legacy_byte_identical=True,
        physical_and_sensor_preserved=True,gimbal_single_change='D 0.5 -> 0',formal_d_gain=[p.findtext('d_gain') for p in ET.parse(out/'formal.sdf').findall("model/plugin[@name='gz::sim::systems::JointPositionController']")],
        runtime=manifest,files={str(x):sha(x) for x in out.rglob('*') if x.is_file()}),indent=2)+'\n')
    print('Actual old/R5 model expansion and SDF preservation passed')

if __name__=='__main__':main()
