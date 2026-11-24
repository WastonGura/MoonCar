"""Bounded fresh-world fixed-candidate control diagnostic; no publication."""
import argparse
import json
from pathlib import Path
import shutil
import time
import xml.etree.ElementTree as ET
from r5_control_analysis import PROFILES
from check_r5_runtime import run
from check_r5_physics import sha


def select_candidate(output):
    """Use only the three approved profiles and the fixed lexicographic rule."""
    passing=[]
    results={}
    for profile in ('A','B','C'):
        files=sorted(p for p in Path(output).glob('*/result.json') if p.parent.name.endswith(profile+'-torque'))
        if len(files)!=1:
            return dict(status='incomplete_candidate_comparison',selected=None,passed=False,
                        reason=f'{profile} requires exactly one completed comparison result; found {len(files)}')
        results[profile]=files[0]
    for order,profile in enumerate(('A','B','C')):
        path=results[profile]
        result=json.loads(path.read_text())
        if not result['passed']:continue
        axes=[a for h in result['step_observation']['holds'] for a in h['axes']]
        key=(max(s['finished']-s['started'] for s in result['sweep']),
             max(a['velocity_rms'] for a in axes),max(a['peak_to_peak_rad'] for a in axes),order)
        passing.append((key,profile,str(path)))
    if not passing:return dict(status='stopped_no_candidate',selected=None,passed=False,
                               reason='All approved A/B/C profiles failed; no additional tuning or publication')
    key,profile,evidence=min(passing)
    return dict(status='candidate_selected_requires_full_validation',selected=profile,passed=False,
                ranking=list(key),evidence=evidence,parameters=PROFILES[profile])

def check(args):
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    ledger=output/'diagnostic-ledger.json'
    entries=json.loads(ledger.read_text()) if ledger.exists() else []
    if len(entries)>=20:raise RuntimeError('20 diagnostic rounds exhausted')
    entries.append(dict(name=args.name,profile=args.profile,step=args.step,started=time.time()))
    ledger.write_text(json.dumps(entries,indent=2))
    model_root=output/'models';model_root.mkdir(exist_ok=True)
    source=Path(args.model);candidate=model_root/(args.name+'.sdf')
    tree=ET.parse(source);p,i,d,integral,limit=PROFILES[args.profile]
    for controller in tree.findall("./model/plugin[@name='gz::sim::systems::JointPositionController']"):
        for key,value in dict(p_gain=p,i_gain=i,d_gain=d,i_max=integral,i_min=-integral,cmd_max=limit,cmd_min=-limit).items():
            controller.find(key).text=str(value)
    tree.write(candidate)
    shutil.copy2(source.with_suffix('.urdf'),candidate.with_suffix('.urdf'))
    args.model=str(candidate)
    run(args)
    result=json.loads((output/args.name/'result.json').read_text())
    entries[-1].update(finished=time.time(),passed=result['passed'],model_sha256=sha(candidate))
    ledger.write_text(json.dumps(entries,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True);p.add_argument('--name',required=True);p.add_argument('--model',required=True)
    p.add_argument('--profile',choices=PROFILES,required=True);p.add_argument('--plugin',required=True)
    p.add_argument('--observer-plugin',required=True)
    p.add_argument('--package',default='/source/ros2_ws/src/mooncar_gazebo')
    p.add_argument('--kind',choices=('static','gimbal-sweep','drive'),default='gimbal-sweep')
    p.add_argument('--scene',choices=('flat','ramp','curve','survey'),default='flat')
    p.add_argument('--step',type=float,choices=(.001,.0005,.00025),default=.001)
    p.add_argument('--linear',type=float,default=.1);p.add_argument('--angular',type=float,default=0.)
    check(p.parse_args())
