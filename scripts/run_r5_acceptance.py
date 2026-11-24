#!/usr/bin/env python3
"""Run a frozen R5 acceptance roster; failed rounds are retained and never overwritten."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys

from check_r5_physics import sha


def run(args):
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    cases=[dict(name='01-flat-static',scene='flat',kind='static',step=.001),
           dict(name='02-flat-gimbal',scene='flat',kind='gimbal',step=.001)]
    for scene in ('ramp','curve','survey'):
        for repeat in (1,2):
            cases.append(dict(name=f'{len(cases)+1:02d}-{scene}-{repeat}',scene=scene,kind='drive',step=.001))
    for motion,linear,angular in [('straight',.1,0.),('left',0.,.2),('right',0.,-.2)]:
        for step in (.001,.0005):
            cases.append(dict(name=f'{len(cases)+1:02d}-{motion}-{int(step*1e6)}us',scene='flat',kind='drive',step=step,
                              linear=linear,angular=angular))
    plan=dict(cases=cases,model_sha256=sha(args.model),plugin_sha256=sha(args.plugin),
              runner_sha256=sha(Path(__file__).with_name('check_r5_runtime.py')),per_case_wall_limit_seconds=600,
              source_sha256={p.name:sha(p) for p in [Path(__file__).with_name(n) for n in
                  ('check_r5_runtime.py','r5_runtime_witness.py','experiment_motion.py')]},
              diagnostic_rounds_limit=20,truth_observer_only=True)
    planfile=output/'plan.json'
    if planfile.exists():raise RuntimeError('Acceptance roster already frozen; choose a new evidence directory')
    planfile.write_text(json.dumps(plan,indent=2)+'\n')
    results=[]
    for case in cases:
        folder=output/case['name']
        reused_from=None
        if not folder.exists() and args.reuse_from:
            candidate=Path(args.reuse_from)/case['name']
            if (candidate/'config.json').is_file() and (candidate/'result.json').is_file():
                config=json.loads((candidate/'config.json').read_text())
                if (config.get('source_sha256')==plan['source_sha256'] and
                    config['model_sha256']==plan['model_sha256'] and config['plugin_sha256']==plan['plugin_sha256'] and
                    all(config[k]==v for k,v in case.items()) and (candidate/'gz-input-timed.jsonl').stat().st_size>0):
                    folder=candidate;reused_from=str(candidate)
        if folder.exists():
            if not args.reuse_existing and reused_from is None:raise RuntimeError('Case exists: '+str(folder))
            config=json.loads((folder/'config.json').read_text())
            assert config['model_sha256']==plan['model_sha256'] and config['plugin_sha256']==plan['plugin_sha256']
            assert all(config[k]==v for k,v in case.items())
        else:
            argv=[sys.executable,str(Path(__file__).with_name('check_r5_runtime.py')),
                  '--output',str(output),'--model',args.model,'--plugin',args.plugin,'--package',args.package]
            for key,value in case.items():argv += ['--'+key,str(value)]
            with (output/(case['name']+'.log')).open('w') as log:
                process=subprocess.run(argv,stdout=log,stderr=subprocess.STDOUT,timeout=660)
            if not (folder/'result.json').exists():
                raise RuntimeError(f'Case failed before recording result: {case["name"]}; exit {process.returncode}')
        result=json.loads((folder/'result.json').read_text())
        results.append(dict(case=case,evidence_directory=str(folder),reused_from=reused_from,**result))
        (output/'progress.json').write_text(json.dumps(results,indent=2)+'\n')
        print(json.dumps(dict(name=case['name'],passed=result['passed'],dx_m=result.get('dx_m'),
                              stop_drift_m=result.get('stop_drift_last_second_m'),error=result.get('error'))),flush=True)
    convergence=[]
    for offset,motion in [(8,'straight'),(10,'left'),(12,'right')]:
        a,b=results[offset:offset+2]
        if 'endpoint' not in a or 'endpoint' not in b:
            convergence.append(dict(motion=motion,passed=False,reason='missing endpoint'));continue
        da,db=a['endpoint'],b['endpoint']
        distance=math.dist(da['xyz'][:2],db['xyz'][:2])
        dyaw=da['rpy'][2]-db['rpy'][2];dyaw=abs(math.atan2(math.sin(dyaw),math.cos(dyaw)))
        convergence.append(dict(motion=motion,position_difference_m=distance,yaw_difference_rad=dyaw,
                                passed=a['passed'] and b['passed'] and distance<=.02 and dyaw<=.02))
    summary=dict(passed=all(r['passed'] for r in results) and all(c['passed'] for c in convergence),
                 results=results,convergence=convergence)
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(dict(passed=summary['passed'],convergence=convergence)),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True);parser.add_argument('--model',required=True)
    parser.add_argument('--plugin',required=True);parser.add_argument('--package',default='/source/ros2_ws/src/mooncar_gazebo')
    parser.add_argument('--reuse-existing',action='store_true',help='Explicitly reuse already recorded static/gimbal cases after hash/config checks')
    parser.add_argument('--reuse-from',help='Reuse only cases with identical frozen observer/model/plugin hashes and timed native command evidence')
    run(parser.parse_args())
