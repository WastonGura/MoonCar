"""Fresh bounded release matrix; no resumed signatures or historical filler."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

ROUNDS=[(f'{i*3+j+1:02d}-{scene}-{kind}',scene,.001,linear,angular)
    for i,scene in enumerate(('ramp','curve','survey'))
    for j,(kind,linear,angular) in enumerate((('left',0.,.2),('right',0.,-.2),('straight',.1,0.)))]
ROUNDS += [('10-ramp-left-repeat','ramp',.001,0.,.2),('11-ramp-left-halfstep','ramp',.0005,0.,.2),
           ('12-ramp-right-halfstep','ramp',.0005,0.,-.2)]

def geometry(probe,linear,angular):
    t=probe.get('trajectory',{});e=probe.get('endpoint_xy_m',[])
    if len(e)!=2:return False
    if linear:
        return 1<=e[0]<=2.5 and probe.get('max_lateral_m',1)<=.01
    turn=t.get('unwrapped_turn_rad',0)
    return t.get('maximum_displacement_m',1)<=.01 and 3<=abs(turn)<=4.5 and turn*angular>0

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--prepared',required=True)
    p.add_argument('--freeze-only',action='store_true');p.add_argument('--execute',action='store_true');a=p.parse_args()
    if a.freeze_only==a.execute:raise ValueError('Choose exactly one action')
    out=Path(a.output);out.mkdir(exist_ok=True);freeze=out/'freeze.json'
    if a.freeze_only:
        if freeze.exists():raise FileExistsError(freeze)
        files=[]
        for root in ('/source/scripts','/source/containers/r5_turn_runtime','/source/ros2_ws/src',
                     '/work/install',a.prepared,'/opt/mooncar/turn/r5-v2'):
            files += [p for p in Path(root).rglob('*') if p.is_file() and '__pycache__' not in str(p)
                and p.suffix in ('.py','.json','.yaml','.xacro','.sdf','.urdf','.so','.2','.obj','.png','.npz','.cc','.hh','.txt')]
        frozen=dict(files={str(p):sha(p) for p in sorted(set(files))},rounds=ROUNDS,maximum_attempts=12,
            maximum_wall_s=600,thresholds=dict(turn_maximum_xy_m=.01,minimum_turn_rad=3.,maximum_turn_rad=4.5,
            repeat_position_difference_m=.01,repeat_yaw_difference_rad=.01,step_position_difference_m=.01,step_yaw_difference_rad=.01),
            selected_profile_gimbal_d0=True,repository_default_published=False,
            physical_acceptance='scoped simulation steering/basic-running only; no hardware claim')
        freeze.write_text(json.dumps(frozen,indent=2)+'\n');print('Frozen',len(frozen['files']));return 0
    ledger=out/'campaign.json'
    if ledger.exists():raise FileExistsError('No resume or overwrite')
    frozen=json.loads(freeze.read_text());status=dict(status='running',attempts=[],maximum_attempts=12,freeze_sha256=sha(freeze),default_published=False)
    def save():ledger.write_text(json.dumps(status,indent=2)+'\n')
    save()
    for name,scene,step,linear,angular in ROUNDS:
        changed=[p for p,h in frozen['files'].items() if not Path(p).is_file() or sha(p)!=h]
        if changed:status.update(status='stopped',reason='Signature changed',changed=changed);save();return 1
        run_name=out.name+'-'+name
        row=dict(name=name,run_name=run_name,scene=scene,step=step,linear=linear,angular=angular,started_wall=time.time())
        status['attempts'].append(row);save()
        cmd=[sys.executable,'-B','/source/scripts/check_r5_coplanar_round.py','--output',str(out),'--name',run_name,
            '--world',str(Path(a.prepared)/(scene+'-'+str(step))/'world.sdf'),'--ode-profile','candidate',
            '--candidate-runtime','/opt/mooncar/turn/r5-v2','--manifest','/opt/mooncar/turn/r5-v2/manifest.json',
            '--round-fixture',str(Path(a.prepared)/'round_fixture.py'),'--linear',str(linear),'--angular',str(angular)]
        started=time.monotonic()
        try:
            with (out/(name+'.log')).open('w') as log:r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,timeout=600)
            row['exit_code']=r.returncode
        except Exception as error:row.update(exit_code=None,error=repr(error))
        row['wall_seconds']=time.monotonic()-started
        for key,path in (('loader',out/(run_name+'-loader.json')),('result',out/run_name/'result.json')):
            if path.is_file():
                value=json.loads(path.read_text());row[key]=value if key=='loader' else {
                    k:value.get(k) for k in ('passed','error','production_error','origin_error','turn_slip_error','parked','max_tilt_rad',
                    'stop_drift_last_second_m','turn_slip_probe','performance','data_complete','stop_zero')}
        probe=(row.get('result') or {}).get('turn_slip_probe') or {}
        row['geometry_passed']=geometry(probe,linear,angular)
        row['passed']=bool(row.get('exit_code')==0 and (row.get('loader') or {}).get('passed') and row['geometry_passed'])
        save();print(json.dumps({k:row[k] for k in ('name','exit_code','wall_seconds','passed')}),flush=True)
        if not row['passed']:
            status.update(status='stopped',reason='Input/data/runtime/steering gate failed',failed_round=name);save();return 1
    comparisons=[]
    by_name={r['name']:r for r in status['attempts']}
    import math
    for x,y,kind in (('01-ramp-left','10-ramp-left-repeat','repeat'),('01-ramp-left','11-ramp-left-halfstep','step'),('02-ramp-right','12-ramp-right-halfstep','step')):
        first,second=[by_name[n]['result']['turn_slip_probe'] for n in (x,y)]
        distance=math.dist(first['endpoint_xy_m'],second['endpoint_xy_m'])
        angle=abs(first['trajectory']['unwrapped_turn_rad']-second['trajectory']['unwrapped_turn_rad'])
        comparisons.append(dict(first=x,second=y,kind=kind,position_difference_m=distance,yaw_difference_rad=angle,passed=distance<=.01 and angle<=.01))
    status.update(comparisons=comparisons,status='passed' if all(c['passed'] for c in comparisons) else 'stopped',
                  reason='All gates passed' if all(c['passed'] for c in comparisons) else 'Repeat/step gate failed')
    save();return 0 if status['status']=='passed' else 1

if __name__=='__main__':raise SystemExit(main())
