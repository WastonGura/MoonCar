#!/usr/bin/env python3
"""Actual command chain with test driver allocation, fixed verified contact model."""
import argparse,json,shutil,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from r5_axle_allocation import qualify_budget
from r5_slip_probe import slip_commands,contact_relation
from r5_turn_control_probe import outcome


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--name',required=True)
    p.add_argument('--profile',choices=('baseline','candidate','restore'),required=True)
    p.add_argument('--linear',type=float,required=True);p.add_argument('--angular',type=float,required=True)
    a=p.parse_args();enabled=a.profile=='candidate'
    manifest=json.loads(Path('/evidence/allocator-models/manifest.json').read_text())
    controller=Path('/work')/(a.name+'-axle-commands.csv')
    import check_r5_runtime as runtime
    old_prepare=runtime.prepare
    def prepare(args,folder):
        path=old_prepare(args,folder);tree=ET.parse(path)
        element=tree.find("world/model[@name='mooncar']/plugin[@name='mooncar::AxleAllocation']/output")
        if element is None:raise ValueError('Missing test controller')
        element.text=str(controller);tree.write(path);return path
    runtime.prepare=prepare
    import r5_d0_analysis
    old_budget=r5_d0_analysis.wheel_budget
    r5_d0_analysis.wheel_budget=lambda active,linear,angular:qualify_budget(active,controller,linear,angular,enabled,manifest['ratios'])
    import check_r5_origin_round
    sys.argv=['origin','--output',a.output,'--name',a.name,'--model','/evidence/allocator-models/'+a.profile+'.sdf',
              '--step','.001','--linear',str(a.linear),'--angular',str(a.angular)]
    try:code=check_r5_origin_round.main()
    finally:runtime.prepare=old_prepare;r5_d0_analysis.wheel_budget=old_budget
    folder=Path(a.output)/a.name;result=json.loads((folder/'result.json').read_text())
    try:
        if result['passed']:
            data=np.genfromtxt(folder/'physics-steps.csv',delimiter=',',names=True)
            active=data[(data['t']>=result['sim_started'])&(data['t']<=result['sim_ended'])]
            allocation=qualify_budget(active,controller,a.linear,a.angular,enabled,manifest['ratios'])
            end=result['schedule']['stop']['after'];marker=controller.with_suffix('.csv.paused').read_text().strip().split(',')
            flush=int(marker[0])==end['iteration'] and abs(float(marker[1])-end['sim'])<=1e-9
            params=json.loads(Path('/evidence/slip-models/manifest.json').read_text())['parameters']
            commands=slip_commands(data,result,'candidate',params)
            contact=contact_relation(folder,result,'candidate',params)
            metrics=outcome(data,result)
            motion=result['schedule']['motion'];rms={}
            for w in ('front_left','front_right','rear_left','rear_right'):
                rows=np.genfromtxt(folder/'load-analysis'/(w+'-contact-series.csv'),delimiter=',',names=True)
                rows=rows[(rows['t']>=motion['before']['sim']+3)&(rows['t']<=motion['after']['sim']-1)]
                rms[w]=float(np.sqrt(np.mean(rows['slip_body_x']**2)))
            slip_guard=all(np.isfinite(v) and v<=.01 for v in rms.values())
            result['axle_probe']=dict(profile=a.profile,allocation=allocation,controller_flush_verified=flush,
                                     contact=contact,commands=commands,trajectory=metrics,
                                     longitudinal_slip_rms_m_s=rms,longitudinal_slip_guard_passed=slip_guard)
            if not allocation['passed'] or not flush or not commands['passed'] or not contact['contacts_qualified']:
                raise ValueError('Driver/contact/flush qualification failed')
            if a.angular and not contact['backend_effect_verified']:raise ValueError('Fixed contact model not verified')
            if a.angular and not slip_guard:raise ValueError('Excess longitudinal slip; reject driver candidate')
    except Exception as error:result.update(passed=False,axle_probe_error=repr(error));code=1
    for path in (controller,controller.with_suffix('.csv.paused')):
        if path.is_file():shutil.copy2(path,folder/path.name)
    (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n');return code


if __name__=='__main__':raise SystemExit(main())
