#!/usr/bin/env python3
"""Independent source-qualified contact contrast, production unchanged."""
import argparse,json,shutil,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from r5_csv import read_numeric_csv
from r5_axle_allocation import qualify_budget
from r5_axle_robustness import qualify_grid
from r5_contact_source import contact_grid,topic_relation
from r5_slip_probe import slip_commands

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--name',required=True)
    p.add_argument('--profile',choices=('original','slip','restore'),required=True);p.add_argument('--linear',type=float,required=True)
    a=p.parse_args();folder=Path(a.output)/a.name
    contact=Path('/work')/(a.name+'-contact-steps.csv');controller=Path('/work')/(a.name+'-axle-commands.csv')
    import check_r5_runtime as runtime
    import r5_origin_analysis,r5_d0_analysis
    old_prepare,old_run,old_qualify,old_budget=runtime.prepare,runtime.run,r5_origin_analysis.qualify_round,r5_d0_analysis.wheel_budget
    manifest=json.loads(Path('/evidence/allocator-models/manifest.json').read_text())
    source_proof={}
    def prepare(args,out):
        path=old_prepare(args,out);tree=ET.parse(path);model=tree.find("world/model[@name='mooncar']")
        model.find("plugin[@name='mooncar::AxleAllocation']/output").text=str(controller)
        plugin=ET.SubElement(model,'plugin',filename='/evidence/contact-observer/libmooncar-contact-observer.so',name='mooncar::ContactObserver')
        ET.SubElement(plugin,'output').text=str(contact);tree.write(path);return path
    def run(args):args.scene='ramp';args.step=.001;return old_run(args)
    def qualify(out,result):
        previous=old_qualify(out,result)
        physics=read_numeric_csv(Path(out)/'physics-steps.csv',delimiter=',',names=True)
        ids=[int(line.split(',')[1]) for line in Path(str(contact)+'.entities').read_text().splitlines()]
        proof,rows=contact_grid(contact,physics,result['schedule'],ids)
        relation=topic_relation(out,rows);source_proof.update(proof=proof,topic_relation=relation)
        replacement=dict(previous,contacts={w:dict(complete=proof['passed']) for w in proof['wheels']})
        qualified=qualify_grid(replacement,result['schedule'])
        qualified.update(legacy_contact_topic_qualification=previous['contacts'],contact_source=proof,
            gate='Complete read-only ECS snapshots with explicit absent contact, force association and paused drain; no missing data imputation')
        return qualified
    runtime.prepare=prepare;runtime.run=run;r5_origin_analysis.qualify_round=qualify
    r5_d0_analysis.wheel_budget=lambda active,linear,angular:qualify_budget(active,controller,linear,angular,True,manifest['ratios'])
    import check_r5_origin_round
    sys.argv=['origin','--output',a.output,'--name',a.name,'--model','/evidence/contact-models/'+a.profile+'.sdf',
        '--step','.001','--linear',str(a.linear),'--angular','0']
    try:code=check_r5_origin_round.main()
    finally:
        runtime.prepare=old_prepare;runtime.run=old_run;r5_origin_analysis.qualify_round=old_qualify;r5_d0_analysis.wheel_budget=old_budget
    result=json.loads((folder/'result.json').read_text())
    try:
        data=read_numeric_csv(folder/'physics-steps.csv',delimiter=',',names=True)
        cfg=json.loads((folder/'config.json').read_text());world=ET.parse(folder/'world.sdf')
        if cfg['scene']!='ramp' or cfg['step']!=.001 or float(world.findtext('world/physics/max_step_size'))!=.001:
            raise ValueError('Actual scene/step mismatch')
        params=json.loads(Path('/evidence/slip-models/manifest.json').read_text())['parameters']
        commands=slip_commands(data,result,'candidate' if a.profile=='slip' else 'baseline',params)
        allocation=read_numeric_csv(controller,delimiter=',',names=True)
        differences=max(float(np.max(abs(allocation[w+'_native']-allocation[w+'_allocated']))) for w in params)
        marker=controller.with_suffix('.csv.paused').read_text().split(',');boundary=result['schedule']['stop']['after']
        if differences!=0 or np.any(allocation['applied']) or int(marker[0])!=boundary['iteration'] or abs(float(marker[1])-boundary['sim'])>1e-9:
            raise ValueError('Unexpected allocation or unconfirmed controller flush')
        if not commands['passed'] or not source_proof:raise ValueError('Contact intervention not actually observed')
        motion=result['schedule']['motion'];moving=data[(data['t']>=motion['before']['sim']-1e-9)&(data['t']<=motion['after']['sim']+1e-9)]
        xy=np.column_stack([moving[k]-moving[k][0] for k in ('x','y')])
        result['contact_source_probe']=dict(profile=a.profile,source=source_proof,commands=commands,
            allocation_applied_steps=0,allocation_target_difference_rad_s=differences,
            endpoint_xy_m=xy[-1].tolist(),max_lateral_displacement_m=float(np.max(abs(xy[:,1]))),
            scope='Ramp diagnostic only; fixed model contrast, no production or whole-vehicle acceptance')
    except Exception as error:result.update(passed=False,contact_source_error=repr(error));code=1
    for path in (contact,Path(str(contact)+'.paused'),Path(str(contact)+'.entities'),Path(str(contact)+'.status'),controller,controller.with_suffix('.csv.paused')):
        if path.is_file():shutil.copy2(path,folder/path.name)
    (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n');return code

if __name__=='__main__':raise SystemExit(main())
