#!/usr/bin/env python3
"""Real limited command chain, contact actuation independently observed."""
import argparse,json,shutil,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from r5_axle_allocation import qualify_budget
from r5_axle_robustness import validate,qualify_grid
from r5_contact_source import contact_grid,topic_relation
from r5_turn_slip_control import qualify
from r5_turn_control_probe import outcome

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--name',required=True)
    p.add_argument('--profile',choices=('always','pure_turn','restore'),required=True)
    p.add_argument('--scene',required=True);p.add_argument('--step',type=float,required=True)
    p.add_argument('--linear',type=float,required=True);p.add_argument('--angular',type=float,required=True)
    a=p.parse_args();validate(a.scene,a.step);folder=Path(a.output)/a.name
    contact=Path('/work')/(a.name+'-contact-steps.csv');controller=Path('/work')/(a.name+'-axle-commands.csv');gate=Path('/work')/(a.name+'-slip-control.csv')
    import check_r5_runtime as runtime
    import r5_origin_analysis,r5_d0_analysis
    old_prepare,old_run,old_qualify,old_budget=runtime.prepare,runtime.run,r5_origin_analysis.qualify_round,r5_d0_analysis.wheel_budget
    manifest=json.loads(Path('/evidence/allocator-models/manifest.json').read_text());source_proof={}
    def prepare(args,out):
        path=old_prepare(args,out);tree=ET.parse(path);model=tree.find("world/model[@name='mooncar']")
        model.find("plugin[@name='mooncar::AxleAllocation']/output").text=str(controller)
        model.find("plugin[@name='mooncar::TurnSlipControl']/output").text=str(gate)
        plugin=ET.SubElement(model,'plugin',filename='/evidence/contact-observer/libmooncar-contact-observer.so',name='mooncar::ContactObserver')
        ET.SubElement(plugin,'output').text=str(contact);tree.write(path);return path
    def run(args):args.scene=a.scene;args.step=a.step;return old_run(args)
    def qualified(out,result):
        previous=old_qualify(out,result);physics=np.genfromtxt(Path(out)/'physics-steps.csv',delimiter=',',names=True)
        ids=[int(line.split(',')[1]) for line in Path(str(contact)+'.entities').read_text().splitlines()]
        proof,rows=contact_grid(contact,physics,result['schedule'],ids)
        relation=topic_relation(out,rows);source_proof.update(proof=proof,topic_relation=relation)
        replacement=dict(previous,contacts={w:dict(complete=proof['passed']) for w in proof['wheels']})
        result=qualify_grid(replacement,result['schedule']);result.update(legacy_contact_topic_qualification=previous['contacts'],contact_source=proof)
        return result
    runtime.prepare=prepare;runtime.run=run;r5_origin_analysis.qualify_round=qualified
    r5_d0_analysis.wheel_budget=lambda active,linear,angular:qualify_budget(active,controller,linear,angular,True,manifest['ratios'])
    import check_r5_origin_round
    sys.argv=['origin','--output',a.output,'--name',a.name,'--model','/evidence/turn-models/'+a.profile+'.sdf',
        '--step',str(a.step),'--linear',str(a.linear),'--angular',str(a.angular)]
    try:code=check_r5_origin_round.main()
    finally:runtime.prepare=old_prepare;runtime.run=old_run;r5_origin_analysis.qualify_round=old_qualify;r5_d0_analysis.wheel_budget=old_budget
    result=json.loads((folder/'result.json').read_text())
    try:
        data=np.genfromtxt(folder/'physics-steps.csv',delimiter=',',names=True);cfg=json.loads((folder/'config.json').read_text());world=ET.parse(folder/'world.sdf')
        if cfg['scene']!=a.scene or cfg['step']!=a.step or float(world.findtext('world/physics/max_step_size'))!=a.step:raise ValueError('Actual scene/step mismatch')
        params=json.loads(Path('/evidence/slip-models/manifest.json').read_text())['parameters']
        proof=qualify(data,result,gate,'pure_turn' if a.profile=='pure_turn' else 'always',params)
        active=data[(data['t']>=result['sim_started'])&(data['t']<=result['sim_ended'])]
        allocation=qualify_budget(active,controller,a.linear,a.angular,True,manifest['ratios'])
        marker=controller.with_suffix('.csv.paused').read_text().split(',');boundary=result['schedule']['stop']['after']
        if not allocation['passed'] or int(marker[0])!=boundary['iteration'] or abs(float(marker[1])-boundary['sim'])>1e-9:raise ValueError('Allocator qualification/flush mismatch')
        metrics=outcome(data,result);motion=result['schedule']['motion'];moving=data[(data['t']>=motion['before']['sim']-1e-9)&(data['t']<=motion['after']['sim']+1e-9)]
        xy=np.column_stack([moving[k]-moving[k][0] for k in ('x','y')])
        result['turn_slip_probe']=dict(profile=a.profile,scene=a.scene,step=a.step,gate=proof,source=source_proof,allocation=allocation,
            trajectory=metrics,endpoint_xy_m=xy[-1].tolist(),max_lateral_m=float(np.max(abs(xy[:,1]))),production_changed=False)
        if a.profile=='pure_turn' and a.angular and metrics['maximum_displacement_m']>.01:raise ValueError('Existing 1 cm turn screen failed')
        if a.profile=='pure_turn' and a.linear and abs(xy[-1,1])>.01:raise ValueError('Frozen straight lateral side-effect screen failed')
        if not source_proof:raise ValueError('Missing complete contact-source proof')
    except Exception as error:result.update(passed=False,turn_slip_error=repr(error));code=1
    for path in (contact,Path(str(contact)+'.paused'),Path(str(contact)+'.entities'),Path(str(contact)+'.status'),controller,controller.with_suffix('.csv.paused'),gate,Path(str(gate)+'.paused')):
        if path.is_file():shutil.copy2(path,folder/path.name)
    (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n');return code

if __name__=='__main__':raise SystemExit(main())
