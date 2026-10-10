#!/usr/bin/env python3
"""Reuse the qualified chain, explicitly inject only world/step selection."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import xml.etree.ElementTree as ET
from r5_axle_robustness import validate,qualify_grid

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--name',required=True)
    p.add_argument('--scene',required=True);p.add_argument('--step',type=float,required=True)
    p.add_argument('--linear',type=float,required=True);p.add_argument('--angular',type=float,required=True)
    a=p.parse_args();validate(a.scene,a.step)
    import check_r5_runtime as runtime
    import r5_origin_analysis
    old_run=runtime.run;old_qualify=r5_origin_analysis.qualify_round
    def run(args):
        args.scene=a.scene;args.step=a.step
        return old_run(args)
    def qualify(folder,result):return qualify_grid(old_qualify(folder,result),result['schedule'])
    runtime.run=run;r5_origin_analysis.qualify_round=qualify
    import check_r5_axle_round
    sys.argv=['axle','--output',a.output,'--name',a.name,'--profile','candidate',
        '--linear',str(a.linear),'--angular',str(a.angular)]
    try:code=check_r5_axle_round.main()
    finally:runtime.run=old_run;r5_origin_analysis.qualify_round=old_qualify
    folder=Path(a.output)/a.name;result=json.loads((folder/'result.json').read_text())
    try:
        config=json.loads((folder/'config.json').read_text());world=ET.parse(folder/'world.sdf')
        data=np.genfromtxt(folder/'physics-steps.csv',delimiter=',',names=True)
        actual=bool(config['scene']==a.scene and config['step']==a.step and
            float(world.findtext('world/physics/max_step_size'))==a.step and
            len(data)>1 and np.isfinite(data['dt']).all() and (abs(data['dt']-a.step)<=1e-15).all())
        result['robustness_probe']=dict(scene=a.scene,step=a.step,actual_config_and_step_verified=actual,
            scope='Test candidate only; not production or whole-vehicle acceptance',
            fixture_adapter='Explicit scene/step selection; no observed record normalization')
        if not actual:raise ValueError('Actual world/config/physical step does not match frozen selection')
    except Exception as error:result.update(passed=False,robustness_error=repr(error));code=1
    (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n');return code

if __name__=='__main__':raise SystemExit(main())
