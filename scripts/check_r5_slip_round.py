#!/usr/bin/env python3
"""Unchanged real command chain, authorized test-only contact model."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from r5_slip_probe import slip_commands, contact_relation
from r5_turn_control_probe import outcome


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--name',required=True)
    p.add_argument('--profile',choices=('baseline','candidate','restore'),required=True)
    p.add_argument('--linear',type=float,required=True);p.add_argument('--angular',type=float,required=True)
    a=p.parse_args()
    import check_r5_origin_round
    sys.argv=['origin','--output',a.output,'--name',a.name,'--model','/evidence/slip-models/'+a.profile+'.sdf',
              '--step','.001','--linear',str(a.linear),'--angular',str(a.angular)]
    code=check_r5_origin_round.main()
    folder=Path(a.output)/a.name
    result=json.loads((folder/'result.json').read_text())
    try:
        if result['passed']:
            data=np.genfromtxt(folder/'physics-steps.csv',delimiter=',',names=True)
            params=json.loads(Path('/evidence/slip-models/manifest.json').read_text())['parameters']
            commands=slip_commands(data,result,a.profile,params)
            contact=contact_relation(folder,result,a.profile,params)
            metrics=outcome(data,result)
            result['slip_probe']=dict(profile=a.profile,commands=commands,contact=contact,trajectory=metrics)
            if not commands['passed'] or not contact['contacts_qualified']:
                raise ValueError('Slip command/contact qualification failed')
            if a.profile=='candidate' and a.angular and not contact['backend_effect_verified']:
                raise ValueError('Backend force/compliance relationship not verified')
            log=(folder/'simulation.log').read_text()
            if 'missing SetShapeFrictionPyramidSlipCompliance' in log or 'No links and surfaces found' in log:
                raise ValueError('WheelSlip unsupported or disabled')
    except Exception as error:
        result.update(passed=False,slip_probe_error=repr(error));code=1
    (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return code


if __name__=='__main__':raise SystemExit(main())
