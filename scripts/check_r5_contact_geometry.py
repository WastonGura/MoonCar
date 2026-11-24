"""Reuse the frozen command/capture chain, changing only approved ground geometry."""
import argparse,json,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from r5_contact_geometry import PROFILES,qualify,metadata
from audit_r5_ramp_contact import plate_top

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--name',required=True)
    p.add_argument('--ground-profile',choices=PROFILES,required=True);p.add_argument('--angular',type=float,choices=(.2,-.2),required=True)
    a=p.parse_args();folder=Path(a.output)/a.name
    profile='plate' if a.ground_profile=='restore_plate' else a.ground_profile
    import check_r5_runtime as runtime
    original=runtime.prepare
    def prepare(args,out):
        # Execute the old preparation for its unchanged validation, then use the
        # archived complete world with only observer outputs reattached below.
        path=original(args,out)
        tree=ET.parse('/probe/fixtures/'+profile+'.sdf');world=tree.find('world')
        model=world.find("model[@name='mooncar']")
        for plugin in list(model.findall('plugin')):
            if plugin.get('name') in ('mooncar::StepObserver','mooncar::ContactObserver'):model.remove(plugin)
        plate=world.find("model[@name='plate']");link=plate.find('link')
        for visual in list(link.findall('visual')):link.remove(visual)
        plugin=ET.SubElement(world,'plugin',filename='/probe/metadata/libmooncar-contact-metadata.so',name='mooncar::ContactMetadata')
        ET.SubElement(plugin,'output').text=str(folder/'collision-metadata.csv')
        tree.write(path)
        if profile=='plate' and abs(plate_top(plate))>1e-12:raise ValueError('Ground support mismatch')
        (Path(out)/'ground-profile.json').write_text(json.dumps(dict(profile=a.ground_profile,production_changed=False))+'\n')
        return path
    runtime.prepare=prepare
    oldargv=sys.argv
    sys.argv=['round','--output',a.output,'--name',a.name,'--profile','pure_turn','--scene','flat','--step','.001','--linear','0','--angular',str(a.angular)]
    try:
        import check_r5_turn_gate_validation
        code=check_r5_turn_gate_validation.main()
    finally:runtime.prepare=original;sys.argv=oldargv
    r=json.loads((folder/'result.json').read_text());proof=qualify(r,code)
    try:
        rows=np.genfromtxt(folder/(a.name+'-contact-steps.csv'),delimiter=',',names=True)
        other=np.unique(rows[rows['state']==1]['other_collision']).astype(int)
        proof['metadata']=metadata(folder/'collision-metadata.csv',other)
    except Exception as error:proof.update(eligible=False,metadata_error=repr(error))
    proof['profile']=a.ground_profile
    (folder/'diagnostic-result.json').write_text(json.dumps(proof,indent=2)+'\n')
    return 0 if proof['eligible'] else 1

if __name__=='__main__':raise SystemExit(main())
