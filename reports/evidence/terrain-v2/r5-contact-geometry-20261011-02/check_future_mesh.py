"""Native loader reproduces missing normals; fixed generation retains geometry."""
import json,subprocess,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,'/source/scripts')
from prepare_r5_contact_geometry import make_mesh
from audit_r5_ramp_contact import read_obj,digest
p=Path('/probe');out=p/'future-fixtures';out.mkdir(exist_ok=False)
reports=[];binary='/work/mesh-preflight-build/mooncar-contact-mesh-preflight'
for profile,mirror in (('coplanar_mesh',False),('mirrored_mesh',True)):
    old=p/'fixtures'/(profile+'.obj');new=out/(profile+'.obj');make_mesh(new,mirror)
    va,fa=read_obj(old);vb,fb=read_obj(new);assert np.array_equal(va,vb) and np.array_equal(fa,fb)
    a=subprocess.run([binary,str(old)],capture_output=True,text=True);b=subprocess.run([binary,str(new)],capture_output=True,text=True)
    (out/(profile+'-old.json')).write_text(a.stdout);(out/(profile+'-fixed.json')).write_text(b.stdout)
    (out/(profile+'-loader.stderr')).write_text(a.stderr+b.stderr)
    assert a.returncode==1 and json.loads(a.stdout)['passed'] is False
    assert b.returncode==0 and json.loads(b.stdout)['passed'] is True
    reports.append(dict(profile=profile,old_rejected=True,fixed_accepted=True,vertices_unchanged=True,faces_unchanged=True,
        old_sha256=digest(old),fixed_sha256=digest(new)))
r=dict(passed=True,new_simulations=0,old_failure_not_relabelled=True,fixed_loader_only_verified=True,turning_cause_confirmed=False,results=reports)
(out/'result.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
