"""Archive only this offline audit, retaining all earlier failures."""
import concurrent.futures, hashlib, json, shutil, sys, urllib.request
from pathlib import Path
root=Path(sys.argv[1]);p=Path(sys.argv[2])
def digest(file):return hashlib.sha256(file.read_bytes()).hexdigest()
urls={
 'EntityManagementFeatures.cc':'https://raw.githubusercontent.com/gazebosim/gz-physics/gz-physics7_7.8.0/dartsim/src/EntityManagementFeatures.cc',
 'SdfEntityCreator.cc':'https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8_8.15.0/src/SdfEntityCreator.cc',
 'OdeCollisionDetector.cpp':'https://raw.githubusercontent.com/dartsim/dart/v6.13.2/dart/collision/ode/OdeCollisionDetector.cpp'}
def download(pair):
    name,url=pair;target=p/'provenance'/name
    if target.is_file():
        recorded=json.loads((p/'provenance/source-manifest.json').read_text())[name]
        if recorded.get('url')!=url or recorded.get('sha256')!=digest(target):raise ValueError('Archived source changed')
        return name,recorded
    try:
        content=urllib.request.urlopen(url,timeout=15).read()
        with target.open('xb') as stream:stream.write(content)
        return name,dict(url=url,sha256=digest(target),bytes=len(content),retrieved=True)
    except Exception as error:return name,dict(url=url,retrieved=False,error=repr(error))
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor: sources=dict(executor.map(download,urls.items()))
(p/'provenance/source-manifest.json').write_text(json.dumps(sources,indent=2)+'\n')
paths=['scripts/audit_r5_ramp_contact.py','scripts/prepare_r5_contact_geometry.py','ros2_ws/src/mooncar_gazebo/tests/test_ramp_contact_audit.py']
snapshot=p/'source-snapshot';snapshot.mkdir(exist_ok=True)
for rel in paths:
    target=snapshot/rel;target.parent.mkdir(parents=True,exist_ok=True)
    if target.exists():
        if digest(target)!=digest(root/rel):raise ValueError('Source snapshot changed')
    else:shutil.copyfile(root/rel,target)
attempts=[dict(directory='r5-ramp-contact-audit-20261011',status='failed_before_analysis',reason='Initial offline plate helper rejected archived link Z offset',new_simulations=0),
          dict(directory='r5-ramp-contact-audit-20261011-02',status='completed_offline',new_simulations=0),
          dict(directory=p.name,status='completed_offline_with_seam_classification',new_simulations=0)]
(p/'attempts.json').write_text(json.dumps(attempts,indent=2)+'\n')
records={}
for path in sorted(p.rglob('*')):
    if 'offline-work' in path.relative_to(p).parts or path.name=='archive-manifest.json' or not path.is_file():continue
    records[str(path.relative_to(p))]=dict(sha256=digest(path),bytes=path.stat().st_size)
(p/'archive-manifest.json').write_text(json.dumps(dict(files=records,file_count=len(records),bytes=sum(v['bytes'] for v in records.values()),
       new_simulations=0,excluded=['offline-work: disposable symlink install/build products; logs retained separately'],
       old_campaign_reverified=True,mechanism_confirmed=False,production_changed=False),indent=2)+'\n')
print(json.dumps(dict(files=len(records),bytes=sum(v['bytes'] for v in records.values()),source_fetches=sources)))
