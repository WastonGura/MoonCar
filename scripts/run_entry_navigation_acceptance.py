"""Fresh, bounded entry/navigation validation; preserve failure and stop, no retries."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
def write(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def analyze(folder,profile):
    state=json.loads((folder/'autonomy.json').read_text())
    report=json.loads((folder/'report.json').read_text())
    moves=[r for r in state['history'] if r['kind']=='advance']
    checks=dict(protocol_passed=report['passed'],navigation_present=bool(moves),
        navigation_succeeded=any(r.get('success') for r in moves),
        no_stall_or_104=all('持续未接近' not in r.get('motion_error','') and
            (r.get('motion_stop') or {}).get('nav2_error_code')!=104 for r in moves))
    zone=[];clocks=[];joint_stamps={};tf_stamps={}
    for line in (folder/'witness.jsonl').open():
        row=json.loads(line)
        if row['kind']=='costmap' and row['topic']=='/global_costmap/costmap':
            assert row['frame']=='map'
            x=int((-1.5-row['origin'][0])/row['resolution']);y=int((-.8-row['origin'][1])/row['resolution'])
            if 0<=x<row['width'] and 0<=y<row['height']:zone.append(row['data'][y*row['width']+x])
        if row['kind']=='raw_clock':clocks.append(row['stamp_ns'])
        if row['kind']=='full_joints':joint_stamps[row['stamp_ns']]=dict(zip(row['names'],row['positions']))
        if row['kind']=='tf':
            for item in row['transforms']:tf_stamps.setdefault(item['stamp_ns'],{})[item['child']]=item
    checks['global_keepout_preserved']=bool(zone) and all(v==100 for v in zone)
    metrics=dict(moves=[{k:r.get(k) for k in ('x','y','success','motion_error')} for r in moves],
                 keepout_samples=len(zone),keepout_values=sorted(set(zone)))
    if profile=='r5_support_v1':
        for line in (folder/'native-clock.jsonl').open():
            message=json.loads(line)
            stamp=message['sim']
            clocks.append(int(stamp.get('sec',0))*10**9+int(stamp.get('nsec',0)))
        robot=ET.parse(folder/'actual-robot.urdf')
        joints=robot.findall("joint[@type='prismatic']")
        matched=0;error=0.
        for stamp,positions in joint_stamps.items():
            transforms=tf_stamps.get(stamp,{})
            if not joints or not all(j.get('name') in positions and j.find('child').get('link') in transforms for j in joints):continue
            matched+=1
            for j in joints:
                item=transforms[j.find('child').get('link')]
                target=[float(a)+positions[j.get('name')]*float(b) for a,b in zip(j.find('origin').get('xyz').split(),j.find('axis').get('xyz').split())]
                error=max(error,max(abs(a-b) for a,b in zip(target,item['xyz'])))
                checks['tf_parent_correct']=checks.get('tf_parent_correct',True) and item['parent']==j.find('parent').get('link')
        receipts=list((folder/'sessions').glob('*/support-runtime/loaded.json'))
        checks['actual_library_receipt']=len(receipts)==1
        if receipts:
            receipt=json.loads(receipts[0].read_text())
            checks['actual_world_precision']=receipt['main_step_s']==.00025
        deltas=[b-a for a,b in zip(clocks,clocks[1:]) if b>a]
        checks.update(four_passive_joints=len(joints)==4,native_clock_precision=bool(deltas) and min(deltas)==250000,
                      suspension_tf_consistent=matched>=30 and error<=1e-9)
        metrics.update(matched_joint_tf_samples=matched,max_tf_error_m=error,minimum_raw_clock_delta_ns=min(deltas) if deltas else None)
    result=dict(passed=all(checks.values()),profile=profile,checks=checks,metrics=metrics)
    write(folder/'entry-navigation-result.json',result)
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    profiles=['legacy','r5_support_v1','r5_support_v1']
    write(a.output/'protocol.json',dict(maximum_physical_trials=3,profiles=profiles,budget_sim_s=90,
        injected_decisions=False,automatic_retry=False,stop_on_failure=True))
    files=[f for base in ('ros2_ws/src','scripts','containers','web') for f in (ROOT/base).rglob('*')
        if f.is_file() and '__pycache__' not in f.parts and f.suffix in ('.py','.cpp','.cc','.hh','.hpp','.xml','.xacro','.yaml','.json','.sdf','.js','.html','.txt')]
    frozen={str(f.relative_to(ROOT)):digest(f) for f in files}
    write(a.output/'freeze.json',frozen)
    results=[]
    for index,profile in enumerate(profiles,1):
        folder=a.output/f'{index:02d}-{profile}'
        with (a.output/f'{index:02d}.log').open('w') as log:
            child=subprocess.run([sys.executable,'-B',str(ROOT/'scripts/check_navigation_diagnosis.py'),str(folder),'--profile',profile],stdout=log,stderr=subprocess.STDOUT)
        if child.returncode!=0:
            results.append(dict(passed=False,return_code=child.returncode));break
        result=analyze(folder,profile);results.append(result)
        if not result['passed']:break
        if any(not (ROOT/f).is_file() or digest(ROOT/f)!=s for f,s in frozen.items()):
            results.append(dict(passed=False,error='Frozen source changed'));break
    passed=len(results)==3 and all(r['passed'] for r in results)
    write(a.output/'result.json',dict(passed=passed,trials=results,attempts=len(results)))
    return 0 if passed else 1

if __name__=='__main__':raise SystemExit(main())
