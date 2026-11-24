"""Run after all three continuous runs and the controlled fault test are captured."""
import hashlib,json,sys
from pathlib import Path
import numpy as np

root=Path(__file__).resolve().parent
sys.path.insert(0,str(root.parents[3]/'scripts'))
from survey_evaluation import metrics

folders=sorted(p for p in root.glob('autonomy-*') if p.is_dir())
assert len(folders)==3,'Three completed continuous runs required'
origins=set()
for folder in folders:
    def read(name):return json.loads((folder/name).read_text(encoding='utf-8'))
    manifest,summary,mission=read('manifest.json'),read('summary.json'),read('mission.json')
    assert manifest['status']=='captured' and manifest['report_status']=='ready' and not manifest['capture_errors']
    for name,digest in read('hashes.json').items():
        assert hashlib.sha256((folder/name).read_bytes()).hexdigest()==digest,(folder.name,name)
    assert summary['budget_seconds']==180 and summary['status'] in ('budget_reached','no_candidates')
    assert 150<=summary['elapsed_seconds']<=181,summary
    assert summary['new_frames']>0 and summary['path_m']>1 and summary['quality_gain_m2']>0
    assert any(h['success'] and h['kind']=='advance' for h in mission['history'])
    assert any(h['success'] and h['kind']=='turn' for h in mission['history'])
    assert summary['scene_distinct_images']>1 and not summary['scene_warning']
    rows=[json.loads(line) for line in (folder/'telemetry.jsonl').read_text().splitlines()]
    for label,row in [('start',rows[0]),('end',rows[-1])]:
        score=metrics(read(f'quality-{label}.json'),mission['origin'],mission['radius'])
        assert abs(score['quality_area_m2']-row['quality_area_m2'])<1e-8
    with np.load(folder/'observations.npz',allow_pickle=False) as data:
        assert data['stamps'][-1]>=rows[-1]['quality_stamp']
    assert summary['height_rmse_m'] is None and summary['slope_mae_deg'] is None
    origins.add(tuple(round(v,1) for v in summary['origin']))
    assert (root/f'{folder.name}-stopped-velocity.txt').read_text(encoding='utf-8-sig').count(': 0.0')==6
assert len(origins)==3
for name in ('map-fault','map-fault-moving'):
    recording=root/name/'research'
    manifest=json.loads((recording/'manifest.json').read_text())
    assert manifest['status']=='captured' and manifest['report_status']=='ready' and not manifest['capture_errors']
    for filename,digest in json.loads((recording/'hashes.json').read_text()).items():
        assert hashlib.sha256((recording/filename).read_bytes()).hexdigest()==digest,(name,filename)
    mission=json.loads((recording/'mission.json').read_text())
    assert mission['phase']=='fault' and mission['history'][-1]['outcome']=='interrupted'
switched=json.loads((root/'clock-switch/after-switch.json').read_text(encoding='utf-8-sig'))
previous=json.loads((root/'clock-switch/before-switch.json').read_text(encoding='utf-8-sig'))
assert previous['task']['id']!=switched['task']['id']
assert previous['motion']['ready'] and switched['motion']['ready']
assert abs(previous['motion']['pose']['x']+6)<.1 and abs(switched['motion']['pose']['x']-12)<.1
fault=root/'map-fault-moving'
before=json.loads((fault/'before.json').read_text())
assert before['autonomy']['phase']=='moving' and before['motion']['active']
velocity=[float(line.split(':')[1]) for line in (fault/'before-velocity.txt').read_text().splitlines() if line.strip().startswith(('x:','y:','z:'))]
assert len(velocity)==6 and any(abs(v)>1e-5 for v in velocity[:3])
state=json.loads((fault/'fault.json').read_text())
assert state['autonomy']['phase']=='fault' and not state['autonomy']['active']
assert not state['motion']['map_fresh'] and '地图' in state['autonomy']['reason']
assert state['motion']['phase'] in ('cancelling','cancelled')
for path in sorted(fault.glob('stopped-velocity-*.txt')):
    assert path.read_text().count(': 0.0')==6
assert len(list(fault.glob('stopped-velocity-*.txt')))==3
recovered=json.loads((fault/'recovered.json').read_text())
assert recovered['motion']['ready'] and not recovered['motion']['active'] and not recovered['autonomy']['active']
print('PASS: three distinct continuous runs, frozen raw evidence, normal stops, stale-map stop and recovery')
