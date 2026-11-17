"""Summarize the actual controlled crossing without turning it into full-field acceptance."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

OUT=Path(__file__).resolve().parent


def read(path):return json.loads(path.read_text(encoding='utf-8'))
def rows(path):return [json.loads(s) for s in path.read_text(encoding='utf-8').splitlines() if s.strip()]


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--trial',type=Path,required=True)
    args=parser.parse_args();trial=args.trial;result=read(trial/'case/result.json')
    ready=read(trial/'case/ready.json')['motion']['diagnostics']['sim_stamp']
    truth=[r for r in rows(trial/'case/observer/truth.jsonl') if r['stamp']>=ready]
    alignment=[r['xy_error_m'] for r in rows(trial/'case/observer/alignment.jsonl') if r['stamp']>=ready]
    states=rows(trial/'case/states.jsonl')
    moving=[r['state']['motion'] for r in states if r['state']['motion'].get('active')]
    unhealthy=sum(not m.get('diagnostics',{}).get('laser_localization',{}).get('healthy',False) for m in moving)
    selected=[];last=-math.inf
    for row in truth:
        if row['stamp']-last>=.1:selected.append(row);last=row['stamp']
    route=[r['xyz'][:2] for r in selected]
    crossing=[r for r in truth if abs(r['xyz'][0]-3.)<.05 and -8.1<r['xyz'][1]<-2.1]
    final=truth[-1]['xyz'];goal=result['actions'][-1]['target']
    assert result['outcome']=='passed' and crossing
    recording=read(trial/'video/recording.json')
    summary=dict(experiment=result['experiment'],tested_commit=result['software']['git_commit'],
        scene='survey_v3_native',outcome=result['outcome'],actions=[dict(kind=a['kind'],target=a['target'],phase=a['phase']) for a in result['actions']],
        successful_moves=sum(a['kind']=='move' and a['phase']=='succeeded' for a in result['actions']),
        truth_track_m=sum(math.dist(a,b) for a,b in zip(route,route[1:])),final_truth_xyz=final,
        final_truth_distance_to_goal_m=math.dist(final[:2],goal),
        crossed_old_background_west_edge=True,edge_crossing_y_m=float(np.mean([r['xyz'][1] for r in crossing])),
        unhealthy_active_state_snapshots=unhealthy,active_state_snapshots=len(moving),
        xy_error_m=dict(median=float(np.median(alignment)),p95=float(np.percentile(alignment,95)),maximum=max(alignment)),
        maximum_absolute_body_roll_deg=max(abs(math.degrees(r['roll'])) for r in truth),
        maximum_absolute_body_pitch_deg=max(abs(math.degrees(r['pitch'])) for r in truth),
        wall_seconds=result['elapsed_wall_s'],actual_screen_captures=recording['frame_count'],capture_error=recording['error'],
        process_exits={n:read(trial/(n+'-exit.json'))['exit_code'] for n in ('driver','video','backend')},
        scope='One controlled route through the old patch edge, default localization and terrain classifier; not whole-field/long-duration or tyre-soil calibration')
    (OUT/'crossing-summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8',newline='\n')
    hashes={}
    for path in (trial/'case/result.json',trial/'case/states.jsonl',trial/'case/observer/truth.jsonl',
                 trial/'case/observer/alignment.jsonl',trial/'video/scene-walltime.mp4',trial/'video/frames.jsonl'):
        digest=hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
        hashes[str(path)]=digest.hexdigest()
    (OUT/'evidence-sha256.json').write_text(json.dumps(hashes,indent=2)+'\n',encoding='utf-8',newline='\n')
    print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
