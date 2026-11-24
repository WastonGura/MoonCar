#!/usr/bin/env python3
"""Read-only independent laser observability replay. Old runs are background only."""
import argparse
import json
from collections import Counter
from pathlib import Path
import numpy as np
from r5_adaptive_turn import scan_points, fit_scan_motion, body_delta


def replay(folder, output):
    scans=[];imu=[]
    with (Path(folder)/'observations.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line)
            if row['kind']=='scan':scans.append(row)
            if row['kind']=='imu':imu.append(row)
    rows=[]
    for before,after in zip(scans,scans[1:]):
        dt=after['stamp']-before['stamp']
        # Archive is sampled at 0.5 s; this only tests registration geometry,
        # never substitutes for the live 0.04--0.25 s controller timing gate.
        if dt<=0:continue
        previous=scan_points(before['ranges'],before['angle_min'],before['angle_increment'],before['range_min'],before['range_max'])
        current=scan_points(after['ranges'],after['angle_min'],after['angle_increment'],after['range_min'],after['range_max'])
        span=[r for r in imu if before['stamp']<=r.get('stamp',r.get('sim',-1))<=after['stamp']]
        # Existing archive gyro representation is inspected, not fabricated.
        seed=None
        if span:
            gyros=[r.get('angular',r.get('angular_velocity',r.get('gyro'))) for r in span]
            if all(g is not None and len(g)==3 for g in gyros):seed=float(np.mean([g[2] for g in gyros]))*dt
        fit=fit_scan_motion(previous,current,seed) if seed is not None else dict(valid=False,reason='missing_gyro_metadata')
        rows.append(dict(before=before['stamp'],after=after['stamp'],dt=dt,seed_yaw=seed,registration=fit,
            body_delta_m=body_delta(fit).tolist() if fit['valid'] else None))
    result=dict(scope='Historical read-only observability background, not fresh control validation',
        pairs=len(rows),valid_pairs=sum(r['registration']['valid'] for r in rows),
        reasons=dict(Counter(r['registration']['reason'] for r in rows)),rows=rows)
    Path(output).write_text(json.dumps(result,indent=2)+'\n')
    print({k:v for k,v in result.items() if k!='rows'})
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--folder',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    replay(a.folder,a.output)
