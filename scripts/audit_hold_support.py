"""Diagnose same-pose ray support against recorded depth; never uses world truth."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from swept_resampling import snapshot,rays,support
from survey_evaluation import write_json,write_csv


def audit(bundle,output):
    output.mkdir(parents=True,exist_ok=False)
    plan=json.loads((bundle/'captures/plan.json').read_text());rows=[];inputs={}
    for trial in plan['trials']:
        folder=bundle/'captures'/trial['folder'];capture=folder/'approach'
        meta=json.loads((capture/'checkpoint.json').read_text());cut=meta['frames']
        terrain,camera,robot=snapshot(capture,cut);cloud=terrain.cloud();quality=terrain.quality(True)
        fmt=json.loads((capture/'depth-format.json').read_text());info=meta['camera_info']
        assert fmt['encoding']=='32FC1','This audit requires the recorded float depth format'
        with np.load(capture/'raw-depth.npz',allow_pickle=False) as data:
            assert data['stamps'][cut-1]==meta['last_stamp']
            raw=data['data'][cut-1].copy()
        depth=np.ndarray((fmt['height'],fmt['width']),dtype=np.dtype('>f4' if fmt['bigendian'] else '<f4'),
            buffer=raw,strides=(fmt['step'],4))[::4,::4].astype(float).ravel()
        local=rays(info,meta['pixel_offset']);directions=local@camera[:3,:3].T
        valid=np.isfinite(depth)&(depth>=.1)&(depth<=6)
        actual=camera[:3,3]+depth[valid,None]*directions[valid]
        keys=np.floor(cloud[:,:2]/quality['resolution']).astype(int)
        for cell in quality['cells']:
            if cell['fit_state'] not in ('too_few_voxels','insufficient_xy_spread'):continue
            key=np.array([cell['ix'],cell['iy']]);observed=cloud[np.all(keys==key,axis=1)]
            height=float(np.median(observed[:,2]))
            t=np.divide(height-camera[2,3],directions[:,2],out=np.zeros(len(depth)),where=np.abs(directions[:,2])>1e-12)
            patch=camera[:3,3]+t[:,None]*directions
            hit=(t>=.1)&(t<=6)&np.all(np.floor(patch[:,:2]/quality['resolution']).astype(int)==key,axis=1)
            measured=actual[np.all(np.floor(actual[:,:2]/quality['resolution']).astype(int)==key,axis=1)]
            baseline=support(observed,quality['resolution'],.05)
            predicted=support(np.vstack([observed,patch[hit]]),quality['resolution'],.05)
            replayed=support(np.vstack([observed,measured]),quality['resolution'],.05)
            if not predicted['can_fit']:continue
            oldvox={tuple(v) for v in np.floor(observed/.05).astype(int)}
            fabricated={tuple(v) for v in np.floor(patch[hit]/.05).astype(int)}-oldvox
            measuredvox={tuple(v) for v in np.floor(measured/.05).astype(int)}
            indices=np.flatnonzero(hit);finite=indices[valid[indices]]
            measured_at_ray=camera[:3,3]+depth[finite,None]*directions[finite]
            other=np.any(np.floor(measured_at_ray[:,:2]/quality['resolution']).astype(int)!=key,axis=1)
            rows.append(dict(folder=trial['folder'],ix=int(key[0]),iy=int(key[1]),
                observed_points=len(observed),baseline_can_fit=baseline['can_fit'],hypothesis_points=predicted['points'],
                actual_ray_replay_points=replayed['points'],actual_ray_replay_can_fit=replayed['can_fit'],
                invented_new_voxels=len(fabricated),new_voxels_supported_by_depth=len(fabricated&measuredvox),
                hypothetical_rays=len(indices),invalid_actual_rays=int(sum(~valid[indices])),
                actual_rays_hit_other_cell=int(sum(other)),
                mean_abs_depth_difference_m=float(np.mean(abs(t[finite]-depth[finite]))) if len(finite) else None))
        for path in (capture/'checkpoint.npz',capture/'raw-depth.npz',capture/'depth-format.json',capture/'camera-info.json'):
            inputs[str(path.relative_to(bundle))]=hashlib.sha256(path.read_bytes()).hexdigest()
    write_csv(output/'cells.csv',rows)
    result=dict(cells=rows,hold_predictions=len(rows),baseline_already_fit=sum(r['baseline_can_fit'] for r in rows),
        measured_replay_fit=sum(r['actual_ray_replay_can_fit'] for r in rows),input_sha256=inputs,
        limits='Diagnostic on existing development data. Same-pose depth replay is not a future-view model or a physical noise guarantee.')
    write_json(output/'summary.json',result)
    print(json.dumps(result,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bundle',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();audit(a.bundle,a.output)
