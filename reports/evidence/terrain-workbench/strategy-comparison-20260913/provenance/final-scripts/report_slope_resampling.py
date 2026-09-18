"""Evaluate held-position/approach captures with the existing visible-surface evaluator."""
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from trajectory_evaluation import report as trajectory_report,ROOT
from surface_quality_evaluation import analyse
from survey_evaluation import write_csv,write_json
from terrain_map import TerrainMap


def report(root,output):
    output.mkdir(parents=True,exist_ok=False);rows=[];categories=[]
    for condition in ('stationary','approach'):
        folder=root/condition;control=json.loads((root/f'{condition}-control.json').read_text())
        if control['status']!='completed':
            rows.append(dict(condition=condition,status='capture_failed',error=control.get('error')));continue
        trajectory_report(folder)
        analyse(folder,output/condition,origin=(6.5,-2.),radius=5.)
        with (output/condition/'cells.csv').open(encoding='utf-8-sig') as handle:
            final={(int(c['ix']),int(c['iy'])):c for c in csv.DictReader(handle)}
        with np.load(folder/'samples.npz',allow_pickle=False) as data:
            points,offsets,cameras,stamps=data['points'],data['offsets'],data['truth_camera'],data['stamp']
            truth=data['truth_robot']
        cut=int(np.searchsorted(stamps,stamps[0]+5,side='right'));baseline=TerrainMap()
        for i in range(cut):
            baseline.add_frame(points[offsets[i]:offsets[i+1]],float(stamps[i]),cameras[i,:3,3],Rotation.from_matrix(cameras[i,:3,:3]).as_quat())
        before={(c['ix'],c['iy']):c for c in baseline.quality(with_surface=True)['cells']}
        for category in ('all','ground','sloped_top','mixed'):
            selected={k:c for k,c in before.items() if k in final and (category=='all' or final[k]['category']==category)}
            eligible=[k for k,c in selected.items() if c['fit_state'] in ('too_few_voxels','insufficient_xy_spread')]
            recovered=sum(final[k]['valid']=='True' for k in eligible)
            categories.append(dict(condition=condition,category=category,baseline_cells=len(selected),eligible=len(eligible),
                 recovered=recovered,recovery_fraction=recovered/len(eligible) if eligible else None))
        summary=json.loads((output/condition/'summary.json').read_text())
        slope=next(g for g in summary['groups'] if g['category']=='sloped_top')
        rows.append(dict(condition=condition,status='captured',frames=len(stamps),baseline_frames=cut,
                         motion_phase=control.get('motion_before_stop',{}).get('phase'),
                         intervention_completed=condition=='stationary' or control.get('motion_before_stop',{}).get('phase')=='succeeded',
                         elapsed_seconds=float(stamps[-1]-stamps[0]),
                         true_path_m=float(np.linalg.norm(np.diff(truth[:,:2,3],axis=0),axis=1).sum()),
                         slope_observed=slope['observed'],slope_invalid=slope['invalid'],slope_paired=slope['paired'],
                         slope_height_rmse_m=slope['height_rmse_m'],slope_mae_deg=slope['slope_mae_deg']))
    write_csv(output/'comparison.csv',rows);write_csv(output/'baseline-recovery.csv',categories)
    write_json(output/'summary.json',dict(trials=rows,baseline_recovery=categories,limits=[
        'One live pair with fresh simulation each time; not repeated statistical evidence.',
        'Independent initial images may differ; denominators and baseline frames are explicit.',
        'Approach uses Nav2 and the same safety lease; true poses are evaluator-only.',
        'Valid-fit recovery is separate from paired visible-surface error; mixed cells are not accuracy evidence.',
        'A fixed new-bearing utility does not model nearer views within the same bearing bin.']))
    print(json.dumps(rows,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();report(a.source,a.output)
