"""Paired replay mechanism check; the held image is a synthetic control, not a new live trial."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/mooncar_terrain/scripts'))
from terrain_map import TerrainMap
from survey_evaluation import write_json,write_csv


def run(capture, labels, output, baseline_seconds=27):
    with np.load(capture/'samples.npz',allow_pickle=False) as data:
        points,offsets,poses,stamps=data['points'],data['offsets'],data['truth_camera'],data['stamp']
    cut=int(np.searchsorted(stamps,stamps[0]+baseline_seconds,side='right'))
    if not 0<cut<len(stamps):raise ValueError('Baseline must leave both prefix and suffix')
    with labels.open(encoding='utf-8-sig') as handle:
        classes={(int(c['ix']),int(c['iy'])):c['category'] for c in csv.DictReader(handle)}
    maps={name:TerrainMap(source='evaluation-only paired replay') for name in ('held_image','changed_view')}
    def add(terrain,index,stamp):
        terrain.add_frame(points[offsets[index]:offsets[index+1]],float(stamp),poses[index,:3,3],
                          Rotation.from_matrix(poses[index,:3,:3]).as_quat())
    for terrain in maps.values():
        for i in range(cut):add(terrain,i,stamps[i])
    baseline={(c['ix'],c['iy']):c for c in maps['held_image'].quality(with_surface=True)['cells']}
    for i in range(cut,len(stamps)):
        add(maps['held_image'],cut-1,stamps[i]);add(maps['changed_view'],i,stamps[i])
    output.mkdir(parents=True,exist_ok=False);rows=[]
    for name,terrain in maps.items():
        cells={(c['ix'],c['iy']):c for c in terrain.quality(with_surface=True)['cells']}
        for key,old in baseline.items():
            new=cells.get(key,{})
            rows.append(dict(condition=name,ix=key[0],iy=key[1],category=classes.get(key,'unclassified'),
                             before=old['fit_state'],after=new.get('fit_state','missing'),
                             q_before=old['quality'],q_after=new.get('quality'),
                             points_before=old['points'],points_after=new.get('points')))
    groups=[]
    for name in maps:
        for category in ('all','ground','sloped_top','mixed'):
            subset=[r for r in rows if r['condition']==name and (category=='all' or r['category']==category)]
            eligible=[r for r in subset if r['before'] in ('too_few_voxels','insufficient_xy_spread')]
            recovered=sum(r['after']=='valid' for r in eligible)
            groups.append(dict(condition=name,category=category,baseline_cells=len(subset),eligible=len(eligible),
                               recovered=recovered,success_fraction=recovered/len(eligible) if eligible else None,
                               invalid_conflict=sum(r['after'] in ('steep_fit','nonplanar') for r in subset)))
    write_csv(output/'cells.csv',rows);write_csv(output/'groups.csv',groups)
    write_json(output/'summary.json',dict(baseline_seconds=baseline_seconds,baseline_frames=cut,
        suffix_frames=len(stamps)-cut,suffix_seconds=float(stamps[-1]-stamps[cut-1]),groups=groups,
        input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (capture/'samples.npz',labels)},
        limitations=['Held-image control repeats one captured image/pose; it does not model fresh sensor noise.',
                     'Changed-view suffix is an existing captured trajectory, not a newly controlled navigation trial.',
                     'Same prefix, frame timestamps/count and fusion; only suffix image/pose differs.',
                     'True poses and final independent surface labels are evaluation-only and never planner inputs.',
                     'Recovery of a fit does not prove accuracy; no strategy superiority or calibrated probability.']))
    print(json.dumps(groups,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('capture',type=Path);p.add_argument('labels',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();run(a.capture,a.labels,a.output)
