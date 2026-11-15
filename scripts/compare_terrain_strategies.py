"""Causal offline A/B replay of frozen terrain strategies on archived observations."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/mooncar_terrain/scripts'))
from terrain_geometry import classify_cells, transform_points
from literature_terrain import ElevationNeighborhood


def sensor_frames(path):
    with np.load(path,allow_pickle=False) as archive:
        for i,stamp in enumerate(archive['stamps']):
            if not archive['robot_valid'][i]:
                raise ValueError('Self-filter needs the archived robot pose')
            sensor=archive['points'][archive['offsets'][i]:archive['offsets'][i+1]]
            points=transform_points(sensor,archive['translations'][i],archive['quaternions'][i])
            robot=archive['robot_poses'][i]
            q=robot[3:]/np.linalg.norm(robot[3:])
            x,y,z,w=q
            rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                               [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                               [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
            local=(points-robot[:3])@rotation
            own=((sensor[:,2]<.40)|((local[:,0]>=-.50)&(local[:,0]<=.50)&
                 (abs(local[:,1])<=.42)&(local[:,2]>=.02)))
            yield float(stamp),points[~own]


def synthetic_scenes():
    x,y=np.meshgrid(np.arange(-.79,.8,.025),np.arange(-.79,.8,.025))
    for name,height in (
        ('flat',x*0),('gentle_10deg',x*math.tan(math.radians(10))),
        ('steep_25deg',x*math.tan(math.radians(25))),
        ('raised_25cm',np.where(x>=0,.25,0.)),
        ('drop_25cm',np.where(x>=0,-.25,0.)),
        ('rough_6cm_rms',.12*np.sin(80*x)*np.sin(80*y)),
    ):
        yield name,np.column_stack((x.ravel(),y.ravel(),height.ravel()))
    line=np.linspace(.01,.19,20)
    yield 'single_scanline',np.column_stack((line,line*0+.1,line*0))


def statistics(values):
    return dict(mean_ms=float(np.mean(values)),p95_ms=float(np.percentile(values,95)),
                max_ms=float(max(values)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--archives',type=Path,nargs='+')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    data=ROOT/'tmp/ros_acceptance/turn-minimap-video-20261111'
    archives=args.archives or [next((data/'sessions').glob('experiment-*/*.npz')),
                               next((data/'retry-sessions').glob('experiment-*/*.npz'))]
    sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    protocol=dict(baseline_commit='5fe51ab',tested_commit=subprocess.check_output(
        ['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
        archives={str(p.relative_to(ROOT)):sha(p) for p in archives},
        source_sha256={str(p.relative_to(ROOT)):sha(p) for p in [
            ROOT/'scripts/literature_terrain.py',Path(__file__),ROOT/'ros2_ws/src/mooncar_terrain/scripts/terrain_geometry.py']},
        parameters=dict(output_resolution=.2,fine_resolution=.05,normal_radius=.36,max_age_s=5.,
                        slope_deg=18.,roughness_m=.04,step_m=.18,drop_m=.12),
        localization='same archived corrected sensor/robot poses',
        threads=dict(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'),
        scope='Offline terrain classification. Movement and safety-check implementations are excluded from this A/B.',
        limitations='Keyframes omit some online depth frames. No calibrated pose/depth covariance is present. Known-cell metrics exclude unknown, which is reported separately.')
    (args.output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    summaries=[];records=[];maps=[]
    for archive in archives:
        candidate=ElevationNeighborhood()
        timing={'baseline':[],'candidate':[]}
        target={'baseline':[],'candidate':[]}
        totals={name:dict(classified=0,hazard=0,observed_unknown=0) for name in timing}
        final={'baseline':{},'candidate':{}}
        maximum_cells=0;frames=0
        for index,(stamp,points) in enumerate(sensor_frames(archive)):
            frames+=1
            observed={tuple(k) for k in np.unique(np.floor(points[:,:2]/.2).astype(int),axis=0)}
            for name in timing:
                begun=time.perf_counter()
                cells=(classify_cells(points,resolution=.2,return_elevation=True) if name=='baseline'
                       else candidate.update(points,stamp,return_elevation=True))
                elapsed=(time.perf_counter()-begun)*1000
                timing[name].append(elapsed)
                decoded={(math.floor(c[0]/.2),math.floor(c[1]/.2)):c for c in cells}
                dangerous=sum(c[4]!=0 or c[2]>=18 or c[3]>=.04 for c in cells)
                totals[name]['classified']+=len(cells)
                totals[name]['hazard']+=dangerous
                totals[name]['observed_unknown']+=len(observed-decoded.keys())
                final[name].update(decoded)
                entry=dict(archive=archive.parent.name,frame=index,stamp=stamp,strategy=name,
                           input_points=len(points),observed_cells=len(observed),
                           classified=len(cells),hazard=dangerous,
                           unknown=len(observed-decoded.keys()),processing_ms=elapsed)
                if (19,6) in observed:
                    cell=decoded.get((19,6))
                    selected=dict(frame=index,stamp=stamp,status='unknown' if cell is None else
                                  'hazard' if cell[4] or cell[2]>=18 or cell[3]>=.04 else 'passable')
                    if cell is not None:
                        selected.update(slope=cell[2],roughness=cell[3],reason=cell[4])
                    target[name].append(selected)
                records.append(entry)
            maximum_cells=max(maximum_cells,len(candidate.cells))
        summary=dict(archive=str(archive.relative_to(ROOT)),frames=frames,candidate_peak_fine_cells=maximum_cells)
        for name in timing:
            summary[name]=dict(**totals[name],timing=statistics(timing[name]),target=target[name],
                               target_counts={s:sum(row['status']==s for row in target[name])
                                              for s in ('passable','hazard','unknown')})
        summaries.append(summary)
        maps.append((archive.parent.name,final))
    checks=[]
    for name,points in synthetic_scenes():
        candidate=ElevationNeighborhood()
        baseline=classify_cells(points,resolution=.2,return_elevation=True)
        candidate.update(points,0.)
        cells=candidate.update(points,.1,return_elevation=True)
        entry=dict(scene=name)
        for strategy,result in (('baseline',baseline),('candidate',cells)):
            half_x=.2 if name in ('raised_25cm','drop_25cm') else .4
            center=[c for c in result if abs(c[0])<half_x and abs(c[1])<.4]
            entry[strategy]=dict(known=len(center),hazard=sum(c[4]!=0 or c[2]>=18 or c[3]>=.04 for c in center),
                                 reasons=sorted({int(c[4]) for c in center}))
        expected='unknown' if name=='single_scanline' else 'passable' if name in ('flat','gentle_10deg') else 'hazard'
        entry['expected']=expected
        for strategy in ('baseline','candidate'):
            result=entry[strategy]
            entry[strategy]['matches_expected']=(result['known']==0 if expected=='unknown' else
                result['known']>0 and result['hazard']==0 if expected=='passable' else
                result['known']>0 and result['hazard']==result['known'])
        checks.append(entry)
    result=dict(protocol=protocol,datasets=summaries,synthetic_checks=checks)
    (args.output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    with (args.output/'frames.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=records[0].keys())
        writer.writeheader();writer.writerows(records)
    draw_comparison(maps,args.output/'comparison.png')
    print(json.dumps(dict(datasets=[dict(archive=s['archive'],frames=s['frames'],
        baseline_target=s['baseline']['target_counts'],candidate_target=s['candidate']['target_counts'],
        baseline_timing=s['baseline']['timing'],candidate_timing=s['candidate']['timing']) for s in summaries],
        synthetic=checks),ensure_ascii=False),flush=True)


def draw_comparison(maps,path):
    from PIL import Image,ImageDraw,ImageFont
    font_path=Path('C:/Windows/Fonts/msyh.ttc')
    if not font_path.exists():
        font_path=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    font=ImageFont.truetype(str(font_path),18)
    picture=Image.new('RGB',(1200,120+len(maps)*440),'#f4f6f8')
    draw=ImageDraw.Draw(picture)
    draw.text((25,18),'同一输入的地形分类对比 · 最后一次有效分类',fill='#13263a',font=font)
    draw.text((25,48),'灰 未知   绿 可通行   红 危险   黄框 已诊断的小石子格',fill='#30465d',font=font)
    for row,(name,mapping) in enumerate(maps):
        for col,strategy in enumerate(('baseline','candidate')):
            left,top=25+col*590,105+row*440
            draw.text((left,top),name+' / '+strategy,fill='#13263a',font=font)
            scale=45.;x0,x1,y0,y1=-1.,6.,-3.,3.
            bx,by=left+30,top+38
            draw.rectangle((bx,by,bx+(x1-x0)*scale,by+(y1-y0)*scale),fill='#cfd5dc')
            def point(x,y):return bx+(x-x0)*scale,by+(y1-y)*scale
            for (ix,iy),c in mapping[strategy].items():
                x,y=(ix+.5)*.2,(iy+.5)*.2
                if not x0<=x<x1 or not y0<=y<y1:
                    continue
                px,py=point(x-.1,y+.1)
                hazard=c[4]!=0 or c[2]>=18 or c[3]>=.04
                draw.rectangle((px,py,px+.2*scale,py+.2*scale),fill='#c95656' if hazard else '#83aa9b')
            px,py=point(3.8,1.4)
            draw.rectangle((px-3,py-3,px+.2*scale+3,py+.2*scale+3),outline='#ffc42b',width=3)
    picture.save(path)


if __name__=='__main__':
    main()
