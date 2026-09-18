"""Offline nearest-visible-surface ray checks; pixel offsets are diagnostic hypotheses only."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
from terrain_reference import read_shapes
from trajectory_evaluation import ROOT
from terrain_geometry import project_depth
from survey_evaluation import write_json,write_csv


def first_hit(shapes,origin,directions):
    """Return nearest positive t in origin+t*direction (directions need not be unit)."""
    origin=np.asarray(origin,dtype=float); directions=np.asarray(directions,dtype=float)
    if origin.shape!=(3,) or directions.ndim!=2 or directions.shape[1]!=3 or not np.isfinite(origin).all() or not np.isfinite(directions).all():
        raise ValueError('Expected finite origin and Nx3 directions')
    nearest=np.full(len(directions),np.inf); owner=np.full(len(directions),-1)
    for index,(_,kind,size,pose) in enumerate(shapes):
        o=(origin-pose[:3,3])@pose[:3,:3]; d=directions@pose[:3,:3]
        hit=np.full(len(d),np.inf)
        if kind=='plane':
            valid=np.abs(d[:,2])>1e-12
            t=np.divide(-o[2],d[:,2],out=np.zeros(len(d)),where=valid)
            point=o+t[:,None]*d
            valid &= (t>1e-8)&np.all(np.abs(point[:,:2])<=size/2,axis=1)
            hit[valid]=t[valid]
        elif kind=='box':
            low=np.full(len(d),-np.inf); high=np.full(len(d),np.inf); valid=np.ones(len(d),bool)
            for axis in range(3):
                moving=np.abs(d[:,axis])>1e-12
                a=np.divide(-size[axis]-o[axis],d[:,axis],out=np.zeros(len(d)),where=moving)
                b=np.divide(size[axis]-o[axis],d[:,axis],out=np.zeros(len(d)),where=moving)
                low=np.maximum(low,np.where(moving,np.minimum(a,b),-np.inf))
                high=np.minimum(high,np.where(moving,np.maximum(a,b),np.inf))
                valid &= moving | (abs(o[axis])<=size[axis])
            t=np.where(low>1e-8,low,high)
            valid &= (high>=low)&(t>1e-8)&np.isfinite(t)
            hit[valid]=t[valid]
        else:
            if kind=='ellipsoid':
                scaled=d/size; point=o/size
            elif kind=='cylinder':
                scaled=d[:,:2]/size[0]; point=o[:2]/size[0]
            else: raise ValueError('Unsupported ray geometry')
            a=np.sum(scaled**2,axis=1); b=2*(scaled@point); c=point@point-1
            disc=b*b-4*a*c; valid=(a>1e-16)&(disc>=0)
            for sign in (-1,1):
                t=np.divide(-b+sign*np.sqrt(np.maximum(disc,0)),2*a,out=np.zeros(len(d)),where=valid)
                ok=valid&(t>1e-8)
                if kind=='cylinder': ok &= np.abs(o[2]+t*d[:,2])<=size[1]
                hit=np.minimum(hit,np.where(ok,t,np.inf))
            if kind=='cylinder':
                for z in (-size[1],size[1]):
                    moving=np.abs(d[:,2])>1e-12
                    t=np.divide(z-o[2],d[:,2],out=np.zeros(len(d)),where=moving)
                    ok=moving&(t>1e-8)&(np.sum((o[:2]+t[:,None]*d[:,:2])**2,axis=1)<=size[0]**2)
                    hit=np.minimum(hit,np.where(ok,t,np.inf))
        update=hit<nearest;nearest[update]=hit[update];owner[update]=index
    return nearest,owner


def statistics(values):
    v=np.asarray(values); v=v[np.isfinite(v)]
    return dict(points=len(v),rmse_m=float(np.sqrt(np.mean(v*v))) if len(v) else None,
                bias_m=float(np.mean(v)) if len(v) else None,
                median_abs_m=float(np.median(np.abs(v))) if len(v) else None,
                p95_abs_m=float(np.percentile(np.abs(v),95)) if len(v) else None)


def evaluate(folder,output):
    if output.exists() or folder.resolve()==output.resolve() or folder.resolve() in output.resolve().parents:
        raise ValueError('Use a new output directory outside the source capture')
    info=json.loads((folder/'camera-info.json').read_text()); fx,fy,cx,cy=[info['k'][i] for i in (0,4,2,5)]
    manifest=json.loads((folder/'manifest.json').read_text())
    if manifest['status']!='captured': raise ValueError('Capture must complete before analysis')
    source_offset=manifest.get('projection_pixel_offset',0.)
    shapes=read_shapes(folder/'survey_yard.sdf'); frames=[]; errors=[[],[]]; owners=[[],[]]; first=None
    raw=np.load(folder/'raw-depth.npz',allow_pickle=False) if (folder/'raw-depth.npz').exists() else None
    raw_data=raw['data'] if raw is not None else None
    fmt=json.loads((folder/'depth-format.json').read_text()) if raw is not None else None
    raw_matches=0; max_pixel_residual=0.
    try:
        with np.load(folder/'samples.npz',allow_pickle=False) as data:
            if raw is not None and not np.array_equal(raw['stamps'],data['stamp']):raise ValueError('Raw image stamps differ')
            for i,stamp in enumerate(data['stamp']):
                points=data['points'][data['offsets'][i]:data['offsets'][i+1]]
                camera=data['truth_camera'][i]
                if raw is not None:
                    projected=project_depth(raw_data[i].tobytes(),fmt['width'],fmt['height'],fmt['step'],fmt['encoding'],fmt['bigendian'],(fx,fy,cx,cy),pixel_offset=source_offset).astype(np.float32)
                    if not np.array_equal(projected,points): raise ValueError('Published cloud differs from same-stamp raw depth projection')
                    raw_matches+=1
                pixel=np.column_stack((points[:,0]/points[:,2]*fx+cx,points[:,1]/points[:,2]*fy+cy))-source_offset
                max_pixel_residual=max(max_pixel_residual,float(np.max(np.abs(pixel-np.round(pixel)))))
                reference=[]; row=dict(stamp=float(stamp),observed_points=len(points))
                for j,offset in enumerate((0.,.5)):
                    local=points/points[:,2,None];local=local.astype(float)
                    local[:,0]+=(offset-source_offset)/fx;local[:,1]+=(offset-source_offset)/fy
                    expected,owner=first_hit(shapes,camera[:3,3],local@camera[:3,:3].T)
                    error=points[:,2]-expected;error[owner<0]=np.nan
                    errors[j].append(error);owners[j].append(owner)
                    row.update({f'offset_{offset}_{k}':v for k,v in statistics(error).items()})
                    reference.append(error)
                frames.append(row)
                if first is None:
                    first=dict(pixel=pixel,error=reference,raw=raw_data[i].copy() if raw is not None else None)
    finally:
        if raw is not None:raw.close()
    result=[]
    for j,offset in enumerate((0.,.5)):
        error=np.concatenate(errors[j]);owner=np.concatenate(owners[j])
        for category in ('all','plane','solid'):
            mask=np.array([k>=0 and (category=='all' or (shapes[k][1]=='plane')==(category=='plane')) for k in owner])
            result.append(dict(pixel_offset=offset,category=category,**statistics(error[mask])))
    output.mkdir(parents=True)
    write_csv(output/'frames.csv',frames);write_csv(output/'groups.csv',result)
    np.savez_compressed(output/'residuals.npz',nominal=np.concatenate(errors[0]),half_pixel=np.concatenate(errors[1]),
                        nominal_owner=np.concatenate(owners[0]),half_pixel_owner=np.concatenate(owners[1]),
                        offsets=np.r_[0,np.cumsum([f['observed_points'] for f in frames])])
    inputs={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir() if p.name in ('manifest.json','samples.npz','raw-depth.npz','camera-info.json','depth-format.json','survey_yard.sdf')}
    write_json(output/'summary.json',dict(status='diagnostic_only',source=str(folder),input_sha256=inputs,frames=len(frames),
        observed_points=sum(f['observed_points'] for f in frames),raw_frames_bitwise_reprojected=raw_matches,
        recovered_pixel_max_fraction=max_pixel_residual,groups=result,
        nominal_missing_reference=int(np.sum(np.concatenate(owners[0])<0)),
        half_pixel_missing_reference=int(np.sum(np.concatenate(owners[1])<0)),
        changed_first_surface=int(np.sum(np.concatenate(owners[0])!=np.concatenate(owners[1]))),
        shapes=[dict(name=s[0],kind=s[1]) for s in shapes],source_pixel_offset=source_offset,
        source_offset_provenance='recorded_node_parameter' if 'projection_pixel_offset' in manifest else 'legacy_zero_projection',
        limitations=['Compare optical-axis depth with first visible analytic visual surface, not vertical highest elevation.',
                     'Absolute pixel offsets 0 and 0.5 are evaluated independently of producer offset; no hardware rule implied.',
                     'Analytic primitives may differ from rendered meshes, sampling and edge rasterization.',
                     'Same observed valid pixels in both hypotheses; missing reference counts and first-surface changes retained.',
                     'Points and stationary frames are correlated; no statistical superiority claim.']))
    plot(first,info,fmt,result,output)
    for name in ('depth_projection_evaluation.py','trajectory_evaluation.py','terrain_reference.py','simulation_experiment.py','survey_evaluation.py'):
        shutil.copyfile(ROOT/'scripts'/name,output/name)
    shutil.copyfile(ROOT/'ros2_ws/src/mooncar_terrain/scripts/terrain_geometry.py',output/'terrain_geometry.py')
    write_json(output/'hashes.json',{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir() if p.is_file() and p.name!='hashes.json'})
    print(json.dumps(result,indent=2))


def plot(first,info,fmt,groups,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(11,3.8),layout='constrained')
    cmap=plt.get_cmap('gray_r').copy();cmap.set_bad('#dddddd')
    if first['raw'] is not None:
        dtype=np.dtype(('>' if fmt['bigendian'] else '<')+('f4' if fmt['encoding']=='32FC1' else 'u2'))
        depth=np.ndarray((fmt['height'],fmt['width']),dtype=dtype,buffer=first['raw'],strides=(fmt['step'],dtype.itemsize)).astype(float)
        if fmt['encoding']=='16UC1':depth*=.001
        im=axes[0].imshow(np.ma.masked_invalid(depth),cmap=cmap,vmin=0,vmax=8);fig.colorbar(im,ax=axes[0],label='Depth (m)',shrink=.7)
    else: axes[0].text(.5,.5,'Raw image not captured\nUse raw-02 evidence',ha='center',va='center',transform=axes[0].transAxes)
    axes[0].set_title('Raw depth (first frame)')
    for j,ax in enumerate(axes[1:]):
        grid=np.full((info['height']//4,info['width']//4),np.nan)
        pixel=np.rint(first['pixel']).astype(int)//4
        grid[pixel[:,1],pixel[:,0]]=np.abs(first['error'][j])*1000
        im=ax.imshow(grid,cmap=cmap,vmin=0,vmax=100,extent=(0,info['width'],info['height'],0))
        fig.colorbar(im,ax=ax,label='Abs depth error (mm)',extend='max',shrink=.7)
        ax.set_title(('Pixel offset = 0','Pixel offset = 0.5')[j])
    for ax in axes:ax.set(xlabel='Image column',ylabel='Image row')
    fig.supxlabel('Gray background: no valid depth/reference. Error display capped at 100 mm; statistics are not clipped.',fontsize=8)
    fig.savefig(output/'depth-comparison.png',dpi=180);fig.savefig(output/'depth-comparison.pdf');plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();evaluate(args.folder,args.output)
