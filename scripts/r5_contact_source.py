"""Test-only model contrast and per-step contact-source qualification."""
import copy,json,hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from r5_csv import read_numeric_csv

WHEELS=('front_left','front_right','rear_left','rear_right')
MATRIX=(('original','zero',0.),('original','straight-1',.1),('original','straight-2',.1),
        ('slip','straight-1',.1),('slip','straight-2',.1),('restore','straight',.1))

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def generate(source,output):
    reference=json.loads(Path('/source/reports/evidence/terrain-v2/r5-axle-allocation-20261010/allocator-models/manifest.json').read_text())
    if sha(source)!=reference['models']['candidate']:raise ValueError('Fixed candidate hash mismatch')
    p=Path(output);p.mkdir();slip=ET.parse(source);original=copy.deepcopy(slip)
    plugin=original.find("model/plugin[@name='gz::sim::systems::WheelSlip']")
    if plugin is None:raise ValueError('Expected native WheelSlip')
    original.find('model').remove(plugin)
    (p/'slip.sdf').write_bytes(Path(source).read_bytes())
    (p/'original.sdf').write_bytes(ET.tostring(original.getroot()))
    (p/'restore.sdf').write_bytes((p/'original.sdf').read_bytes())
    for name in ('original','slip','restore'):
        (p/(name+'.urdf')).write_bytes(Path(source).with_suffix('.urdf').read_bytes())
    from r5_load_diagnostic import semantic
    check=copy.deepcopy(slip);check.find('model').remove(check.find("model/plugin[@name='gz::sim::systems::WheelSlip']"))
    if semantic(original.getroot())!=semantic(check.getroot()):raise ValueError('Unexpected model change')
    manifest=dict(single_difference='Native WheelSlip10/0 enabled or absent; axle allocator identical and inactive during straight',
        models={n:sha(p/(n+'.sdf')) for n in ('original','slip','restore')},production_changed=False)
    (p/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');return manifest

def contact_grid(path,physics,schedule,identities):
    """Explicit empty ECS snapshots qualify absent contact; missing never qualifies."""
    rows=read_numeric_csv(path,delimiter=',',names=True);rows=np.atleast_1d(rows)
    first=schedule['parked']['before']['iteration'];last=schedule['stop']['after']['iteration']
    active=rows[(rows['iteration']>first)&(rows['iteration']<=last)]
    phy=physics[(physics['iteration']>first)&(physics['iteration']<=last)]
    if len(active)<4 or len(phy)!=last-first:raise ValueError('Missing per-step grid')
    if not np.array_equal(phy['iteration'],np.arange(first+1,last+1)):raise ValueError('Physics steps missing or duplicated')
    if np.any(active['state']<0):raise ValueError('Contact component unavailable; unknown cannot become zero')
    labels=active['iteration']*4+active['wheel'];unique,indices=np.unique(labels,return_index=True)
    expected=(np.arange(first+1,last+1)[:,None]*4+np.arange(4)).ravel()
    if not np.array_equal(unique,expected) or not np.all(np.diff(labels)>=0):raise ValueError('Source wheel/step missing or out of order')
    if np.any(active['self_collision']!=np.array(identities)[active['wheel'].astype(int)]):raise ValueError('Collision identity mismatch')
    pi=(active['iteration']-first-1).astype(int)
    if np.any(active['t_ns']!=np.rint(phy['t'][pi]*1e9)) or np.any(active['dt_ns']!=np.rint(phy['dt'][pi]*1e9)):
        raise ValueError('Source and physical observation times differ')
    positive=active['state']==1;empty=active['state']==0
    fields=('x','y','z','nx','ny','nz','depth','f1x','f1y','f1z','f2x','f2y','f2z')
    if np.any(~(positive|empty)):raise ValueError('Illegal source state')
    if np.any(active['extra_complete'][positive]!=1) or any(not np.isfinite(active[n][positive]).all() for n in fields):
        raise ValueError('Contact point/normal/depth/force incomplete')
    if any(not np.isnan(active[n][empty]).all() for n in fields):raise ValueError('Empty contact contains fabricated force or point')
    force1=np.column_stack([active[k][positive] for k in ('f1x','f1y','f1z')]);force2=np.column_stack([active[k][positive] for k in ('f2x','f2y','f2z')])
    if len(force1) and np.max(np.linalg.norm(force1+force2,axis=1))>1e-8:raise ValueError('Force direction association invalid')
    ends=np.r_[indices[1:],len(active)];per_wheel={w:dict(positive_steps=0,empty_steps=0) for w in WHEELS}
    for i,end in zip(indices,ends):
        group=active[i:end];w=WHEELS[int(group['wheel'][0])];state=group['state'][0]
        if not np.all(group['state']==state):raise ValueError('Inconsistent same-step state')
        if state==0:
            if len(group)!=1 or group['contact_count'][0]!=0 or group['point_count'][0]!=0:
                raise ValueError('Malformed empty snapshot')
            per_wheel[w]['empty_steps']+=1
        else:
            count=int(group['contact_count'][0])
            if count<1 or not np.all(group['contact_count']==count) or set(group['contact_index'])!=set(range(count)):
                raise ValueError('Contact group missing')
            for c in range(count):
                points=group[group['contact_index']==c];n=int(points['point_count'][0])
                if n<1 or len(points)!=n or not np.all(points['point_count']==n) or not np.array_equal(points['point_index'],np.arange(n)):
                    raise ValueError('Contact point missing/duplicated/unordered')
            per_wheel[w]['positive_steps']+=1
    if not all(v['positive_steps'] for v in per_wheel.values()):raise ValueError('Backend support not demonstrated for every wheel')
    marker=Path(str(path)+'.paused').read_text().strip().split(',')
    boundary=schedule['stop']['after']
    if len(marker)!=4 or int(marker[0])!=last or int(marker[1])!=round(boundary['sim']*1e9) or int(marker[2])!=len(rows) or int(marker[3])!=len(rows):
        raise ValueError('Actual pause or bounded-writer flush mismatch')
    return dict(passed=True,rows=len(rows),steps_per_wheel=last-first,wheels=per_wheel,
        missing_is_unknown=True,empty_is_explicit=True,flush_confirmed=True),rows

def topic_relation(folder,rows):
    """Compare sparse native contact publication with the complete ECS snapshots."""
    out={}
    for i,wheel in enumerate(WHEELS):
        source=rows[rows['wheel']==i];stamps,indices=np.unique(source['t_ns'].astype(np.int64),return_index=True)
        source_state=source['state'][indices];received=[]
        with (Path(folder)/(wheel+'-contact.jsonl')).open() as stream:
            for line in stream:
                record=json.loads(line);h=record.get('header',{}).get('stamp',{})
                received.append(int(h.get('sec',0))*1_000_000_000+int(h.get('nsec',0)))
        received=np.array(received,dtype=np.int64)
        matched=np.isin(stamps,received);positive=source_state==1
        runs=[];empty=source_state==0
        edges=np.diff(np.r_[False,empty,False].astype(int));starts=np.flatnonzero(edges==1);ends=np.flatnonzero(edges==-1)-1
        for a,b in zip(starts,ends):
            if stamps[b]-stamps[a]>=100_000_000:runs.append(dict(first_ns=int(stamps[a]),last_ns=int(stamps[b]),duration_s=float((stamps[b]-stamps[a])/1e9)))
        out[wheel]=dict(source_positive_steps=int(positive.sum()),positive_steps_without_topic=int((positive&~matched).sum()),
            source_empty_steps=int(empty.sum()),long_source_empty_intervals=runs,
            received_unknown_source_stamps=int((~np.isin(received,stamps)).sum()),
            received_at_empty_source_stamps=int((np.isin(stamps,received)&empty).sum()))
    return out
