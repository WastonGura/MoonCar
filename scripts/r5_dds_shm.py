"""Test-only DDS runtime identity, metadata correlation and strict stage gates."""
import hashlib,json,os
from pathlib import Path

def project_runtime(source_root=None):
    """Fail before Gazebo when the project ROS overlay was not loaded."""
    from ament_index_python.packages import get_package_prefix
    import os
    try:prefix=Path(get_package_prefix('mooncar_control'))
    except LookupError as error:raise RuntimeError('Project ROS overlay not loaded: mooncar_control') from error
    executables={}
    for name in ('cmd_vel_safety_relay.py','wheel_odometry.py'):
        installed=prefix/'lib/mooncar_control'/name
        source=Path(source_root or '/source')/'ros2_ws/src/mooncar_control/scripts'/name
        if not installed.is_file() or not os.access(installed,os.X_OK):
            raise RuntimeError('Project ROS executable unavailable: '+str(installed))
        digest=hashlib.sha256(installed.read_bytes()).hexdigest()
        if digest!=hashlib.sha256(source.read_bytes()).hexdigest():
            raise RuntimeError('Installed control executable differs from frozen source: '+name)
        executables[name]=dict(path=str(installed),sha256=digest)
    return dict(prefix=str(prefix),executables=executables)

def dds_arguments(parser):
    parser.add_argument('--dds-runtime',choices=('stock','observed'),default='stock')
    parser.add_argument('--dds-trace',choices=('off','images'),default='off')
    parser.add_argument('--dds-prefix')
    parser.add_argument('--dds-candidate-evidence')

def runtime_options(args):
    mode=getattr(args,'dds_runtime','stock');trace=getattr(args,'dds_trace','off');prefix=getattr(args,'dds_prefix',None)
    if mode=='stock' and trace=='off' and not prefix:return {}
    manifest=Path(prefix)/'manifest.json' if prefix else None
    options=dict(dds_runtime=mode,dds_trace=trace,dds_runtime_manifest_sha256=
        hashlib.sha256(manifest.read_bytes()).hexdigest() if manifest else None)
    evidence=getattr(args,'dds_candidate_evidence',None)
    if getattr(args,'image_delivery_profile','legacy')=='shm_tx8m':
        options['dds_candidate_evidence_sha256']=hashlib.sha256(Path(evidence).read_bytes()).hexdigest() if evidence else None
    return options

def verify_runtime(args):
    mode=getattr(args,'dds_runtime','stock');trace=getattr(args,'dds_trace','off')
    if mode=='stock':
        if trace!='off':raise RuntimeError('Stock DDS has no internal image trace')
        if getattr(args,'image_delivery_profile','legacy')=='shm_tx8m':raise RuntimeError('SHM candidate requires observed RMW entry')
        return None
    prefix=Path(args.dds_prefix or '')
    manifest=json.loads((prefix/'manifest.json').read_text())
    if manifest['versions']!={'fastdds':'2.14.6','rmw_fastrtps_shared_cpp':'8.4.4','rmw_fastrtps_cpp':'8.4.4'}:
        raise RuntimeError('DDS runtime version mismatch')
    for name,digest in manifest['libraries'].items():
        if hashlib.sha256((prefix/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('DDS runtime library missing or changed: '+name)
    if getattr(args,'image_delivery_profile','legacy')=='shm_tx8m':
        evidence=getattr(args,'dds_candidate_evidence',None)
        if not evidence:raise RuntimeError('8MiB requires two complete observed baseline correlations')
        proof=json.loads(Path(evidence).read_text())
        if proof['runtime_manifest_sha256']!=hashlib.sha256((prefix/'manifest.json').read_bytes()).hexdigest():
            raise RuntimeError('Candidate evidence belongs to another DDS runtime')
        if len(proof['observed_defaults'])!=2:raise RuntimeError('Two observed baselines required')
        for item in proof['observed_defaults']:
            p=Path(item['result']);r=json.loads(p.read_text())
            if hashlib.sha256(p.read_bytes()).hexdigest()!=item['sha256'] or not r.get('dds',{}).get('passed') or not r['dds'].get('capacity_qualified'):
                raise RuntimeError('Incomplete or changed baseline correlation evidence')
    return manifest

def process_env(args,env,folder,label):
    env=dict(env);prefix=getattr(args,'dds_prefix',None)
    if prefix:env['LD_LIBRARY_PATH']=':'.join(p for p in env.get('LD_LIBRARY_PATH','').split(':') if p!=str(Path(prefix)/'lib'))
    if prefix:env['AMENT_PREFIX_PATH']=':'.join(p for p in env.get('AMENT_PREFIX_PATH','').split(':') if p!=str(prefix))
    env.pop('MOONCAR_DDS_TRACE',None);env.pop('MOONCAR_DDS_SHM_TX_BYTES',None)
    if getattr(args,'dds_runtime','stock')=='observed' and label in ('main','bridge','capture-probe','cpp-images'):
        env['LD_LIBRARY_PATH']=str(Path(prefix)/'lib')+':'+env.get('LD_LIBRARY_PATH','')
        env['AMENT_PREFIX_PATH']=str(prefix)+':'+env.get('AMENT_PREFIX_PATH','')
        if getattr(args,'dds_trace','off')=='images':env['MOONCAR_DDS_TRACE']=str(Path(folder)/(label+'-dds'))
        if label=='bridge' and getattr(args,'image_delivery_profile','legacy')=='shm_tx8m':env['MOONCAR_DDS_SHM_TX_BYTES']='8388608'
    return env

def correlation(rows,missing_stamps,image_kinds=None):
    """Only exact SHM tuples and publisher GUID/sequence join evidence."""
    key=lambda r:(r['segment'],r['offset'],r['validity'])
    tx={};timestamps={};recovery={};invalid={};replaced=set()
    for r in rows:
        if r['stage']=='fragment_tx':
            tx.setdefault(key(r),[]).append(r)
            if r.get('stamp_ns') is not None:timestamps[(r['writer'],r['sequence'])]=r['stamp_ns']
        if r['stage']=='capacity_recovery' and r['c']==1 and r['a']>0:
            recovery.setdefault(key(r),[]).append(r)
        if r['stage']=='buffer_invalid':invalid.setdefault(key(r),[]).append(r)
        if r['stage']=='reassembly_replaced':replaced.add((r['pid'],r['writer'],r['sequence']))
    links=[]
    for k,evs in recovery.items():
        if k not in invalid:continue
        for t in tx.get(k,[]):
            stamp=timestamps.get((t['writer'],t['sequence']))
            missing_key=stamp if image_kinds is None else (image_kinds.get(t['writer']),stamp)
            if stamp is None or missing_key not in missing_stamps:continue
            for rec in evs:
                # Hook wall times are reception/log times. The recovery hook runs
                # after the original warning; a receiver can log invalidity first.
                # The exact non-wrapping validity tuple provides identity.
                victims=invalid[k]
                if victims:links.append(dict(stamp_ns=stamp,image_kind=image_kinds.get(t['writer']) if image_kinds else None,writer=t['writer'],sequence=t['sequence'],fragment=t['fragment'],
                    buffer=dict(segment=k[0],offset=k[1],validity=k[2]),recovery=rec,invalid_receivers=victims,
                    incomplete_replaced=any((r['pid'],t['writer'],t['sequence']) in replaced for r in victims)))
    return links

def fragment_inventory(rows):
    """Received subsets remain diagnostic; unavailable identity stays missing."""
    groups={};unknown=[]
    for r in rows:
        if r['stage']!='fragment_rx':continue
        if r.get('sequence') is None:
            unknown.append(dict(pid=r['pid'],writer=r['writer'],reader=r['reader'],sequence=None,
                expected_fragments=None,missing_fragments=None,reason='Cannot group without sequence'))
            continue
        key=(r['pid'],r['writer'],r['reader'],r['sequence'])
        g=groups.setdefault(key,dict(arrived=[],sample_size=r['a'],fragment_size=r['b']))
        g['arrived'].extend(range(r['fragment'],r['fragment']+r['fragments']))
    out=[]
    for key,g in groups.items():
        arrived=g['arrived'];expected=(g['sample_size']+g['fragment_size']-1)//g['fragment_size'] if g['fragment_size'] else None
        from collections import Counter
        out.append(dict(pid=key[0],writer=key[1],reader=key[2],sequence=key[3],sample_size=g['sample_size'],
            fragment_size=g['fragment_size'],expected_fragments=expected,
            missing_fragments=sorted(set(range(1,expected+1))-set(arrived)) if expected else None,
            duplicate_fragments=[n for n,c in Counter(arrived).items() if c>1],
            out_of_order=any(a>b for a,b in zip(arrived,arrived[1:]))))
    return out+unknown

def analyze_dds(folder,result,args):
    folder=Path(folder);errors=[];rows=[];statuses=[]
    if args.dds_runtime=='stock':return dict(passed=True,available=False,reason='Original DDS/RMW; internal events unavailable')
    manifest=verify_runtime(args)
    expected_pids=json.loads((folder/'dds-processes.json').read_text())
    transport=[json.loads(s) for s in (folder/'delivery-transport.jsonl').read_text().splitlines()]
    loaded={}
    for label in ('main','bridge','capture-probe','cpp-images'):
        paths=list(folder.glob(label+'-dds.*.jsonl'))
        pid=expected_pids[label];primary=folder/(label+'-dds.'+str(pid)+'.jsonl')
        if primary not in paths:errors.append('Missing primary DDS log: '+label)
        snapshots=[t['processes'][str(pid)] for t in transport if str(pid) in t.get('processes',{})]
        libraries=sorted({p for s in snapshots for p in s['libraries']})
        loaded[label]=dict(pid=pid,libraries=libraries)
        for name in manifest['libraries']:
            if str(Path(args.dds_prefix)/name) not in libraries:errors.append('Actual DDS loaded library mismatch: '+label+': '+name)
        for path in paths:
            data=[json.loads(s) for s in path.read_text().splitlines()];rows.extend(data)
            sp=Path(str(path)+'.status.json')
            status=json.loads(sp.read_text()) if sp.exists() else {}
            statuses.append(dict(path=path.name,status=status))
            if not(status.get('drained') and not status.get('failed') and status.get('written')==len(data)==status.get('submitted')):
                errors.append('Incomplete DDS writer: '+path.name)
            if path!=primary:
                if data:errors.append('Unexpected DDS participant in archival worker: '+path.name)
                continue
            config=[r for r in data if r['stage']=='segment_initialized']
            expected=8388608 if label=='bridge' and args.image_delivery_profile=='shm_tx8m' else 524288
            if not config or any(r['a']!=expected or r['c']!=512 for r in config):errors.append('Effective SHM segment mismatch: '+label)
    counts={s:sum(r['stage']==s for r in rows) for s in sorted({r['stage'] for r in rows})}
    # Qualify the main recorder's missing source frames within failed archive gaps.
    observations=[json.loads(s) for s in (folder/'observations.jsonl').read_text().splitlines()]
    native=[json.loads(s) for s in (folder/'native-images.jsonl').read_text().splitlines()]
    missing=set();gaps=[]
    for kind in ('rgb','depth'):
        archived=sorted(round(r['stamp']*1e9) for r in observations if r['kind']==kind)
        received={r['stamp_ns'] for r in observations if r['kind']=='image_trace' and r['image_kind']==kind}
        for a,b in zip(archived,archived[1:]):
            if b-a<=1500000000:continue
            lost=[r['stamp_ns'] for r in native if r['kind']==kind and a+1000000000<=r['stamp_ns']<b and r['stamp_ns'] not in received]
            missing.update((kind,t) for t in lost);gaps.append(dict(kind=kind,start_ns=a,end_ns=b,missing_source_stamps=lost))
    main_pid=expected_pids['main']
    received_all={(r['image_kind'],r['stamp_ns']) for r in observations if r['kind']=='image_trace'}
    source_missing={(r['kind'],r['stamp_ns']) for r in native if round(result['sim_started']*1e9)<=r['stamp_ns']<=round(result['sim_ended']*1e9)
        and (r['kind'],r['stamp_ns']) not in received_all}
    image_kinds={r['writer']:r['image_kind'] for r in rows if r['stage']=='rmw_publish' and r.get('image_kind')}
    links=correlation(rows,source_missing,image_kinds)
    links=[l for l in links if any(r['pid']==main_pid for r in l['invalid_receivers'])]
    archive_links=[l for l in links if (l['image_kind'],l['stamp_ns']) in missing and l['incomplete_replaced']]
    effective_qos=[]
    for label in ('main','bridge','capture-probe','cpp-images'):
        stage='rmw_publish' if label=='bridge' else 'reader_qos'
        configs={(r['reader'] if stage=='reader_qos' else r['writer'],r['a'],r['b'],r['c'],r['d'])
            for r in rows if r['pid']==expected_pids[label] and r['stage']==stage}
        if not configs:errors.append('Missing actual DDS image QoS: '+label)
        effective_qos.extend(dict(process=label,endpoint=ep,reliability_raw=a,history_raw=b,depth=c,durability_raw=d)
            for ep,a,b,c,d in sorted(configs))
    (folder/'dds-fragment-inventory.json').write_text(json.dumps(fragment_inventory(rows))+'\n')
    metadata=dict(passed=not errors,available=True,errors=errors,counts=counts,writers=statuses,
        capacity_links=links,archive_capacity_links=archive_links,archive_gaps=gaps,capacity_qualified=bool(archive_links),normal_release=counts.get('normal_release',0),
        forced_waiting_recovery=sum(r['stage']=='capacity_recovery' and r['c']==1 and r['a']>0 for r in rows),
        actual_loaded_libraries=loaded,effective_qos=effective_qos,effective_segments=[r for r in rows if r['stage']=='segment_initialized'],
        interpretation='Invalid buffer content is never read; tuple joins use previously recorded transmitted headers.')
    (folder/'dds-correlation.json').write_text(json.dumps(metadata,indent=2)+'\n')
    return metadata

def diagnostic_complete(result,observed=False):
    """Image gaps may reproduce; every other observation/input gate stays strict."""
    errors=[]
    for name in ('capture_analysis','image_trace_analysis','delivery_analysis','step_observation'):
        if not result.get(name,{}).get('passed'):errors.append('Incomplete '+name)
    formal=result.get('formal',{})
    if not formal.get('command_window',{}).get('passed'):errors.append('Invalid real command window')
    if not formal.get('contacts_complete') or not formal.get('imagery_observable'):errors.append('Missing contacts or visual surface')
    if formal.get('safe_gap_s',float('inf'))>.100001:errors.append('Safety velocity observation gap')
    for name in ('scan','imu','joints','truth'):
        if not formal.get('sensors',{}).get(name,{}).get('passed'):errors.append('Missing '+name)
    if result.get('errors') or not result.get('observer_flush_confirmed'):errors.append('Runtime error or pause/flush missing')
    if observed and not result.get('dds',{}).get('passed'):errors.append('DDS observation incomplete')
    if errors:raise RuntimeError('; '.join(errors))
    return True

def qualify_candidate(entries,results):
    if len(entries)!=4:raise RuntimeError('Exactly four new baseline rounds required')
    stock=entries[:2];observed=entries[2:]
    if (all(e.get('passed') for e in stock) and all(not e.get('passed') for e in observed)) or \
            (all(not e.get('passed') for e in stock) and all(e.get('passed') for e in observed)):
        raise RuntimeError('Opposite stock/observed patterns: possible observer interference')
    if all(e.get('passed') for e in entries):raise RuntimeError('Un-reproduced; cause unconfirmed; no repair or calibration')
    for r in results:
        d=r.get('dds',{})
        if not d.get('passed') or not d.get('capacity_qualified'):raise RuntimeError('Both observed rounds need complete capacity/loss correlation')
    return 'shm_tx8m'
