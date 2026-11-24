"""Layered capture checks using raw evidence; receipt times are not source stamps."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

def read(path):
    with Path(path).open() as stream:return [json.loads(line) for line in stream]
def coverage(values,start,end,bound=.100001):
    times=sorted(float(t) for t in values if start-1e-8<=float(t)<=end+1e-8)
    gaps=[b-a for a,b in zip([start]+times,times+[end])]
    return dict(samples=len(times),max_gap_s=max(gaps,default=end-start),passed=len(times)>1 and max(gaps)<=bound)

def layered_capture(folder,result):
    folder=Path(folder);start=result['sim_started'];end=result['sim_ended']
    main=read(folder/'observations.jsonl');probe=read(folder/'ros-probe.jsonl');native=read(folder/'native-imu.jsonl')
    layers=dict(native_imu=coverage([r['stamp'] for r in native],start,end),
        probe_imu=coverage([r['stamp'] for r in probe if r['kind']=='imu'],start,end),
        witness_imu=coverage([r['stamp'] for r in main if r['kind']=='imu'],start,end),
        probe_safe=coverage([r['sim'] for r in probe if r['kind']=='safe_command'],start,end),
        witness_safe=coverage([r['sim'] for r in main if r['kind']=='safe_command'],start,end))
    native_stamps={round(r['stamp'],8) for r in native if start<=r['stamp']<=end}
    probe_stamps={round(r['stamp'],8) for r in probe if r['kind']=='imu' and start<=r['stamp']<=end}
    main_stamps={round(r['stamp'],8) for r in main if r['kind']=='imu' and start<=r['stamp']<=end}
    missing=dict(native_not_in_probe=sorted(native_stamps-probe_stamps),probe_not_in_witness=sorted(probe_stamps-main_stamps))
    delays={}
    for kind in ('imu','safe_command'):
        observed={r['dds_source_timestamp_ns']:r for r in probe if r['kind']==kind and r.get('dds_source_timestamp_ns',0)>0}
        diffs=[r['wall']-observed[r['dds_source_timestamp_ns']]['wall'] for r in main
            if r['kind']==kind and r.get('dds_source_timestamp_ns') in observed and start<=r['sim']<=end]
        delays[kind]=dict(matched=len(diffs),max_witness_minus_probe_wall_s=max(diffs,default=None),
            note='Relative callback receipt latency; DDS timestamps are not simulation time')
    callback=[r for r in main if r['kind']=='image_callback']
    image_times={k:max((r['callback_duration_s'] for r in callback if r['image_kind']==k),default=None) for k in ('rgb','depth')}
    link_names=['mooncar']+['mooncar::'+link.get('name') for link in ET.parse(folder/'world.sdf').findall("./world/model[@name='mooncar']/link")]
    times={name:[] for name in link_names}
    for raw in read(folder/'link-poses.jsonl'):
        for pose in raw.get('pose',[]):
            if pose.get('name') in times:
                stamp=pose.get('header',{}).get('stamp',{});times[pose['name']].append(float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9)
    links={name:coverage(t,start,end) for name,t in times.items()}
    capture=json.loads((folder/'capture-status.json').read_text());probe_status=json.loads((folder/'probe-status.json').read_text())
    errors=list(capture['errors'])+list(probe_status['errors'])
    async_profile=capture['config']['profile']=='async';receipts=True
    if async_profile:
        receipt_rows=read(folder/'writer-receipts.jsonl');ids=[r['event_id'] for r in main];receipt_ids=[r['event_id'] for r in receipt_rows]
        receipts=len(set(ids))==len(ids) and set(ids)==set(receipt_ids) and len(ids)==len(receipt_ids)==capture['events']['written']
        if not receipts:errors.append('Event writer receipts incomplete')
    passed=all(c['passed'] for c in layers.values()) and all(c['passed'] for c in links.values()) and not errors and capture['drained'] and probe_status['thread_drained'] and receipts
    return dict(passed=bool(passed),layers=layers,imu_stamp_differences=missing,relative_receipt_delays=delays,
        maximum_image_callback_s=image_times,link_truth=links,capture_status=capture,probe_status=probe_status,errors=errors)
