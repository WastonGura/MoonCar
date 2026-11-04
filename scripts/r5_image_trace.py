"""Test-only image header tracing and exact relative sampling decisions."""
from collections import Counter
import json
from pathlib import Path

def decision(stamp_ns,previous_ns,mode='legacy'):
    if mode not in ('legacy','nanoseconds'):raise ValueError(mode)
    if previous_ns is None:return True
    if mode=='nanoseconds':return stamp_ns-previous_ns>=1_000_000_000
    seconds=lambda ns:ns//1_000_000_000+(ns%1_000_000_000)/1e9
    return seconds(stamp_ns)-seconds(previous_ns)>=1.

def analyze_images(folder,result):
    folder=Path(folder)
    def rows(name):
        with (folder/name).open() as stream:return [json.loads(line) for line in stream]
    native=rows('native-images.jsonl');probe=rows('ros-probe.jsonl');main=rows('observations.jsonl')
    status=json.loads((folder/'native-images-status.json').read_text())
    start=round(result['sim_started']*1e9);end=round(result['sim_ended']*1e9)
    out={};errors=list(status['errors'])
    probe_status=json.loads((folder/'probe-status.json').read_text())
    receipts=rows('probe-receipts.jsonl')
    if (len(native)!=status['written'] or status['written']!=status['received'] or
        len(probe)!=probe_status['events']['written'] or not probe_status['events']['drained'] or
        Counter(r['event_id'] for r in probe)!=Counter(r['event_id'] for r in receipts)):
        errors.append('Header writer receipt counts incomplete')
    errors.extend(probe_status.get('errors',[]))
    for kind in ('rgb','depth'):
        trace=[r for r in main if r['kind']=='image_trace' and r['image_kind']==kind]
        n=[r for r in native if r['kind']==kind and start<=r['stamp_ns']<=end]
        p=[r for r in probe if r['kind']==kind and start<=r['stamp_ns']<=end]
        t=[r for r in trace if start<=r['stamp_ns']<=end]
        def stamps(items):return Counter(r['stamp_ns'] for r in items)
        ns,ps,ts=stamps(n),stamps(p),stamps(t)
        missing=dict(native_not_probe=list((ns-ps).elements()),probe_not_main=list((ps-ts).elements()),
            probe_not_native=list((ps-ns).elements()),main_not_probe=list((ts-ps).elements()))
        wrong=[r for r in trace if r['selected']!=decision(r['stamp_ns'],r['previous_selected_ns'],r['sampling_mode'])]
        float_skips=[r for r in trace if not r['selected'] and decision(r['stamp_ns'],r['previous_selected_ns'],'nanoseconds')]
        selected=Counter(r['stamp_ns'] for r in trace if r['selected'])
        archived=Counter(round(r['stamp']*1e9) for r in main if r['kind']==kind)
        def gap(items):
            values=sorted(r['stamp_ns'] for r in items)
            return max([b-a for a,b in zip(values,values[1:])],default=0)/1e9
        matched=all(not v for v in missing.values())
        out[kind]=dict(native_frames=len(n),probe_frames=len(p),main_frames=len(t),missing=missing,
            layer_stamps_match=matched,native_max_gap_s=gap(n),probe_max_gap_s=gap(p),main_max_gap_s=gap(t),
            float_boundary_skips=float_skips,decision_errors=wrong,selected_archive_match=selected==archived)
        received_count=result.get('sensor_counts',{}).get(kind)
        trace_complete=received_count==len(trace) if received_count is not None else False
        out[kind]['received_trace_count_matches']=trace_complete
        # Best-effort sensor subscriptions may receive different frame subsets.
        # Preserve those differences as diagnosis, not as invented 10 Hz gates.
        if not(n and p and t and trace_complete and not wrong and selected==archived):errors.append(kind+' header/decision/archive trace incomplete')
    if not status['drained']:errors.append('Native image observer did not drain')
    return dict(passed=not errors,images=out,native_status=status,errors=errors,
        interpretation='Native stamps are transport observations, not proof of publisher completeness')
