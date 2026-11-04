"""Test-only delivery configuration and evidence qualification."""
from collections import Counter
import hashlib
import json
from pathlib import Path

def delivery_arguments(parser):
    parser.add_argument('--image-bridge',choices=('stock','traced'),default='stock')
    parser.add_argument('--image-delivery-profile',choices=('legacy','udp_rx4m','isolated_rx','shm_tx8m'),default='legacy')
    parser.add_argument('--traced-bridge')
    parser.add_argument('--cpp-image-observer')
    parser.add_argument('--dds-config')
    from r5_dds_shm import dds_arguments
    dds_arguments(parser)

def delivery_options(args):
    bridge=getattr(args,'image_bridge','stock');profile=getattr(args,'image_delivery_profile','legacy')
    cpp=getattr(args,'cpp_image_observer',None)
    if bridge=='stock' and profile=='legacy' and not cpp:return {}
    digest=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest() if p else None
    return dict(image_bridge=bridge,image_delivery_profile=profile,
        traced_bridge_sha256=digest(getattr(args,'traced_bridge',None)),
        cpp_image_observer_sha256=digest(cpp),dds_config_sha256=digest(getattr(args,'dds_config',None)))

def bridge_stages(rows):
    frames={};errors=[]
    for row in rows:
        key=(row['stream'],row['stamp_ns']);frames.setdefault(key,[]).append(row['stage'])
        if row.get('exception'):errors.append('Bridge publication/conversion exception')
    expected=['entry','converted','publish_begin','publish_return']
    for key,stages in frames.items():
        if stages!=expected:errors.append('Incomplete bridge stages: '+str(key))
    if not frames:errors.append('No bridge image stages')
    return dict(passed=not errors,frames=len(frames),errors=errors)

def writer_complete(rows,status):
    return bool(status.get('drained') and not status.get('failed') and
        status.get('received')==status.get('written')==len(rows))

def analyze_delivery(folder,result,traced=False):
    folder=Path(folder)
    load=lambda n:[json.loads(line) for line in (folder/n).read_text().splitlines()]
    cpp=load('cpp-images.jsonl');status=json.loads((folder/'cpp-images.jsonl.status.json').read_text())
    transport=json.loads((folder/'delivery-transport-status.json').read_text())
    errors=[]
    if not writer_complete(cpp,status):errors.append('C++ receiver did not completely drain')
    if transport.get('errors') or not transport.get('drained'):errors.append('Transport observer incomplete')
    bridge=load('bridge-images.jsonl') if traced else []
    phases=bridge_stages(bridge) if traced else dict(passed=True,available=False,reason='Stock bridge has no internal trace')
    if traced:
        bs=json.loads((folder/'bridge-images.jsonl.status.json').read_text())
        if not writer_complete(bridge,bs):errors.append('Bridge writer did not completely drain')
        errors.extend(phases['errors'])
    start=round(result['sim_started']*1e9);end=round(result['sim_ended']*1e9)
    native=load('native-images.jsonl');probe=load('ros-probe.jsonl');main=load('observations.jsonl')
    images={}
    for kind in ('rgb','depth'):
        topic='/mooncar/'+kind+'/image'
        layers={name:Counter(r['stamp_ns'] for r in rows if start<=r['stamp_ns']<=end) for name,rows in {
            'native':[r for r in native if r['kind']==kind],
            'cpp':[r for r in cpp if r['kind']==kind],
            'python_probe':[r for r in probe if r['kind']==kind],
            'main':[r for r in main if r['kind']=='image_trace' and r['image_kind']==kind],
            'bridge_return':[r for r in bridge if r['stream']==topic and r['stage']=='publish_return']}.items()}
        gaps={name:max([b-a for a,b in zip(sorted(values),sorted(values)[1:])],default=0)/1e9 for name,values in layers.items()}
        images[kind]=dict(counts={n:sum(v.values()) for n,v in layers.items()},max_gap_s=gaps,
            native_not_cpp=list((layers['native']-layers['cpp']).elements()),
            bridge_not_cpp=list((layers['bridge_return']-layers['cpp']).elements()),
            cpp_not_probe=list((layers['cpp']-layers['python_probe']).elements()),
            cpp_not_main=list((layers['cpp']-layers['main']).elements()),
            publication_sequence_available=any(r.get('publication_sequence') is not None for r in cpp if r['kind']==kind))
        if not traced:
            # No stock instrumentation: unavailable is not an observed zero.
            images[kind]['counts']['bridge_return']=None
            images[kind]['max_gap_s']['bridge_return']=None
            images[kind]['bridge_not_cpp']=None
        if not layers['cpp']:errors.append('No C++ '+kind+' records')
    return dict(passed=not errors,errors=errors,bridge_stages=phases,images=images,transport_status=transport,
        interpretation='Received best-effort subsets may differ; publish return is not delivery acknowledgement.')

def choose_candidate(evidence):
    """Fail closed. Endpoint-correlated transport/backlog evidence is required."""
    if evidence.get('contradictory') or evidence.get('shared_memory_fault'):return None
    if evidence.get('payload_udp_confirmed') and evidence.get('overflow_correlated'):return 'udp_rx4m'
    if evidence.get('cpp_continuous') and evidence.get('python_backlog_correlated'):return 'isolated_rx'
    return None
