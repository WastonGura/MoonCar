from pathlib import Path
import hashlib,json,subprocess
import argparse
p=argparse.ArgumentParser();p.add_argument('--prefix',default='/work/dds-observed');args=p.parse_args()
prefix=Path(args.prefix)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
names=('lib/libfastrtps.so.2.14.6','lib/librmw_fastrtps_cpp.so','lib/librmw_fastrtps_shared_cpp.so','lib/libmooncar-dds-trace.so')
metadata=dict(versions=dict(fastdds='2.14.6',rmw_fastrtps_shared_cpp='8.4.4',rmw_fastrtps_cpp='8.4.4'),
    libraries={p:sha(prefix/p) for p in names},
    observer_sources={p.name:sha(p) for p in Path('/source/containers/r5_dds_observer').iterdir() if p.is_file()},
    release_provenance=json.loads(Path('/evidence/source-provenance.json').read_text()),
    baseline_segment_bytes=524288,candidate_bridge_segment_bytes=8388608,queue_capacity=8192)
(prefix/'manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
Path('/evidence/'+prefix.name+'-manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
stock=Path('/opt/ros/jazzy/lib')
Path('/evidence/stock-library-hashes.json').write_text(json.dumps({str(p):sha(p) for p in stock.glob('lib*.so*')
    if any(n in p.name for n in ('fastrtps','fastcdr','rmw_fastrtps'))},indent=2)+'\n')
