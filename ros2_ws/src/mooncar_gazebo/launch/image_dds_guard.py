"""Bridge-only, fail-closed image DDS runtime selection and process verification."""
import hashlib, json, os, re, signal, subprocess, sys, tempfile, time
from pathlib import Path
import xml.etree.ElementTree as ET

WORLDS=('mooncar_ramp_v2','mooncar_curve_v2','mooncar_survey_v2')
VERSIONS={'fastdds':'2.14.6','rmw_fastrtps_shared_cpp':'8.4.4','rmw_fastrtps_cpp':'8.4.4'}
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def default_profile(path):
    value=json.loads(Path(path).read_text())
    if set(value)!={'profile'} or value['profile'] not in ('auto','legacy'):
        raise RuntimeError('Invalid installed image DDS default configuration')
    return value['profile']

def complete_receipt(path):
    try:return json.loads(Path(path).read_text())
    except (FileNotFoundError,json.JSONDecodeError):return None

def select(model,world,env,default='legacy'):
    requested=env.get('MOONCAR_IMAGE_DDS_PROFILE',default)
    if requested not in ('auto','legacy','shm_tx8m'):raise RuntimeError('Invalid MOONCAR_IMAGE_DDS_PROFILE')
    discovery=env.get('ROS_AUTOMATIC_DISCOVERY_RANGE','SUBNET').upper()
    # The legacy variable can override discovery; explicit conflicting inputs are unsupported.
    compatible=(model=='mooncar_r5.urdf.xacro' and ET.parse(world).find('world').get('name') in WORLDS
                and discovery=='LOCALHOST' and env.get('ROS_LOCALHOST_ONLY','0') in ('0','1',''))
    if requested=='shm_tx8m' and not compatible:raise RuntimeError('shm_tx8m requires R5/v2 and LOCALHOST discovery')
    selected=('shm_tx8m' if compatible else 'legacy') if requested=='auto' else requested
    return dict(requested=requested,selected=selected,eligible=compatible,discovery=discovery)

def verify(root):
    root=Path(root).resolve();m=json.loads((root/'manifest.json').read_text())
    if m.get('versions')!=VERSIONS or m.get('capacity_bytes')!=8388608 or m.get('diagnostic_instrumentation') is not False:
        raise RuntimeError('Image DDS runtime manifest incompatible')
    installed={name:ET.parse(Path('/opt/ros/jazzy/share')/name/'package.xml').findtext('version')
        for name in ('rmw_fastrtps_shared_cpp','rmw_fastrtps_cpp')}
    version_file=Path('/opt/ros/jazzy/share/fastrtps/cmake/fastrtps-config-version.cmake').read_text()
    version=re.search(r'set\(PACKAGE_VERSION "([^"]+)"\)',version_file)
    installed['fastdds']=version.group(1) if version else None
    if installed!=VERSIONS:raise RuntimeError('Installed DDS/RMW version mismatch')
    for name in ('lib/librmw_fastrtps_cpp.so','lib/librmw_fastrtps_shared_cpp.so'):
        if sha(root/name)!=m['libraries'].get(name):raise RuntimeError('Image DDS runtime library hash mismatch')
    if sha(m['stock_fastdds_path'])!=m['stock_fastdds_sha256']:raise RuntimeError('Installed Fast DDS changed')
    return m

def bridge_env(base,root,receipt):
    env=dict(base);verify(root)
    if env.get('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')!='rmw_fastrtps_cpp':raise RuntimeError('Unsupported RMW')
    if env.get('ROS_AUTOMATIC_DISCOVERY_RANGE','').upper()!='LOCALHOST':raise RuntimeError('LOCALHOST discovery required')
    if any(k in env for k in ('MOONCAR_DDS_TRACE','MOONCAR_DDS_SHM_TX_BYTES','MOONCAR_IMAGE_BRIDGE_TRACE')):
        raise RuntimeError('Diagnostic DDS environment cannot enter production bridge')
    env['LD_LIBRARY_PATH']=str(Path(root)/'lib')+':'+env.get('LD_LIBRARY_PATH','')
    env['MOONCAR_IMAGE_DDS_TX_BYTES']='8388608';env['MOONCAR_IMAGE_DDS_RECEIPT']=str(receipt)
    return env

def process_receipt(pid,receipt,root):
    row=json.loads(Path(receipt).read_text());manifest=verify(root)
    if row.get('pid')!=pid or row.get('segment_bytes')!=8388608 or row.get('port_queue_capacity')!=512 or row.get('user_transports')!=2 or row.get('builtin_transports') is not False or not row.get('participant_created'):
        raise RuntimeError('Actual DDS initialization mismatch')
    paths=set(line.split()[-1] for line in Path(f'/proc/{pid}/maps').read_text().splitlines() if '/' in line)
    for n,digest in manifest['libraries'].items():
        expected=str((Path(root)/n).resolve())
        if expected not in paths or sha(expected)!=digest:raise RuntimeError('Custom RMW not actually loaded')
    if manifest['stock_fastdds_path'] not in paths:raise RuntimeError('Stock Fast DDS not actually loaded')
    if any(Path(n).name.startswith('libfastrtps.so') and n!=manifest['stock_fastdds_path'] for n in paths):
        raise RuntimeError('Unexpected additional Fast DDS library loaded')
    segments=[dict(path=n,bytes=Path(n).stat().st_size) for n in paths if n.startswith('/dev/shm/') and Path(n).is_file()]
    # Fast DDS 2.14.6 segment payload plus its allocation metadata, recorded separately.
    if not any(n['bytes']==8782368 for n in segments):raise RuntimeError('8 MiB shared segment mapping absent')
    return dict(**row,libraries=sorted(paths),segments=segments,manifest_sha256=sha(Path(root)/'manifest.json'))

def supervise(binary,argv,root,folder,timeout=10):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    receipt=folder/'initialization.json';child=None
    def forward(sig,_):
        if child and child.poll() is None:child.send_signal(sig)
    old={s:signal.signal(s,forward) for s in (signal.SIGINT,signal.SIGTERM)}
    try:
        child=subprocess.Popen([binary]+argv,env=bridge_env(os.environ,root,receipt))
        end=time.monotonic()+timeout
        while complete_receipt(receipt) is None:
            if child.poll() is not None:raise RuntimeError('Bridge exited before DDS receipt')
            if time.monotonic()>end:raise TimeoutError('Bridge DDS receipt timeout')
            time.sleep(.02)
        # Writer flush/close and SHM mapping precede eligibility to report ready.
        proof=process_receipt(child.pid,receipt,root)
        (folder/'ready.json').write_text(json.dumps(proof,indent=2)+'\n')
        print('MoonCar image DDS ready: 8 MiB bridge segment, stock Fast DDS',flush=True)
        return child.wait()
    finally:
        if child and child.poll() is None:
            child.terminate()
            try:child.wait(5)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        for s,handler in old.items():signal.signal(s,handler)

def main():
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--receipt-directory',required=True);p.add_argument('--binary',required=True)
    a,argv=p.parse_known_args()
    try:return supervise(a.binary,argv,a.root,a.receipt_directory)
    except Exception as error:print('MoonCar image DDS startup failed: '+repr(error),file=sys.stderr,flush=True);return 1

if __name__=='__main__':raise SystemExit(main())
