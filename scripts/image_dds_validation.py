"""Read-only acceptance helpers; model instrumentation is confined to the test overlay."""
import hashlib, json, os, signal, subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

ET.register_namespace('xacro','http://www.ros.org/wiki/xacro')
OBSERVERS={'gz::sim::systems::PosePublisher','mooncar::StepObserver'}
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def observed_model(source,output,steps=None):
    tree=ET.parse(source);gz=ET.SubElement(tree.getroot(),'gazebo')
    pose=ET.SubElement(gz,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher')
    for k,v in dict(publish_link_pose='true',publish_model_pose='true',publish_sensor_pose='true',use_pose_vector_msg='true',update_frequency='100',static_publisher='false').items():ET.SubElement(pose,k).text=v
    if steps:
        plugin=ET.SubElement(gz,'plugin',filename='/evidence/observer/libmooncar-step-observer.so',name='mooncar::StepObserver')
        ET.SubElement(plugin,'output').text=str(steps)
    tree.write(output,encoding='utf-8',xml_declaration=True)

def semantic(element):
    return (element.tag,tuple(sorted(element.attrib.items())),(element.text or '').strip(),tuple(semantic(c) for c in element))

def strip_observers(tree):
    for parent in tree.iter():
        for child in list(parent):
            if child.tag=='plugin' and child.get('name') in OBSERVERS:parent.remove(child)
    # URDF instrumentation uses a new global gazebo element, now empty.
    for child in list(tree):
        if child.tag=='gazebo' and not len(child) and not child.attrib:tree.remove(child)
    return tree

def model_preflight(folder):
    folder=Path(folder);folder.mkdir()
    result={}
    for name in ('mooncar_r5.urdf.xacro','mooncar.urdf.xacro'):
        source=Path('/work/install/share/mooncar_description/urdf')/name
        output=folder/name;observed_model(source,output,folder/'steps.csv' if 'r5' in name else None)
        converted={}
        for kind,path in (('original',source),('observed',output)):
            urdf=folder/(name+'.'+kind+'.urdf')
            text=subprocess.check_output(['xacro',str(path),'use_sim_time:=true'],text=True);urdf.write_text(text)
            sdf=folder/(name+'.'+kind+'.sdf')
            sdf.write_text(subprocess.check_output(['gz','sdf','-p',str(urdf)],text=True))
            converted[kind]=(ET.parse(urdf).getroot(),ET.parse(sdf).getroot())
        for i,format in enumerate(('urdf','sdf')):
            if semantic(strip_observers(converted['observed'][i]))!=semantic(converted['original'][i]):
                raise RuntimeError('Observation changed model '+format+': '+name)
        result[name]=dict(source_sha256=sha(source),observed_sha256=sha(output),urdf_equal=True,sdf_equal=True)
        if 'r5' in name:
            for i,format in enumerate(('urdf','sdf')):
                if semantic(converted['original'][i])!=semantic(ET.parse('/evidence/r5-before.'+format).getroot()):
                    raise RuntimeError('Archived R5 baseline changed: '+format)
    return result

def installed_files(source='/source',prefix='/work/install'):
    source=Path(source);prefix=Path(prefix);files={};errors=[]
    for package,parts in {'mooncar_description':('urdf','launch'),'mooncar_gazebo':('launch','config','worlds','models','materials')}.items():
        for part in parts:
            for path in (source/'ros2_ws/src'/package/part).rglob('*'):
                if not path.is_file() or '__pycache__' in path.parts:continue
                rel=path.relative_to(source/'ros2_ws/src'/package)
                target=prefix/'share'/package/rel
                expected=source/'containers/image_dds/auto-defaults.json' if str(rel)=='config/image_dds_defaults.json' else path
                if not target.is_file() or sha(target)!=sha(expected):errors.append(str(target))
                else:files[str(target)]=sha(target)
    for name in ('cmd_vel_safety_relay.py','wheel_odometry.py'):
        target=prefix/'lib/mooncar_control'/name;expected=source/'ros2_ws/src/mooncar_control/scripts'/name
        if not target.is_file() or sha(target)!=sha(expected) or not os.access(target,os.X_OK):errors.append(str(target))
        else:files[str(target)]=sha(target)
    if errors:raise RuntimeError('Installed files differ: '+repr(errors))
    return files

def receiver_snapshot(pid=None):
    pid=pid or os.getpid();maps=Path(f'/proc/{pid}/maps').read_text()
    paths={line.split()[-1] for line in maps.splitlines() if '/' in line}
    libraries=sorted(p for p in paths if any(n in p for n in ('librmw_fastrtps','libfastrtps')))
    segments=[dict(path=p,bytes=Path(p).stat().st_size) for p in paths if p.startswith('/dev/shm/') and Path(p).is_file()]
    return dict(pid=pid,libraries=libraries,segments=segments,verified=bool(
        any(s['bytes']==549408 for s in segments) and
        not any(s['bytes']==8782368 for s in segments) and
        any('librmw_fastrtps_cpp.so' in p for p in libraries) and
        all(p.startswith('/opt/ros/jazzy/lib/') for p in libraries)))

def participants(bridge_pid=None):
    result={}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit() or int(path.name)==bridge_pid:continue
        try:
            maps=(path/'maps').read_text()
            if 'librmw_fastrtps_cpp.so' not in maps:continue
            row=receiver_snapshot(int(path.name))
            environment=(path/'environ').read_bytes().split(b'\0')
            row['capacity_environment_leaked']=any(v.startswith((b'MOONCAR_IMAGE_DDS_TX_BYTES=',b'MOONCAR_IMAGE_DDS_RECEIPT=')) for v in environment)
            row['verified']=row['verified'] and not row['capacity_environment_leaked']
            result[path.name]=row
        except (FileNotFoundError,ProcessLookupError):continue
    return result

def image_files(folder,records):
    import numpy as np
    from PIL import Image
    selected=[r for r in records if r['kind'] in ('rgb','depth')];errors=[];content=[]
    for row in selected:
        try:
            path=Path(folder)/row['file']
            if row.get('archive_completed_wall') is None:raise RuntimeError('No image completion receipt')
            if row['kind']=='rgb':
                with Image.open(path) as image:array=np.asarray(image.convert('RGB'));image.load()
                valid=array.ndim==3 and array.shape[2]==3 and float(array.std())>0
                content.append(dict(file=row['file'],pixel_std=float(array.std()),valid=bool(valid)))
            else:
                with np.load(path) as archive:array=archive['depth'];valid=bool((np.isfinite(array)&(array>0)).any())
                content.append(dict(file=row['file'],finite_positive=int((np.isfinite(array)&(array>0)).sum()),valid=valid))
            if not valid:raise RuntimeError('No inspectable content')
        except Exception as error:errors.append(dict(file=row.get('file'),reason=repr(error)))
    return dict(passed=bool(selected and not errors and {r['kind'] for r in selected}=={'rgb','depth'}),errors=errors,content=content)

def stop_process(process,timeout=8):
    if process is None:return
    if process.poll() is None:
        try:os.killpg(process.pid,signal.SIGINT)
        except ProcessLookupError:pass
    try:process.wait(timeout)
    except subprocess.TimeoutExpired:
        try:os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        process.wait()
