"""Bind diagnostic ROS kinematics to the actual frozen support model."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

def description_path(args):
    return Path(getattr(args,'robot_description_urdf',None) or Path(args.model).with_suffix('.urdf'))

def description_parameters(args):
    result=dict(robot_description=description_path(args).read_text(),use_sim_time=True)
    frequency=getattr(args,'robot_state_publish_frequency_hz',None)
    if frequency is not None:
        if not math.isfinite(frequency) or frequency<=0:raise ValueError('Invalid robot state publication frequency')
        result['publish_frequency']=frequency
    return result

def validate_description(world,urdf):
    tree=ET.parse(world);model=tree.find("world/model[@name='mooncar']")
    if model is None:model=tree.find('model')
    robot=ET.parse(urdf)
    actual={j.get('name'):j for j in model.findall('joint') if j.get('type')!='fixed'}
    described={j.get('name'):j for j in robot.findall('joint') if j.get('type')!='fixed'}
    if set(actual)!=set(described):raise ValueError('Robot description dynamic joints differ from physical support model')
    for name,j in actual.items():
        other=described[name];kind='revolute' if other.get('type')=='continuous' else other.get('type')
        if kind!=j.get('type') or other.find('child').get('link')!=j.findtext('child'):
            raise ValueError('Robot description joint type/child mismatch: '+name)
        if name.endswith('_suspension_joint'):
            if tuple(map(float,other.find('axis').get('xyz').split()))!=tuple(map(float,j.findtext('axis/xyz').split())):raise ValueError('Suspension TF axis mismatch')
            for field in ('lower','upper'):
                if abs(float(other.find('limit').get(field))-float(j.findtext('axis/limit/'+field)))>1e-12:
                    raise ValueError('Suspension TF travel mismatch')
    return dict(passed=True,urdf=str(urdf),sha256=hashlib.sha256(Path(urdf).read_bytes()).hexdigest(),dynamic_joints=sorted(actual))

def validate_bound_description(spec):
    if spec.get('robot_state_publish_frequency_hz')!=30.:
        raise ValueError('Research TF publication must match the 30 Hz joint throttle')
    path=spec.get('robot_description_urdf')
    if not path:raise ValueError('Frozen robot description binding missing')
    if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=spec.get('robot_description_sha256'):
        raise ValueError('Frozen robot description hash mismatch')
    return validate_description(spec['world'],path)

def qualify_feedback(folder,urdf):
    robot=ET.parse(urdf);expected=[j for j in robot.findall('joint') if j.get('type')!='fixed']
    joints={};transforms={}
    with (Path(folder)/'observations.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line)
            if row['kind']=='joints':
                joints[row.get('stamp_ns',round(row['stamp']*1e9))]=dict(zip(row['names'],row['positions']))
            elif row['kind']=='robot_tf':
                for tf in row['transforms']:transforms.setdefault(tf['stamp_ns'],{})[tf['child']]=tf
    matched=0;maximum_error=0.;stamps=[]
    for stamp,positions in joints.items():
        tf=transforms.get(stamp,{})
        if not all(j.get('name') in positions and j.find('child').get('link') in tf for j in expected):continue
        for j in expected:
            actual=tf[j.find('child').get('link')]
            if actual['parent']!=j.find('parent').get('link'):raise ValueError('Runtime TF parent mismatch')
            if j.get('type')=='prismatic':
                origin=j.find('origin');xyz=list(map(float,origin.get('xyz','0 0 0').split()))
                rpy=list(map(float,origin.get('rpy','0 0 0').split()))
                if any(abs(a)>1e-12 for a in rpy):raise ValueError('Unsupported rotated prismatic origin for this research candidate')
                axis=list(map(float,j.find('axis').get('xyz').split()));q=positions[j.get('name')]
                target=[a+q*b for a,b in zip(xyz,axis)]
                maximum_error=max(maximum_error,max(abs(a-b) for a,b in zip(target,actual['translation'])))
                rotation=actual['rotation_xyzw']
                if max(abs(a) for a in rotation[:3])>1e-9 or abs(abs(rotation[3])-1)>1e-9:raise ValueError('Prismatic TF rotation mismatch')
        matched+=1;stamps.append(stamp)
    gap=max((b-a)/1e9 for a,b in zip(stamps,stamps[1:])) if len(stamps)>1 else math.inf
    return dict(passed=matched>1 and gap<=.100001 and maximum_error<=1e-9,
        matched_joint_tf_samples=matched,maximum_tf_translation_error_m=maximum_error,
        maximum_matched_gap_s=gap if math.isfinite(gap) else None,dynamic_joints=len(expected))
