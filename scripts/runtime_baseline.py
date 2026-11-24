#!/usr/bin/env python3
"""Freeze committed product sources and verify the installed simulation runtime."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]
PRODUCT_DIRS=('config','containers','firmware','ros2_ws/src','scripts','web')

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        while block:=stream.read(4*1024*1024):h.update(block)
    return h.hexdigest()

def write(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def freeze(root,output,allow_dirty=False):
    def git(*args):return subprocess.check_output(['git','-C',str(root),*args])
    dirty=git('status','--porcelain=v1','--untracked-files=normal','--',*PRODUCT_DIRS).decode().strip()
    if dirty and not allow_dirty:raise RuntimeError('Product source is not committed; commit it or explicitly use --allow-dirty')
    names=[p.decode() for p in git('ls-files','-z','--',*PRODUCT_DIRS).split(b'\0') if p]
    files={n:sha(root/n) for n in names if (root/n).is_file()}
    baseline=json.loads((root/'config/simulation_baseline.json').read_text(encoding='utf-8'))
    identity=dict(git_commit=git('rev-parse','HEAD').decode().strip(),
        git_branch=git('branch','--show-current').decode().strip(),source_root=str(root),
        product_clean=not bool(dirty),baseline=baseline,
        source_digest=hashlib.sha256(json.dumps(files,sort_keys=True).encode()).hexdigest())
    value=dict(schema_version=1,identity=identity,source_sha256=files)
    write(output,value);return value

def verify_sources(root,manifest):
    for name,expected in manifest['source_sha256'].items():
        path=root/name
        if not path.is_file() or sha(path)!=expected:raise RuntimeError('Source changed: '+name)

def install_prefix(install,package):
    return install/package if (install/package/'share'/package).is_dir() else install

def verify_pair(source,target):
    if not target.is_file() or sha(source)!=sha(target):raise RuntimeError('Installed file differs: '+str(target))
    return dict(installed=str(target),sha256=sha(source))

def verify(root,manifest,install,output):
    verify_sources(root,manifest)
    pairs=[];slam=install_prefix(install,'mooncar_slam')/'lib/mooncar_slam'
    pairs += [(root/'scripts'/n,slam/n) for n in ('rgbd_window_odometry.py','rgbd_window_shadow.py','localization_quality.py')]
    pairs += [(p,slam/'offline_rgbd_window'/p.name) for p in (root/'scripts/offline_rgbd_window').glob('*.py')]
    terrain_install = install_prefix(install,'mooncar_terrain')/'lib/mooncar_terrain'
    pairs += [(root/'ros2_ws/src/mooncar_terrain/scripts'/name, terrain_install/name)
              for name in ('terrain_geometry.py', 'terrain_map.py', 'map_recorder.py')]
    pairs += [(root/'scripts/literature_terrain.py',
               install_prefix(install,'mooncar_terrain')/'lib/mooncar_terrain/literature_terrain.py')]
    for package,parts in [('mooncar_description',['urdf']),('mooncar_gazebo',['launch','config','worlds','models','materials']),('mooncar_navigation',['launch','config'])]:
        prefix=install_prefix(install,package)/'share'/package
        for part in parts:
            source=root/'ros2_ws/src'/package/part
            for path in source.rglob('*'):
                if path.is_file() and '__pycache__' not in path.parts and path.suffix!='.pyc':
                    pairs.append((path,prefix/path.relative_to(root/'ros2_ws/src'/package)))
    installed={str(a.relative_to(root)):verify_pair(a,b) for a,b in pairs}
    spec=importlib.util.spec_from_file_location('verified_support_profile',root/'ros2_ws/src/mooncar_gazebo/launch/support_profile.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    support=module.verify(manifest['identity']['baseline']['support_runtime'])
    receipt=dict(schema_version=1,verified=True,identity=manifest['identity'],installed=installed,
                 support_libraries=support['libraries'],dart_sha256=support['dart_sha256'])
    write(output,receipt);return receipt

def load_identity(path):
    if not path:return dict(verified=False)
    receipt=json.loads(Path(path).read_text(encoding='utf-8'))
    if receipt.get('verified') is not True:raise RuntimeError('Runtime baseline is not verified')
    return dict(verified=True,**receipt['identity'])

def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='operation',required=True)
    f=sub.add_parser('freeze');f.add_argument('--output',required=True,type=Path);f.add_argument('--allow-dirty',action='store_true')
    v=sub.add_parser('verify');v.add_argument('--manifest',required=True,type=Path);v.add_argument('--install',required=True,type=Path);v.add_argument('--output',required=True,type=Path)
    args=p.parse_args()
    value=(freeze(ROOT,args.output,args.allow_dirty) if args.operation=='freeze' else
        verify(ROOT,json.loads(args.manifest.read_text(encoding='utf-8')),args.install,args.output))
    print(json.dumps(dict(operation=args.operation,git_commit=value.get('identity',{}).get('git_commit'),
                         source_files=len(value.get('source_sha256',{})),installed_files=len(value.get('installed',{})))))

if __name__=='__main__':main()
