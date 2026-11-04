"""Use the real installed launch checker, then require its scoped runtime proof."""
import argparse
import json
from pathlib import Path
import threading
import time
import sys

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--name',required=True)
    p.add_argument('--legacy',action='store_true');a=p.parse_args();a.scene='ramp'
    folder=Path(a.output)/a.name
    sys.path.insert(0,'/work/install/share/mooncar_gazebo/launch')
    from turn_guard import verify
    manifest=verify('/opt/mooncar/turn/r5-v2')
    stopped=threading.Event();seen={};errors=[]
    def observe():
        try:
            while not stopped.wait(.2):
                for entry in Path('/proc').iterdir():
                    if not entry.name.isdigit() or entry.name in seen:continue
                    try:
                        paths={x.split()[-1] for x in (entry/'maps').read_text().splitlines() if '/opt/mooncar/turn/' in x}
                        command=(entry/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
                    except (FileNotFoundError,ProcessLookupError):continue
                    if paths:seen[entry.name]=dict(command=command,paths=sorted(paths))
        except Exception as error:errors.append(repr(error))
    worker=threading.Thread(target=observe,daemon=True);worker.start()
    code=1;failure=None
    try:
        import check_image_dds_launch
        code=check_image_dds_launch.run(a)
    except Exception as error:failure=repr(error)
    finally:stopped.set();worker.join(3)
    raw=folder/'result.json';result=json.loads(raw.read_text()) if raw.exists() else {'passed':False}
    proof=[]
    for line in (folder/'launch.log').read_text(errors='replace').splitlines() if (folder/'launch.log').exists() else []:
        marker='MoonCar steering runtime ready: '
        if marker in line:
            proof.append(json.JSONDecoder().raw_decode(line.split(marker,1)[1])[0])
    expected=set(manifest['libraries'].values())|{manifest['dart_sha256']}
    runtime_ok=not seen and not proof if a.legacy else bool(len(proof)==1 and set(proof[0]['libraries'].values())==expected and seen)
    isolated=all('gz ' in r['command'] or 'gz-sim' in r['command'] for r in seen.values())
    passed=bool(result.get('passed') and runtime_ok and isolated and not errors and not failure and code==0)
    (Path(a.output)/(a.name+'-turn-release.json')).write_text(json.dumps(dict(passed=passed,legacy=a.legacy,
        runtime_verified=runtime_ok,environment_isolated=isolated,loaded_processes=seen,actual_runtime_proof=proof,
        monitor_errors=errors,error=failure,underlying_passed=result.get('passed'),exit_code=code),indent=2)+'\n')
    print(json.dumps(dict(name=a.name,passed=passed,runtime_verified=runtime_ok,error=failure)),flush=True)
    return 0 if passed else 1

if __name__=='__main__':raise SystemExit(main())
