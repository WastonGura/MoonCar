"""Separate stock bridge lifecycle from the RMW-only native-node test."""
import json,os,signal,subprocess,sys,time
from pathlib import Path
p=Path('/evidence');results=[]
for label in ('stock','minimal'):
    log_path=p/f'bridge-{label}.log';folder=p/f'bridge-{label}'
    args=['/opt/ros/jazzy/lib/ros_gz_bridge/bridge_node','--ros-args','-r',f'__node:=mooncar_lifecycle_{label}',
        '-p','config_file:=/source/ros2_ws/src/mooncar_gazebo/config/ros_gz_bridge_r5.yaml']
    if label=='minimal':args=[sys.executable,'/evidence/test-guard.py','--root','/opt/mooncar/image-dds/rmw-8.4.4',
        '--receipt-directory',str(folder),'--binary',args[0]]+args[1:]
    env=dict(os.environ);env['GZ_PARTITION']='mooncar_no_gazebo_bridge_close_20261011_'+label
    with log_path.open('x') as log:
        child=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        start=time.monotonic();last_time=None;previous='';ready=False
        while time.monotonic()-start<15 and child.poll() is None:
            text=log_path.read_text()
            if text!=previous:previous=text;last_time=time.monotonic()
            if text.count('Creating ')>=8 and last_time and time.monotonic()-last_time>=3.:ready=True;break
            time.sleep(.05)
        # Signal only this diagnostic's own process group.
        if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
        try:code=child.wait(5)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);code=child.wait()
    results.append(dict(label=label,bridge_routes_created=previous.count('Creating '),stable_after_routes=ready,
        exit_code=code,wall_seconds=time.monotonic()-start,
        proof=json.loads((folder/'ready.json').read_text()) if (folder/'ready.json').is_file() else None))
(p/'bridge-close-summary.json').write_text(json.dumps(dict(no_gazebo=True,simulations=0,results=results),indent=2)+'\n')
print(json.dumps([{k:v for k,v in row.items() if k!='proof'} for row in results]))
