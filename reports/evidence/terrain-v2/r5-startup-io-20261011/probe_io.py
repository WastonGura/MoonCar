"""No ROS/Gazebo: exercise unchanged receipt parser against concurrent flush."""
import hashlib,importlib.util,json,subprocess,time,traceback
from pathlib import Path
path=Path('/source/ros2_ws/src/mooncar_gazebo/launch/image_dds_guard.py')
expected='a86a52b9ec2cfb55baf748ac08abb139be92a9e5a8e5b4ed318325a04fee6d6c'
assert hashlib.sha256(path.read_bytes()).hexdigest()==expected
spec=importlib.util.spec_from_file_location('guard',path);guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)
summary=[]
with Path('/evidence/io-events.jsonl').open('x') as log:
    for root in (Path('/evidence/windows-receipts'),Path('/work/linux-receipts')):
        root.mkdir(parents=True);events=0;errors=[];empty=0;valid=0
        proc=subprocess.Popen(['/work/receipt-writer',str(root)],stdout=subprocess.PIPE,text=True)
        for line in proc.stdout:
            filename=line.strip();deadline=time.monotonic()+.1;selected=None
            while time.monotonic()<deadline:
                try:
                    row=guard.complete_receipt(filename);selected=row
                    if row is None:empty+=1
                    else:valid+=1
                    event=dict(path=filename,wall=time.time(),complete=row is not None)
                except OSError as error:
                    event=dict(path=filename,wall=time.time(),error=repr(error),errno=error.errno,traceback=traceback.format_exc())
                    errors.append(event)
                log.write(json.dumps(event)+'\n');events+=1
                if selected is not None:break
                time.sleep(.0001)
            if selected is None:errors.append(dict(path=filename,error='No complete receipt within100ms'))
        code=proc.wait();summary.append(dict(root=str(root),exit_code=code,reads=events,pending=empty,complete=valid,errors=errors))
result=dict(no_ros=True,no_gazebo=True,simulations=0,guard_sha256=expected,
    controlled_write_delay_s=.001,tests=summary,
    enodata_reproduced=any(e.get('errno')==61 for s in summary for e in s['errors']))
Path('/evidence/io-summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
