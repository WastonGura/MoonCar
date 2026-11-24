"""No Gazebo: trace unchanged bridge guard errors, never suppress/retry them."""
import errno,importlib.util,json,os,queue,signal,subprocess,sys,threading,time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
p=Path('/evidence/startup-probe-stable');p.mkdir()
os.environ['GZ_PARTITION']='mooncar_no_gazebo_startup_probe_20261011'
spec=importlib.util.spec_from_file_location('stock_guard',str(Path(get_package_share_directory('mooncar_gazebo'))/'launch/image_dds_guard.py'))
guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)
rows=queue.Queue(8192);errors=[];stop=object()
def write():
    try:
        with (p/'exceptions.jsonl').open('x') as f:
            while True:
                r=rows.get()
                if r is stop:break
                f.write(json.dumps(r)+'\n');f.flush()
    except Exception as e:errors.append(repr(e))
writer=threading.Thread(target=write);writer.start()
def trace(frame,event,arg):
    if event=='exception' and isinstance(arg[1],OSError):
        e=arg[1]
        if e.errno==errno.ENODATA:
            r=dict(wall=time.time(),file=frame.f_code.co_filename,line=frame.f_lineno,
                function=frame.f_code.co_name,error=repr(e),path=str(frame.f_locals.get('self','')))
            try:rows.put_nowait(r)
            except queue.Full:errors.append('Trace queue overflow')
    return trace
original=subprocess.Popen;process=[None]
def capture(*args,**kwargs):
    process[0]=original(*args,**kwargs);return process[0]
results=[]
try:
    for i in range(4):
        folder=p/f'{i+1:02d}';process[0]=None;cancel=threading.Event()
        def terminate_after_ready():
            deadline=time.monotonic()+12
            ready_at=None
            while not cancel.wait(.02):
                child=process[0]
                if child is None:continue
                if ready_at is None and (folder/'ready.json').is_file():ready_at=time.monotonic()
                if (ready_at is not None and time.monotonic()-ready_at>=1.) or time.monotonic()>deadline:
                    if child.poll() is None:child.send_signal(signal.SIGINT)
                    return
        terminator=threading.Thread(target=terminate_after_ready);terminator.start()
        guard.subprocess.Popen=capture;sys.settrace(trace)
        start=time.monotonic()
        try:
            code=guard.supervise('/opt/ros/jazzy/lib/ros_gz_bridge/bridge_node',
                ['--ros-args','-r','__node:=mooncar_startup_probe','-p',
                 'config_file:=/evidence/campaign/11-pure_turn-flat-500us-zero/bridge.yaml'],
                '/opt/mooncar/image-dds/rmw-8.4.4',folder)
            result=dict(attempt=i+1,code=code,ready=(folder/'ready.json').is_file())
        except Exception as e:result=dict(attempt=i+1,ready=False,error=repr(e))
        finally:
            sys.settrace(None);guard.subprocess.Popen=original;cancel.set();terminator.join()
        result['wall_seconds']=time.monotonic()-start;results.append(result)
        if not result['ready'] or result.get('code')!=0:break
finally:
    guard.subprocess.Popen=original;sys.settrace(None);rows.put(stop);writer.join()
    result=dict(no_gazebo=True,physics_attempts=0,trace_changes_exception_flow=False,
        guard_source_unchanged=True,bridge_capacity_unchanged=True,results=results,trace_errors=errors,
        observed_enodata=bool((p/'exceptions.jsonl').stat().st_size),
        verdict='Reproduced only if exact ENODATA trace present; old failed round remains failed')
    (p/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
