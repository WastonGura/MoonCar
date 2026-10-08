"""One Gazebo query worker, while the caller continues ROS reception/heartbeat."""
import concurrent.futures, subprocess, threading, time

class QueryWorker:
    def __init__(self):
        self.pool=concurrent.futures.ThreadPoolExecutor(max_workers=1,thread_name_prefix='gazebo-query')
        self.future=None; self.child=None; self.cancelled=threading.Event(); self.timeline=[]
    def submit(self, function, *args):
        if self.future is not None and not self.future.done(): raise RuntimeError('Gazebo query already active')
        self.future=self.pool.submit(function,*args); return self.future
    def command(self, argv, deadline):
        if self.cancelled.is_set(): raise RuntimeError('Query cancelled')
        begin=time.monotonic(); self.child=subprocess.Popen(argv,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            stdout,stderr=self.child.communicate(timeout=max(.001,deadline-time.monotonic()))
            row=dict(argv=argv,started_wall=begin,ended_wall=time.monotonic(),stdout=stdout,stderr=stderr,exit_code=self.child.returncode)
            self.timeline.append(row)
            if self.child.returncode: raise RuntimeError('Gazebo query failed: '+repr(row))
            return stdout
        except subprocess.TimeoutExpired:
            self.child.kill(); stdout,stderr=self.child.communicate()
            self.timeline.append(dict(argv=argv,started_wall=begin,ended_wall=time.monotonic(),stdout=stdout,stderr=stderr,timeout=True))
            raise TimeoutError('Gazebo pause/query combined deadline exceeded')
        finally: self.child=None
    def close(self):
        self.cancelled.set()
        child=self.child
        if child and child.poll() is None: child.kill()
        self.pool.shutdown(wait=True,cancel_futures=True)

def wait_query(worker, function, pump, deadline, *args):
    future=worker.submit(function,*args)
    while not future.done():
        if time.monotonic()>=deadline:
            worker.close(); raise TimeoutError('Query deadline exceeded while ROS reception continued')
        pump()
    return future.result()

def actual_pause(worker, world, pump, deadline, parse, last_message, minimum_sim=0.):
    started=time.monotonic()
    def query():
        answer=worker.command(['gz','service','-s','/world/'+world+'/control','--reqtype','gz.msgs.WorldControl',
            '--reptype','gz.msgs.Boolean','--timeout','5000','--req','pause: true'],deadline)
        if 'data: true' not in answer: raise RuntimeError('Pause request rejected')
        while time.monotonic()<deadline:
            raw=last_message(worker.command(['gz','topic','-n','1','-e','--json-output','-t','/world/'+world+'/stats'],deadline))
            # A delayed packet from before the request is not pause confirmation.
            if raw.get('paused') is True:
                state=parse(raw,time.monotonic())
                if state['sim']>=minimum_sim: return dict(state=state,raw=raw,request_response=answer,
                                                         started_wall=started,ended_wall=time.monotonic())
        raise TimeoutError('Actual pause not confirmed')
    return wait_query(worker,query,pump,deadline)
