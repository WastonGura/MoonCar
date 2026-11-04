"""Test-only integer-step scheduling through the unchanged ROS command path."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


def step_count(seconds,step):
    if seconds<=0 or step<=0:raise ValueError('Duration and step must be positive')
    count=round(seconds/step)
    if abs(count*step-seconds)>1e-9:
        raise ValueError('Duration must contain a positive integer number of physical steps')
    return count


def confirmed(rows,kind,target,after):
    samples=[r for r in rows if r['kind']==kind and r['wall']>=after]
    return len(samples)>=2 and all(len(r['values'])==len(target) and max(abs(a-b) for a,b in zip(r['values'],target))<=1e-12 for r in samples[-2:])


def pause_marker(path,iteration,sim_time):
    try:
        values=Path(path).read_text().strip().split(',')
        return int(values[0])==iteration and abs(float(values[1])-sim_time)<=1e-8
    except (OSError,ValueError,IndexError):return False


def stats_state(value,wall):
    stamp=value.get('simTime',value.get('sim_time'))
    if stamp is None:raise ValueError('Missing actual Gazebo simulation time')
    return dict(iteration=int(value.get('iterations',0)),paused=bool(value.get('paused',False)),
        sim=float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9,wall=wall)


def last_step(path):
    with Path(path).open('rb') as stream:
        header=stream.readline().decode().strip().split(',')
        stream.seek(max(0,Path(path).stat().st_size-16384))
        line=stream.read().decode().strip().splitlines()[-1]
    # Velocity-driven wheels have no JointForceCmd component. Preserve their
    # unavailable command as null in JSON, while retaining the untouched CSV.
    return {name:(value if math.isfinite(value) else None) for name,value in zip(header,map(float,line.split(',')))}


class PausedSteps:
    def __init__(self,node,executor,processes,env,observer_path,step,deadline,command,ready):
        from geometry_msgs.msg import Twist
        self.Twist=Twist;self.node=node;self.executor=executor;self.processes=processes
        self.observer=Path(observer_path);self.step=step;self.deadline=deadline
        self.command=command;self.ready=ready;self.phases=[];self.latest=None
        self.lock=threading.Lock();self.condition=threading.Condition();self.target=[0.]*6
        self.stop=threading.Event();self.failure=None
        self.err=(node.folder/'world-stats.stderr').open('w')
        self.stats_proc=subprocess.Popen(['gz','topic','-e','--json-output','-t','/world/r5_check/stats'],
            stdout=subprocess.PIPE,stderr=self.err,text=True,env=env,start_new_session=True)
        self.threads=[threading.Thread(target=f,daemon=True) for f in (self.read_stats,self.spin,self.heartbeat)]
        for thread in self.threads:thread.start()

    def read_stats(self):
        try:
            for line in self.stats_proc.stdout:
                value=json.loads(line);state=stats_state(value,time.monotonic())
                self.node.record('world_stats',state=state,raw=value)
                with self.condition:self.latest=state;self.condition.notify_all()
        except Exception as error:
            if not self.stop.is_set():self.failure=error

    def spin(self):
        try:
            while not self.stop.is_set():self.executor.spin_once(timeout_sec=.01)
        except Exception as error:self.failure=error

    def heartbeat(self):
        try:
            while not self.stop.is_set():
                with self.lock:values=list(self.target)
                message=self.Twist();message.linear.x=values[0];message.angular.z=values[5]
                self.node.publisher.publish(message);self.node.record('source_command',values=values)
                self.stop.wait(.02)
        except Exception as error:self.failure=error

    def healthy(self):
        if getattr(self.node,'errors',[]):raise RuntimeError('Capture worker failed: '+repr(self.node.errors))
        if self.failure:raise RuntimeError('Scheduler worker failed: '+repr(self.failure))
        if time.monotonic()>self.deadline:raise TimeoutError('600 second case deadline')
        if any(p.poll() is not None for p,_ in self.processes):raise RuntimeError('Runtime process exited')
        if self.node.truth and max(abs(self.node.truth[-1]['roll']),abs(self.node.truth[-1]['pitch']))>.8:
            raise RuntimeError('Physical instability: roll/pitch exceeded 0.8 rad')

    def wait_state(self,iteration=None,after=0.,timeout=5.):
        until=min(self.deadline,time.monotonic()+timeout)
        while time.monotonic()<until:
            self.healthy()
            with self.condition:
                value=self.latest
                if value and value['wall']>=after:
                    if iteration is not None and value['iteration']>iteration:raise RuntimeError('Physical step overshoot')
                    if value['paused'] and (iteration is None or value['iteration']==iteration):return dict(value)
                self.condition.wait(.02)
        raise TimeoutError('World statistics did not confirm actual paused iteration')

    def acknowledge(self,linear=0.,angular=0.):
        before=self.wait_state();sent=time.monotonic();target=[linear,0.,0.,0.,0.,angular]
        with self.lock:self.target=target
        until=min(self.deadline,sent+5.)
        while time.monotonic()<until:
            self.healthy()
            with self.node.lock:rows=list(self.node.rows)
            if all(confirmed(rows,kind,target,sent) for kind in ('safe_command','gz_command')):
                current=self.wait_state(after=sent)
                if current['iteration']!=before['iteration'] or abs(current['sim']-before['sim'])>1e-8:
                    raise RuntimeError('Physics advanced during command confirmation')
                self.node.record('command_barrier',values=target,iteration=current['iteration'],physical_time=current['sim'],
                    sent_wall=sent,confirmed_wall=time.monotonic(),latency_s=time.monotonic()-sent)
                return
            time.sleep(.01)
        raise TimeoutError('Two consecutive ROS-safe and Gazebo input confirmations missing within 5 wall seconds')

    def advance(self,label,seconds):
        before=self.wait_state();count=step_count(seconds,self.step);target=before['iteration']+count
        sent=time.monotonic()
        answer=self.command('gz','service','-s','/world/r5_check/control','--reqtype','gz.msgs.WorldControl',
            '--reptype','gz.msgs.Boolean','--timeout','5000','--req',f'pause: true, multi_step: {count}')
        if 'data: true' not in answer:raise RuntimeError('WorldControl request rejected: '+answer)
        current=self.wait_state(target,sent,timeout=max(.1,self.deadline-time.monotonic()))
        if abs(current['sim']-before['sim']-seconds)>1e-8:raise RuntimeError('Physical time does not match requested integer steps')
        marker=Path(str(self.observer)+'.paused');until=min(self.deadline,time.monotonic()+5.)
        while not pause_marker(marker,target,current['sim']) and time.monotonic()<until:
            self.healthy();time.sleep(.01)
        if not pause_marker(marker,target,current['sim']):raise RuntimeError('Observer flush marker is stale or missing')
        destination=self.node.folder/f'pause-{len(self.phases):02d}-{label}.txt'
        with destination.open('x') as stream:stream.write(marker.read_text())
        row=last_step(self.observer)
        if int(row['iteration'])!=target or abs(row['t']-current['sim'])>1e-8:raise RuntimeError('Observer tail differs from actual paused boundary')
        phase=dict(label=label,steps=count,before=before,after=current,request_wall=sent,
            completed_wall=time.monotonic(),flush_file=destination.name,last_step=row)
        self.phases.append(phase);self.node.record('paused_stage',phase=phase)
        return phase

    def run(self,linear,angular):
        self.wait_state(timeout=10.);self.acknowledge()
        startup=time.monotonic()
        for _ in range(20):
            self.advance('warmup',.1);time.sleep(.1)
            if self.ready():break
            if time.monotonic()-startup>90:raise TimeoutError('Sensor readiness exceeded 90 wall seconds')
        else:raise RuntimeError('Sensors not ready within two simulation seconds')
        start=self.wait_state()['sim'];self.acknowledge()
        park=self.advance('parked',5.);self.acknowledge(linear,angular)
        motion=self.advance('motion',20.);self.acknowledge()
        stop=self.advance('stop',3.)
        until=min(self.deadline,time.monotonic()+5.)
        while time.monotonic()<until:
            self.healthy()
            with self.node.lock:rows=list(self.node.rows)
            if all(max((r.get('stamp',-1) for r in rows if r['kind']==kind),default=-1)>=stop['after']['sim']-.1
                   for kind in ('truth','joints')):break
            time.sleep(.02)
        else:raise RuntimeError('ROS truth/joint tail did not drain after final pause')
        time.sleep(.2)
        return dict(mode='paused_steps',step=self.step,sim_started=start,sim_ended=stop['after']['sim'],
            parked=park,motion=motion,stop=stop,phases=self.phases)

    def close(self):
        with self.lock:self.target=[0.]*6
        self.node.publisher.publish(self.Twist());self.stop.set()
        try:os.killpg(self.stats_proc.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:self.stats_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:self.stats_proc.kill();self.stats_proc.wait()
        for thread in self.threads:thread.join(timeout=3)
        self.err.close()
