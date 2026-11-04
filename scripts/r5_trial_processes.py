"""Own a trial by inherited token AND Linux PID start time, not process names."""
import os
import signal
import subprocess
import time
from pathlib import Path


def owned(token, known=()):
    result=[]
    marker=('MOONCAR_TRIAL_ID='+token).encode()
    identities={(r['pid'],r['start_ticks']):r for r in known}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:
            stat=(path/'stat').read_text().rsplit(')',1)[1].split()
            key=(int(path.name),int(stat[19]))
            try:environment=(path/'environ').read_bytes().split(b'\0')
            except OSError:
                if key not in identities:continue
                environment=[]  # Zombie environments can disappear before stat.
            if key not in identities and marker not in environment:continue
            variables=dict(x.split(b'=',1) for x in environment if b'=' in x)
            command=(path/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
            result.append(dict(pid=key[0],start_ticks=key[1],state=stat[0],
                               gz_partition=variables.get(b'GZ_PARTITION',identities.get(key,{}).get('gz_partition','').encode()).decode(),
                               command=command or identities.get(key,{}).get('command','')))
        except (OSError,ValueError):continue
    return result


def server_context(processes):
    servers=[row for row in processes if row['state']!='Z' and row['command'].startswith('gz sim ')]
    if not servers:return None
    partitions={row.get('gz_partition','') for row in servers}
    if len(partitions)!=1 or not next(iter(partitions)):raise ValueError('Owned simulation transport partition missing or ambiguous')
    return dict(GZ_PARTITION=next(iter(partitions)),ROS_DOMAIN_ID='95',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')


def finish_transport_stream(process,raw,error_stream):
    """A parent may already have stopped the reader group on an abort."""
    try:
        if process.poll() is None:
            try:os.killpg(process.pid,signal.SIGINT)
            except ProcessLookupError:pass
        try:process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            process.wait()
    finally:
        raw.close();error_stream.close()


def cleanup(token, world, abnormal, known=(), reap=None):
    result=dict(abnormal=abnormal,owned_before=owned(token,known))
    control_passed=True;context=None
    if abnormal:
        try:context=server_context(result['owned_before'])
        except ValueError as error:control_passed=False;result['transport_context_error']=str(error)
    if abnormal and context:
        result['transport_context']=context
        requests=[['gz','topic','-t','/cmd_vel','-m','gz.msgs.Twist','-p','linear {x: 0} angular {z: 0}'],
                  ['gz','service','-s',f'/world/{world}/control','--reqtype','gz.msgs.WorldControl',
                   '--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true']]
        result['stop_pause_requests']=[]
        for request in requests:
            try:
                p=subprocess.run(request,capture_output=True,text=True,timeout=4,env=dict(os.environ,**context))
                result['stop_pause_requests'].append(dict(exit_code=p.returncode,output=p.stdout+p.stderr))
                if p.returncode or 'timed out' in (p.stdout+p.stderr).lower():control_passed=False
                if request[1]=='service' and 'data: true' not in p.stdout:control_passed=False
            except subprocess.TimeoutExpired:
                control_passed=False;result['stop_pause_requests'].append(dict(timeout=True))
        time.sleep(.2)  # Give paused observers their flush boundary before SIGINT.
    for signum,seconds in [(signal.SIGINT,5),(signal.SIGTERM,3),(signal.SIGKILL,2)]:
        if reap is not None:reap()
        processes=owned(token,known)
        if not processes:break
        for row in processes:
            # Re-check identity at the last possible moment to avoid PID reuse.
            if row['state']=='Z':continue  # Only its parent or container init can reap it.
            if any(x['pid']==row['pid'] and x['start_ticks']==row['start_ticks'] for x in owned(token,known)):
                try:os.kill(row['pid'],signum)
                except ProcessLookupError:pass
        deadline=time.monotonic()+seconds
        while owned(token,known) and time.monotonic()<deadline:
            if reap is not None:reap()
            time.sleep(.05)
    result['owned_after']=owned(token,known)
    result['stop_pause_passed']=control_passed
    result['passed']=not result['owned_after'] and control_passed
    return result
