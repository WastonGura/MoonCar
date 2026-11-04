"""Read mounted R5 ledgers on the host. Never enter or control a container.

Examples: python scripts/r5_host_progress.py --runtime D:/.../runtime
          python scripts/r5_host_progress.py --runtime D:/.../runtime --watch
Progress is advisory. Only the existing qualification/analysis gates accept data.
"""
import argparse
import json
import math
import os
from pathlib import Path
import time
from r5_monitoring import monitoring_profile

def read_shared(path):
    """Allow a Windows-mounted producer to write/replace while the host reads."""
    path=Path(path)
    if os.name!='nt':return path.read_bytes()
    import ctypes
    from ctypes import wintypes
    import msvcrt
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.CreateFileW.argtypes=[wintypes.LPCWSTR,wintypes.DWORD,wintypes.DWORD,
        wintypes.LPVOID,wintypes.DWORD,wintypes.DWORD,wintypes.HANDLE]
    kernel.CreateFileW.restype=wintypes.HANDLE
    kernel.CloseHandle.argtypes=[wintypes.HANDLE];kernel.CloseHandle.restype=wintypes.BOOL
    handle=kernel.CreateFileW(str(path),0x80000000,1|2|4,None,3,0x80,None)
    if handle==wintypes.HANDLE(-1).value:raise ctypes.WinError(ctypes.get_last_error())
    try:fd=msvcrt.open_osfhandle(handle,os.O_RDONLY|os.O_BINARY)
    except BaseException:kernel.CloseHandle(handle);raise
    with os.fdopen(fd,'rb') as stream:return stream.read()

def snapshot(runtime,now=None):
    """Partial/unreadable progress stays pending and never becomes acceptance."""
    root=Path(runtime);now=time.time() if now is None else now
    result=dict(purpose='progress_only',monitoring_profile=monitoring_profile(),sampled_wall=now)
    stage='preparation';path=None
    try:
        for candidate,filename in [('campaign','campaign.json'),('diagnostic','ledger.json')]:
            probe=root/candidate/filename
            if probe.exists():stage=candidate;path=probe;break
        if path is None:return dict(result,stage=stage,status='pending',reason='No ledger published yet')
        value=json.loads(read_shared(path).decode('utf-8-sig'))
        rows=value['attempts'];status=value['status'];maximum=value['maximum_attempts']
        if not isinstance(rows,list) or any(not isinstance(r,dict) for r in rows):raise ValueError('Invalid progress rows')
        if type(maximum) is not int or not 0<=len(rows)<=maximum:raise ValueError('Invalid attempt count')
        if status not in ('running','completed','stopped'):raise ValueError('Unknown batch status')
        row=rows[-1] if rows else {};started=row.get('started_wall',now)
        if not isinstance(started,(float,int)) or not math.isfinite(started):raise ValueError('Invalid start time')
        result.update(stage=stage,status=status,ledger=str(path),started=len(rows),maximum_attempts=maximum,
            current=row.get('name'),reason=value.get('reason'),elapsed_s=max(0,round(now-started,1)),
            declared_monitoring_profile=value.get('monitoring_profile'),
            qualification_source='stored_ledger_only')
        if stage=='campaign':
            result['qualified']=sum(r.get('qualification',{}).get('diagnostic_qualified') is True for r in rows)
        else:
            result['completed']=sum(r.get('cleanup_passed') is True and not r.get('failure') for r in rows)
        return result
    except (OSError,ValueError,KeyError,TypeError,AttributeError) as error:
        return dict(result,stage=stage,status='pending',ledger=str(path),reason='Progress sample unavailable: '+repr(error))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime',required=True,type=Path);p.add_argument('--watch',action='store_true')
    p.add_argument('--interval',type=float,default=5.);p.add_argument('--samples',type=int,default=0)
    a=p.parse_args()
    if not math.isfinite(a.interval) or not .05<=a.interval<=60 or a.samples<0:p.error('Invalid polling interval/count')
    count=0
    try:
        while True:
            value=snapshot(a.runtime);print(json.dumps(value,allow_nan=False),flush=True);count+=1
            if not a.watch or value['status'] in ('completed','stopped') or a.samples and count>=a.samples:return 0
            time.sleep(a.interval)
    except KeyboardInterrupt:return 0

if __name__=='__main__':raise SystemExit(main())
