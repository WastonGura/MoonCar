"""Read-only witness shutdown with one absolute deadline; no physics calls."""
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import traceback

def shutdown_witness(child,reader,writer,errors,folder,seconds=5.):
    started=time.monotonic();deadline=started+seconds;stages={}
    if writer:writer.begin_finish(deadline)
    stages['drain_started']=time.monotonic()-started
    if child.poll() is None:child.send_signal(signal.SIGINT)
    try:child.wait(max(0,deadline-time.monotonic()))
    except subprocess.TimeoutExpired:
        errors.append('native reader process deadline');child.kill()
    stages['native_exit']=time.monotonic()-started
    reader.join(max(0,deadline-time.monotonic()))
    stages['reader_exit']=time.monotonic()-started
    if reader.is_alive():errors.append('native reader thread deadline')
    events=writer.finish(deadline) if writer else None
    stages['event_files_closed_and_verified']=time.monotonic()-started
    passed=not errors and not reader.is_alive() and child.poll() is not None and (events is None or events['drained'])
    result=dict(errors=list(errors),thread_drained=not reader.is_alive(),events=events,
        passed=passed,deadline_seconds=seconds,stage_seconds=stages)
    if not passed:
        frames=sys._current_frames()
        stacks={t.name:''.join(traceback.format_stack(frames[t.ident])) for t in threading.enumerate() if t.ident in frames}
        (Path(folder)/'probe-shutdown-stacks.json').write_text(json.dumps(stacks,indent=2)+'\n')
    (Path(folder)/'probe-status.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
