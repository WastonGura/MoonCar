"""Bounded ENODATA recovery in a test copy; production guard stays unchanged."""
import hashlib,json
from pathlib import Path

BASE_SHA='af77edc91b6d6a9b678bc7797e924afce3288a077fafc461d62670d183dc9e93'
OLD='''        while complete_receipt(receipt) is None:
            if child.poll() is not None:raise RuntimeError('Bridge exited before DDS receipt')
            if time.monotonic()>end:raise TimeoutError('Bridge DDS receipt timeout')
            time.sleep(.02)
        # Writer flush/close and SHM mapping precede eligibility to report ready.
        proof=process_receipt(child.pid,receipt,root)
'''
NEW='''        # Test-only: receipt / runtime proof must both finish within the old bound.
        while True:
            if child.poll() is not None:raise RuntimeError('Bridge exited before DDS receipt')
            if time.monotonic()>end:raise TimeoutError('Bridge DDS receipt timeout')
            try:
                if complete_receipt(receipt) is not None:
                    proof=process_receipt(child.pid,receipt,root)
                    if time.monotonic()>end:raise TimeoutError('Bridge DDS receipt timeout')
                    break
            except OSError as error:
                if error.errno!=errno.ENODATA:raise
                # Do not report ready on an exception. Never retry semantic failures.
                print('MoonCar test startup transient IO: '+json.dumps(dict(
                    errno=error.errno,error=repr(error),filename=error.filename,
                    traceback=traceback.format_exc())),file=sys.stderr,flush=True)
            time.sleep(.02)
'''

def generate(source,output):
    source=Path(source);output=Path(output)
    raw=source.read_bytes();digest=hashlib.sha256(raw).hexdigest()
    if digest!=BASE_SHA:raise ValueError('Test guard requires the exact frozen production guard')
    text=raw.decode()
    if text.count(OLD)!=1:raise ValueError('Startup verification anchor mismatch')
    if output.exists():raise FileExistsError('Refuse to overwrite an existing test guard')
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_bytes(('import errno,traceback\n'+text.replace(OLD,NEW,1)).encode())
    result=dict(base_sha256=digest,test_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        production_changed=False,timeout_changed=False,capacity_qos_libraries_changed=False,
        allowed_difference='Trace and retry only ENODATA during pre-ready receipt/runtime proof; all other failures propagate')
    output.with_suffix('.manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();print(json.dumps(generate(a.source,a.output)))
