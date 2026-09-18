"""Repeat the observed atomic-replacement race on the existing evidence bind mount."""
import sys,threading,warnings,hashlib
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from compare_survey_strategies import read_pending_json
from survey_evaluation import write_json
root=Path(sys.argv[1]);root.mkdir(exist_ok=False)
path=root/'atomic-manifest.json';errors=[];read_errors=[];seen=[];missing=0
write_json(path,{'sequence':0})
def writer():
    for i in range(1,1001):
        try:write_json(path,{'sequence':i})
        except OSError as e:errors.append(dict(errno=e.errno,message=str(e)))
thread=threading.Thread(target=writer);thread.start()
with warnings.catch_warnings(record=True) as events:
    warnings.simplefilter('always')
    while thread.is_alive():
        try:
            value=read_pending_json(path)
            if value is None:missing+=1
            else:seen.append(value['sequence'])
        except OSError as e:read_errors.append(dict(errno=e.errno,message=str(e)))
thread.join()
write_json(root/'summary.json',dict(writes=1000,read_count=len(seen),missing_or_transient=missing,
    warnings=[str(e.message) for e in events],writer_errors=errors,reader_errors=read_errors,final=read_pending_json(path),
    source_sha256=hashlib.sha256(Path('/source/scripts/compare_survey_strategies.py').read_bytes()).hexdigest()))
print((root/'summary.json').read_text())
