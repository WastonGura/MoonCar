import sys,json,time,shutil,hashlib
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from capture_slope_resampling import run
from compare_survey_strategies import request,wait_for
root=Path('/evidence/visible-path-trials-20260913')
plan=json.loads((root/'plan.json').read_text())
for name,digest in plan['source_sha256'].items():assert hashlib.sha256((Path('/source')/name).read_bytes()).hexdigest()==digest,name
(root/'source').mkdir()
for name in plan['source_sha256']:shutil.copyfile(Path('/source')/name,root/'source'/Path(name).name)
try:
    for trial in plan['trials']:
        run(root/trial['scene'],goal=trial['goal'],seconds=trial['seconds'],start=trial['start'],scene=trial['scene'])
        wait_for(lambda s:s['phase']=='stopped',90)
finally:
    request('stop',{})
print('ALL VISIBLE-SURFACE CAPTURES FINISHED',flush=True)
