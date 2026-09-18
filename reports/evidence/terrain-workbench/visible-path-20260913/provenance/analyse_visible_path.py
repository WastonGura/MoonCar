import sys,time
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from path_resampling_evaluation import report
root=Path('/evidence/visible-path-trials-20260913')
deadline=time.monotonic()+1800
while time.monotonic()<deadline:
    log=(root/'run.log').read_text()
    if 'ALL VISIBLE-SURFACE CAPTURES FINISHED' in log:break
    if 'Traceback (most recent call last)' in log:raise RuntimeError('Capture runner failed; inspect retained run.log')
    time.sleep(3)
else:raise TimeoutError('Waiting for four captures')
report(root,Path('/evidence/visible-path-analysis-20260913'))
print('VISIBLE PATH ANALYSIS FINISHED',flush=True)
