import sys,time
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from path_resampling_evaluation import report
root=Path('/evidence/nav2-observation-fixed-20260913');logfile=Path('/evidence/nav2-observation-fixed-run-20260913.log')
deadline=time.monotonic()+2400
while time.monotonic()<deadline:
    log=logfile.read_text()
    if 'ALL NAV2 OBSERVATION CAPTURES FINISHED' in log:break
    if 'Traceback (most recent call last)' in log:raise RuntimeError('Capture runner failed; inspect retained log')
    time.sleep(3)
else:raise TimeoutError('Waiting for four captures')
report(root,Path('/evidence/nav2-observation-analysis-20260913'))
print('NAV2 OBSERVATION ANALYSIS FINISHED',flush=True)
