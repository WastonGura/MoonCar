import sys,time
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from path_resampling_evaluation import report
from report_heading_control import report as paired_report
root=Path('/evidence/heading-control-20260913');output=Path('/evidence/heading-control-analysis-20260913')
deadline=time.monotonic()+1800
while not (root/'finished.json').exists():
    if time.monotonic()>deadline:raise TimeoutError('Capture batch not finished; retain partial evidence')
    time.sleep(5)
report(root,output)
paired_report(root,output)
print('HEADING CONTROL ANALYSIS FINISHED',flush=True)
