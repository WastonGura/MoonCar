import sys,time,shutil
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from path_resampling_evaluation import report as paths
from arrival_resampling import report as stages
from assess_arrival_gate import report as assess
root=Path('/evidence/arrival-validation-prechecked-20260913');bundle=Path('/evidence/arrival-validation-evaluation-20260913')
deadline=time.monotonic()+3600
while not (root/'finished.json').exists():
    if time.monotonic()>deadline:raise TimeoutError('Capture batch not complete; retain partial attempts')
    time.sleep(5)
bundle.mkdir(exist_ok=False);shutil.copytree(root,bundle/'captures')
paths(bundle/'captures',bundle/'analysis')
stages(bundle,bundle/'stages')
assess(bundle)
(bundle/'finished.txt').write_text('Analysis finished; inspect acceptance before online integration.\n')
print('ALL ARRIVAL ANALYSIS FINISHED',flush=True)
