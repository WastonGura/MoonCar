import json,hashlib,subprocess,sys
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from compare_survey_strategies import request
root=Path('/evidence/action-value-20260913');output=Path('/evidence/action-value-analysis-20260913')
assert (root/'finished.json').exists(),'Capture batch is unfinished'
state=request();assert state['phase']=='stopped' and not state['motion']['active'],'Do not analyse while this experiment is moving'
for path,digest in json.loads((root/'plan.json').read_text())['source_sha256'].items():
    assert hashlib.sha256((Path('/source')/path).read_bytes()).hexdigest()==digest,'Frozen source changed: '+path
subprocess.run([sys.executable,'/source/tmp/evaluate_action_value.py',str(root),str(output)],check=True)
subprocess.run([sys.executable,'/source/tmp/plot_action_value.py',str(root),str(output)],check=True)
print('PASS current evaluation sources match the pre-capture frozen protocol')
