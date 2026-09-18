import json,hashlib,subprocess,sys
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from compare_survey_strategies import request
root=Path('/evidence/common-action-20260913');output=Path('/evidence/common-action-analysis-20260913')
assert (root/'finished.json').exists()
state=request();assert state['phase']=='stopped' and not state['motion']['active']
for path,digest in json.loads((root/'plan.json').read_text())['source_sha256'].items():
    assert hashlib.sha256((Path('/source')/path).read_bytes()).hexdigest()==digest,path
subprocess.run([sys.executable,'/source/tmp/evaluate_common_action.py',str(root),str(output)],check=True)
subprocess.run([sys.executable,'/source/tmp/plot_action_value.py',str(root),str(output)],check=True)
subprocess.run([sys.executable,'/source/tmp/plot_common_support.py',str(output)],check=True)
print('PASS evaluation sources match the pre-capture frozen protocol')
