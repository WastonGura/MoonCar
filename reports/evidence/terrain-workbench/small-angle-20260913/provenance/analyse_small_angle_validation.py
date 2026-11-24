import sys,json,time,hashlib,shutil
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from swept_resampling import report
sys.path.insert(0,'/source/tmp')
from assess_small_angle_validation import assess
captures=Path('/evidence/small-angle-validation-20260913');output=Path('/evidence/small-angle-validation-analysis-20260913')
while not (captures/'plan.json').exists():time.sleep(1)
source=Path('/source/tmp/assess_small_angle_validation.py')
shutil.copyfile(source,captures/'assessment_source.py')
(captures/'assessment-protocol.json').write_text(json.dumps(dict(created_utc=__import__('datetime').datetime.now(__import__('datetime').timezone.utc).isoformat(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())))
while not (captures/'finished.json').exists():time.sleep(5)
report(captures,output,'v3')
assess(captures,output)
shutil.copyfile(source,output/'assess_small_angle_validation.py')
(output/'finished.txt').write_text('Capture and analysis complete\n')
