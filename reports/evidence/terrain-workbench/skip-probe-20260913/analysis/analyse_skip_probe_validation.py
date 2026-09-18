import sys,json,hashlib,runpy,subprocess,shutil
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from swept_resampling import report
captures=Path('/evidence/skip-probe-validation-20260913');output=Path('/evidence/skip-probe-analysis-20260913')
assert (captures/'finished.json').exists(),'Wait for all attempts to finish'
plan=json.loads((captures/'plan.json').read_text())
for name,digest in plan['source_sha256'].items():
    assert hashlib.sha256((Path('/source')/name).read_bytes()).hexdigest()==digest,name
report(captures,output,'v3')
runpy.run_path(str(captures/'source/assess_skip_probe_validation.py'))['assess'](captures,output)
for name in ('verify_skip_prefixes.py','audit_skip_probe_windows.py','audit_hold_baseline.py','plot_skip_probe_validation.py'):
    subprocess.run([sys.executable,'/source/tmp/'+name,str(captures),str(output)],check=True)
    shutil.copyfile('/source/tmp/'+name,output/name)
shutil.copyfile(__file__,output/Path(__file__).name)
print('ANALYSIS, PREFIX REPRODUCTION AND PLOTS FINISHED',flush=True)
