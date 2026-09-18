import sys,json,hashlib,shutil
from pathlib import Path
sys.path.insert(0,'/source/scripts')
from assess_arrival_gate import report
from path_resampling_evaluation import plot_capture
from resampling_dataset import TerrainMap
root=Path('/evidence/arrival-validation-evaluation-20260913');old=(root/'stages/gate-assessment.json').read_bytes()
report(root)
assert old==(root/'stages/gate-assessment.json').read_bytes(),'Formatting must not change assessment'
plan=json.loads((root/'captures/plan.json').read_text())
for t in plan['trials']:
    folder=root/'captures'/t['folder'];out=root/'analysis'/(t['folder']+'-approach')
    query=json.loads((folder/'approach-nav2-plan.json').read_text())
    plot_capture(folder,TerrainMap.load(out/'estimated-map.npz'),dict(t,display_route=[p[:2] for p in query['path']]),out)
shutil.copyfile('/source/scripts/path_resampling_evaluation.py',root/'analysis/path_resampling_plot_version.py')
(root/'stages/plot-correction.json').write_text(json.dumps(dict(reason='Presentation-only correction: map axes include negative-X starts; bar labels have headroom. Quantitative gate JSON unchanged byte for byte.',assessment_sha256=hashlib.sha256(old).hexdigest(),assessment_source_sha256=hashlib.sha256((root/'stages/assess_arrival_gate.py').read_bytes()).hexdigest(),path_plot_source_sha256=hashlib.sha256((root/'analysis/path_resampling_plot_version.py').read_bytes()).hexdigest()),indent=2))
print('PLOT FORMATTING CHECK PASSED')
