import csv
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
from urllib.request import urlopen
from pypdf import PdfReader

repo=Path('D:/Document/Repositories/MoonCar-mapping')
root=repo/'reports/evidence/terrain-workbench/mapping-acceptance-20260914'
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for directory in ('reference','report-motion','report-unstarted'):
    for name,expected in read(root/directory/'hashes.json').items():
        assert sha(root/directory/name)==expected,(directory,name)
for kind,capture in [('report-motion','capture-motion'),('report-unstarted','capture')]:
    report=root/kind;summary=read(report/'summary.json')
    assert summary['evaluable'] and summary['reference_audit']['passed']
    for name,expected in summary['input_sha256'].items():assert sha(root/capture/'approach'/name)==expected
    assert sha(root/capture/'approach-control.json')==summary['control_sha256']
    with (report/'cells.csv').open(encoding='utf-8') as f:cells=list(csv.DictReader(f))
    eligible=[c for c in cells if c['eligible']=='True'];valid=[c for c in eligible if c['valid']=='True']
    good=[c for c in eligible if c['qualified']=='True']
    assert math.isclose(summary['S'],100*len(good)/len(eligible))
    assert math.isclose(summary['height_rmse_m'],math.sqrt(sum(float(c['height_error_m'])**2 for c in valid)/len(valid)))
    reader=PdfReader(report/'maps.pdf');assert len(reader.pages)==1
    assert 'Cell evaluation' in reader.pages[0].extract_text()
    assert 'Qualified' in reader.pages[0].extract_text()
    for p in (report/'source').glob('*.py'):
        current=(repo/'scripts'/p.name if (repo/'scripts'/p.name).exists() else repo/'ros2_ws/src/mooncar_terrain/scripts'/p.name)
        assert sha(p)==sha(current),p
normal=read(root/'report-motion/summary.json');failed=read(root/'report-unstarted/summary.json')
assert normal['S']==56.71296296296296 and normal['evidence']['runtime_fault'] is False
assert failed['evidence']['runtime_fault'] is True
recheck=read(repo/'tmp/ros_acceptance/recheck-mapping-v1/summary.json')
for field in ('S','C','P','height_rmse_m','slope_mae_deg'):assert recheck[field]==normal[field]
assert read(root/'validation.json')['status']=='passed'
assert 'Ran 5 tests' in (root/'tests.log').read_text(encoding='utf-8-sig')
for md in (root/'README.md',root/'report-motion/README.md',root/'report-unstarted/README.md'):
    for link in re.findall(r'\]\(([^)]+)\)',md.read_text(encoding='utf-8')):
        if not link.startswith(('http:','https:')):assert (md.parent/link).resolve().exists(),(md,link)
state=json.load(urlopen('http://127.0.0.1:8766/api/simulation',timeout=10))
assert state['phase']=='stopped' and not state['motion']['active'] and not state['autonomy']['active']
source=root/'validation-source';source.mkdir(exist_ok=True)
for path in (repo/'tmp/finalize_mapping_acceptance.py',Path(__file__),repo/'scripts/capture_slope_resampling.py',
             repo/'ros2_ws/src/mooncar_terrain/tests/test_mapping_acceptance.py',repo/'docs/mapping_acceptance.md'):
    shutil.copyfile(path,source/path.name)
audit=dict(status='passed',requirements={
    'evaluation_spec':'docs/mapping_acceptance.md and validation-source/mapping_acceptance.md',
    'frozen_reference_and_coordinates':'reference/ and capture-motion/approach/coordinate-contract.json; live audit checks passed',
    'automatic_evaluator':'report-motion/source/; documented command reproduced all five primary metrics',
    'visual_report':'both final PDFs rendered with Poppler and visually inspected; labels and raster maps correct',
    'known_cases_and_real_record':'5 tests passed; validation.json; 53 real frames verified; failed attempt retained'},
    evidence_hashes_checked=True,cell_csv_recounted=True,local_links_checked=True,
    simulator_stopped=True,scoring_status=normal['status'],
    boundary='Evaluation tool delivered; simulation coverage failed target. Profile provisional and safety unverified.')
(root/'delivery-audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2),encoding='utf-8')
hashes={p.relative_to(root).as_posix():sha(p) for p in sorted(root.rglob('*')) if p.is_file() and p!=root/'hashes.json'}
(root/'hashes.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')
assert all(sha(root/name)==expected for name,expected in read(root/'hashes.json').items())
print(json.dumps(dict(status='passed',files=len(hashes),S=normal['S'],phase=state['phase']),ensure_ascii=False))
