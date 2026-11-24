"""Verify the v1.1 evaluation amendment and reviewed MoonCar source."""
import hashlib
import json
from pathlib import Path
import zipfile

root = Path(__file__).resolve().parents[4]
here = Path(__file__).parent
freeze = json.loads((here / 'freeze.json').read_text(encoding='utf-8'))
for name, expected in freeze['source_sha256'].items():
    path = root / name
    assert hashlib.sha256(path.read_bytes()).hexdigest() == expected, f'changed: {name}'
archive = here / 'source.zip'
assert hashlib.sha256(archive.read_bytes()).hexdigest() == freeze['source_archive_sha256']
with zipfile.ZipFile(archive) as source:
    assert set(source.namelist()) == set(freeze['source_sha256'])
    for name, expected in freeze['source_sha256'].items():
        assert hashlib.sha256(source.read(name)).hexdigest() == expected, f'archive changed: {name}'
assert freeze['evaluation']['version'] == 'mapping-acceptance-v1.1'
assert freeze['evaluation']['score'] == '100 * coverage * precision'
assert freeze['evaluation']['height_error_limit_m'] == .05
assert freeze['evaluation']['slope_error_limit_deg'] == 5.
print(f"PASS: {len(freeze['source_sha256'])} files and source.zip match evaluation v1.1")
