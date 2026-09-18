"""Verify the reviewed MoonCar research source and evaluation freeze."""
import hashlib
import json
from pathlib import Path
import zipfile

root = Path(__file__).resolve().parents[4]
freeze = json.loads((Path(__file__).parent / 'freeze.json').read_text(encoding='utf-8'))
for name, expected in freeze['source_sha256'].items():
    path = root / name
    assert path.is_file(), f'missing: {name}'
    assert hashlib.sha256(path.read_bytes()).hexdigest() == expected, f'changed: {name}'
archive_path = Path(__file__).parent / 'source.zip'
assert hashlib.sha256(archive_path.read_bytes()).hexdigest() == freeze['source_archive_sha256']
with zipfile.ZipFile(archive_path) as archive:
    assert set(archive.namelist()) == set(freeze['source_sha256'])
    for name, expected in freeze['source_sha256'].items():
        assert hashlib.sha256(archive.read(name)).hexdigest() == expected, f'archive changed: {name}'
assert freeze['scope']['online_strategies'] == [
    'nearest_view_v1', 'coverage_view_v1', 'surface_view_v1'
]
assert freeze['evaluation']['score'] == '100 * coverage * precision'
assert freeze['evaluation']['height_error_limit_m'] == 0.05
assert freeze['evaluation']['slope_error_limit_deg'] == 5.0
smoke = json.loads((Path(__file__).parent / 'smoke-result.json').read_text(encoding='utf-8'))
for name, expected in smoke['evidence_sha256'].items():
    assert hashlib.sha256((root / name).read_bytes()).hexdigest() == expected, f'smoke changed: {name}'
assert smoke['successful_attempt']['evidence_verified']
assert smoke['headless_batch_check']['evidence_verified']
assert smoke['headless_batch_check']['capture_gui'] is False
print(f"PASS: {len(freeze['source_sha256'])} files and source.zip match; strategies and evaluation v1 unchanged")
