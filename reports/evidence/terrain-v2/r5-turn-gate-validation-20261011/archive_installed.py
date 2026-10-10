import hashlib,json,shutil
from pathlib import Path
p=Path('/evidence');freeze=json.loads((p/'campaign/freeze.json').read_text());dest=p/'installed-snapshot';dest.mkdir()
for name,expected in freeze['installed'].items():
    source=Path(name)
    if hashlib.sha256(source.read_bytes()).hexdigest()!=expected:raise RuntimeError('Installed source changed '+name)
    target=dest/str(source).lstrip('/');target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
(p/'installed-snapshot.json').write_text(json.dumps(dict(files=len(freeze['installed']),verified=True),indent=2)+'\n')
print(len(freeze['installed']))
