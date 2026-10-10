"""Close only this campaign's exact container; preserve original workbench."""
import json, subprocess
from pathlib import Path
p = Path(__file__).resolve().parent
docker = r'C:\Users\jiker\AppData\Local\Programs\DockerDesktop\resources\bin\docker.exe'
def inspect(name):
    return json.loads(subprocess.check_output([docker, 'inspect', name], text=True))
def save(name, value):
    (p / name).write_text(json.dumps(value, indent=2) + '\n')
target = 'mooncar-r5-turn-gate-validation-20261011'
expected = '26333f7347b7f126db00824a7f9752702de349177818b96b86c8722beb5d10f5'
before = inspect(target)
original = json.loads((p / 'workbench-before.json').read_text(encoding='utf-8-sig'))
assert before[0]['Id'] == expected and expected != original[0]['Id']
assert json.loads((p / 'campaign/summary.json').read_text())['status'] != 'running'
save('container-before-stop.json', before)
completed = subprocess.run([docker, 'stop', '--time', '5', expected], capture_output=True, text=True, timeout=30)
save('container-stop.json', dict(returncode=completed.returncode, stdout=completed.stdout, stderr=completed.stderr))
assert completed.returncode == 0
save('container-final.json', inspect(expected))
save('workbench-after.json', inspect(original[0]['Id']))
print('Only this campaign container stopped')
