"""Run the frozen eight-attempt entry and externally sample resource use."""
import hashlib, json, shutil, subprocess, time
from pathlib import Path

p = Path(__file__).resolve().parent
root = Path(__file__).resolve().parents[4]
docker = shutil.which('docker') or r'C:\Users\jiker\AppData\Local\Programs\DockerDesktop\resources\bin\docker.exe'
name = 'mooncar-r5-contact-gemini-20261011'
command = 'source /opt/ros/jazzy/setup.bash; source /work/install/setup.bash; python3 /source/scripts/run_r5_contact_geometry.py --output /probe/campaign'

last = None
snapshot = False
with (p / 'campaign.log').open('x') as log, (p / 'performance.jsonl').open('x', buffering=1) as perf:
    child = subprocess.Popen([docker, 'exec', name, 'bash', '-c', command], stdout=log, stderr=subprocess.STDOUT)
    try:
        while child.poll() is None:
            if (p / 'campaign/freeze.json').is_file() and not snapshot:
                f = json.loads((p / 'campaign/freeze.json').read_text())
                out = p / 'frozen-source-snapshot'
                for rel, digest in f['source'].items():
                    dest = out / rel
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(root / rel, dest)
                    assert hashlib.sha256(dest.read_bytes()).hexdigest() == digest
                snapshot = True
                print('Frozen source snapshot verified and saved.', flush=True)
            if (p / 'campaign/ledger.json').is_file():
                rows = json.loads((p / 'campaign/ledger.json').read_text())
                summary = json.loads((p / 'campaign/summary.json').read_text())
                state = (len(rows), sum(x.get('qualified', False) for x in rows), summary['status'])
                if state != last:
                    print(json.dumps(dict(attempts=state[0], qualified=state[1], status=state[2], latest=rows[-1] if rows else None)), flush=True)
                    last = state
                try:
                    raw = subprocess.check_output([docker, 'stats', '--no-stream', '--format', '{{json .}}', name], text=True, timeout=10)
                    perf.write(json.dumps(dict(wall=time.time(), attempt=state[0], docker_stats=json.loads(raw))) + '\n')
                except Exception as error:
                    perf.write(json.dumps(dict(wall=time.time(), error=repr(error))) + '\n')
            time.sleep(15)
    finally:
        if child.poll() is None:
            child.terminate()
            child.wait()

print('Campaign exit:', child.returncode, flush=True)
if (p / 'campaign/summary.json').is_file():
    print((p / 'campaign/summary.json').read_text(), flush=True)
raise SystemExit(child.returncode)
