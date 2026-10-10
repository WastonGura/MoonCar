"""External sampling only, outside the frozen controller and physical process."""
import json,subprocess,time
from pathlib import Path
p=Path(__file__).resolve().parent
docker=r'C:\Users\jiker\AppData\Local\Programs\DockerDesktop\resources\bin\docker.exe'
with (p/'performance.jsonl').open('a',buffering=1) as f:
    while True:
        s=json.loads((p/'campaign/summary.json').read_text())
        if s['status']!='running':break
        try:
            raw=subprocess.check_output([docker,'stats','--no-stream','--format','{{json .}}',
                'mooncar-r5-contact-geometry-20261011'],timeout=10,text=True)
            r=dict(wall=time.time(),docker_stats=json.loads(raw),campaign=s['attempts'])
        except Exception as error:r=dict(wall=time.time(),error=repr(error),campaign=s['attempts'])
        f.write(json.dumps(r)+'\n');time.sleep(15)

