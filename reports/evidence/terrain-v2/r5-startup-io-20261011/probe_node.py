"""Real RMW proof/close; no Gazebo, control topics or simulated images."""
import json,signal,subprocess,time,sys
from pathlib import Path
p=Path('/evidence');results=[]
for i in range(2):
    folder=p/f'native-{i+1:02d}';log_path=p/f'native-{i+1:02d}.log'
    with log_path.open('x') as log:
        child=subprocess.Popen([sys.executable,'/evidence/test-guard.py','--root','/opt/mooncar/image-dds/rmw-8.4.4',
            '--receipt-directory',str(folder),'--binary','/work/native-node/rmw-node'],stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+15;ready=False
        while time.monotonic()<deadline and child.poll() is None:
            if 'NODE_READY' in log_path.read_text() and (folder/'ready.json').is_file():ready=True;break
            time.sleep(.02)
        if child.poll() is None:child.send_signal(signal.SIGINT)
        try:code=child.wait(5)
        except subprocess.TimeoutExpired:child.kill();code=child.wait()
    proof=json.loads((folder/'ready.json').read_text()) if (folder/'ready.json').is_file() else None
    result=dict(name=folder.name,node_constructed=ready,code=code,
        node_closed='NODE_CLOSED' in log_path.read_text(),proof=proof,
        passed=bool(ready and code==0 and 'NODE_CLOSED' in log_path.read_text() and proof))
    results.append(result)
    if not result['passed']:break
summary=dict(no_gazebo=True,simulations=0,results=results,passed=len(results)==2 and all(x['passed'] for x in results))
(p/'native-summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary))
raise SystemExit(0 if summary['passed'] else 1)
