"""Post-audit preservation checks; does not execute ROS/Gazebo."""
import ast, hashlib, json, subprocess, sys, re
from pathlib import Path
root=Path(sys.argv[1]); p=Path(sys.argv[2]); docker=sys.argv[3]; node=sys.argv[4]
def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()
old=root/'reports/evidence/terrain-v2/r5-turn-gate-validation-20261011'
proof=json.loads((p/'analysis.json').read_text())
inputs={str(Path(k).relative_to(root)):digest(Path(k))==v for k,v in proof['input_hashes'].items()}
baseline=json.loads((old/'preservation-before.json').read_text())
preserved={k:(root/k).is_file() and digest(root/k)==v for k,v in baseline.items()}
recorded=json.loads((old/'archive-manifest.json').read_text())['files']
archive_failures=[]
for key,item in recorded.items():
    file=old/Path(key.replace('\\','/'))
    if not file.is_file() or file.stat().st_size!=item['bytes'] or digest(file)!=item['sha256']:
        archive_failures.append(key)
containers={};actual={}
for name in ('mooncar-terrain-validation','mooncar-r5-turn-gate-validation-20261011'):
    item=json.loads(subprocess.check_output([docker,'inspect',name],text=True))[0]
    actual[name]=item
    containers[name]=dict(id=item['Id'],running=item['State']['Running'],status=item['State']['Status'],oom=item['State']['OOMKilled'])
expected=json.loads((old/'workbench-before.json').read_text())
if isinstance(expected,list):expected=expected[0]
same_workbench=containers['mooncar-terrain-validation']['id']==expected['Id']
workbench_state_same=actual['mooncar-terrain-validation']['State']==expected['State']
workbench_config_same=actual['mooncar-terrain-validation']['Config']==expected['Config']
offline_left=[line for line in subprocess.check_output([docker,'ps','-a','--format','{{.Names}}'],text=True).splitlines()
              if line in ('mooncar-r5-ramp-offline-audit-20261011','mooncar-r5-ramp-offline-checks-20261011')]
files=sorted((root/'scripts').glob('*.py'))+sorted((root/'ros2_ws/src').rglob('*.py'))+sorted((root/'containers').rglob('*.py'))
syntax=[]
for file in files:
    try:ast.parse(file.read_text(encoding='utf-8-sig'))
    except Exception as error:syntax.append(dict(file=str(file.relative_to(root)),error=repr(error)))
js=sorted((root/'web').glob('*.js'))+sorted((root/'web').glob('*.mjs'));jsfail=[]
for file in js:
    r=subprocess.run([node,'--check',str(file)],capture_output=True,text=True)
    if r.returncode:jsfail.append(dict(file=str(file.relative_to(root)),error=r.stderr))
log=(p/'software-regression-retry.log').read_text()
counts=[int(n) for n in re.findall(r'Ran (\d+) tests',log)]
front=(p/'frontend-regression-complete.log').read_text(encoding='utf-8-sig')
checks=dict(software_counts=counts,software_total=sum(counts),software_passed='Software regression checks passed.' in log and 'FAILED' not in log,
            python_files=len(files),python_failures=syntax,frontend_syntax_files=len(js),frontend_syntax_failures=jsfail,
            frontend_tests_passed='pass 3' in front,firmware_strict_c11_ubsan='firmware core:' in log,
            sdf_checks={n:'Valid.' in (p/'fixtures'/f'{n}-check.log').read_text() for n in ('plate','coplanar_mesh','mirrored_mesh')})
checks['passed']=checks['software_passed'] and not syntax and not jsfail and checks['frontend_tests_passed'] and checks['firmware_strict_c11_ubsan'] and all(checks['sdf_checks'].values())
result=dict(input_hashes_after=inputs,original_files_count=len(preserved),original_file_failures=[k for k,v in preserved.items() if not v],
            previous_archive_files=len(recorded),previous_archive_failures=archive_failures,containers=containers,
            workbench_same_identity=same_workbench,workbench_state_same=workbench_state_same,
            workbench_config_same=workbench_config_same,offline_containers_remaining=offline_left,new_simulations=0,checks=checks)
result['passed']=all(inputs.values()) and all(preserved.values()) and not archive_failures and same_workbench and workbench_state_same and workbench_config_same and not offline_left and not containers['mooncar-r5-turn-gate-validation-20261011']['running'] and checks['passed']
(p/'verification-complete.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(passed=result['passed'],checks=checks,preserved=len(preserved),archive_files=len(recorded),failures=result['original_file_failures']+archive_failures)))
if not result['passed']:raise SystemExit(1)
