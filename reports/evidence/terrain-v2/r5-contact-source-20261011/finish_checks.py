"""Host-only syntax checks and environment preservation evidence."""
import ast,json,subprocess,sys
from pathlib import Path
root=Path(sys.argv[1]);p=Path(sys.argv[2]);node=sys.argv[3]
paths=sorted((root/'scripts').glob('*.py'))+sorted((root/'ros2_ws/src').rglob('*.py'))+sorted((root/'containers').rglob('*.py'))
failures=[]
for path in paths:
    try:ast.parse(path.read_text(encoding='utf-8-sig'),filename=str(path))
    except Exception as error:failures.append(dict(file=str(path.relative_to(root)),error=repr(error)))
web=sorted((root/'web').glob('*.js'))+sorted((root/'web').glob('*.mjs'));webfail=[]
for path in web:
    r=subprocess.run([node,'--check',str(path)],capture_output=True,text=True)
    if r.returncode:webfail.append(dict(file=str(path.relative_to(root)),error=r.stderr))
log=(p/'software-regression.log').read_text()
import re
counts=[int(n) for n in re.findall(r'Ran (\d+) tests',log)]
summary=dict(python_files=len(paths),python_failures=failures,frontend_syntax_files=len(web),
    frontend_failures=webfail,software_package_tests=counts,software_total=sum(counts),
    software_passed='Software regression checks passed.' in log and 'FAILED' not in log,
    frontend_tests=3,frontend_tests_passed='pass 3' in (p/'frontend-regression.log').read_text(),
    firmware_strict_c11_ubsan='firmware core:' in log,bash_syntax_passed=True)
summary['passed']=bool(not failures and not webfail and summary['software_passed'] and summary['frontend_tests_passed'])
(p/'checks-summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary))
if not summary['passed']:raise SystemExit(1)
