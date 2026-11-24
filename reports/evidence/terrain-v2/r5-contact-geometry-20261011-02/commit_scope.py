"""Commit only this investigation, excluding raw data and pre-existing edits."""
import subprocess
from pathlib import Path
p=Path(__file__).resolve().parent;root=p.parents[3];base=str(p.relative_to(root)).replace('\\','/')
paths=['scripts/prepare_r5_contact_geometry.py','scripts/prepare_r5_contact_geometry.sh','scripts/run_r5_contact_geometry.py',
    'scripts/r5_contact_geometry.py','ros2_ws/src/mooncar_gazebo/tests/test_contact_geometry.py',
    'containers/r5_contact_geometry/CMakeLists.txt','containers/r5_contact_geometry/MeshPreflight.cc',
    'docs/work_logs/2026-10-11-r5-contact-geometry-recheck.md']
paths += [base+'/'+name for name in ('README.md','NEXT_RUN.md','authorization.json','prediction.json','poststop-mesh-repair.json',
    'closure.json','checks-summary.json','archive-manifest.json','archive-verification.json',
    'setup_run.py','execute_run.py','receipt_preflight.py','check_future_mesh.py','analyze_run.py','closeout.py','save_repair.py','commit_scope.py',
    'campaign/freeze.json','campaign/summary.json','campaign/ledger.json','analysis/results.json','analysis/contact-comparison.png',
    'future-fixtures/result.json','future-fixtures/coplanar_mesh-old.json','future-fixtures/coplanar_mesh-fixed.json',
    'future-fixtures/mirrored_mesh-old.json','future-fixtures/mirrored_mesh-fixed.json','receipt-preflight/result.json')]
assert not subprocess.check_output(['git','diff','--cached','--name-only'],cwd=root).strip(),'Preserve existing index'
subprocess.run(['git','add','--']+paths,cwd=root,check=True)
staged=set(subprocess.check_output(['git','diff','--cached','--name-only'],cwd=root,text=True).splitlines())
assert staged==set(paths),(staged-set(paths),set(paths)-staged)
subprocess.run(['git','diff','--cached','--check'],cwd=root,check=True)
subprocess.run(['git','commit','-m','test: reject R5 diagnostic meshes without loaded normals'],cwd=root,check=True)
print(subprocess.check_output(['git','rev-parse','--short','HEAD'],cwd=root,text=True).strip())
