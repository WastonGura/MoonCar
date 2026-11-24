"""Separate stopped execution signature from future tool repair."""
import hashlib,json
from pathlib import Path
p=Path(__file__).resolve().parent;root=p.parents[3]
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
freeze=json.loads((p/'campaign/freeze.json').read_text());changed={}
for name,h in freeze['source'].items():
    current=sha(root/name)
    if current!=h:changed[name]=dict(frozen=h,future=current,snapshot=sha(p/'frozen-source-snapshot'/name))
added={str(f.relative_to(root)):sha(f) for f in (root/'containers/r5_contact_geometry').iterdir() if f.is_file()}
receipt=dict(stopped_attempts=3,qualified=2,failed=1,not_run=5,execution_unchanged_at_stop=True,
    repair_after_stop=True,new_simulations_after_repair=0,changed_execution_sources=changed,added_native_gate_sources=added,
    preparation_shell=dict(future=sha(root/'scripts/prepare_r5_contact_geometry.sh')),
    native_gate_binary=sha(p/'future-fixtures/mooncar-contact-mesh-preflight'),
    generator_fix_verified_by_native_loader=True,physical_turning_cause_confirmed=False,production_changed=False)
(p/'poststop-mesh-repair.json').write_text(json.dumps(receipt,indent=2)+'\n')
