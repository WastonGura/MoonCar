import sys,json,hashlib,runpy,shutil,subprocess
from pathlib import Path
root=Path('/evidence/target-probes-20260913');output=Path('/evidence/target-probes-analysis-20260913')
assert (root/'finished.json').exists()
plan=json.loads((root/'plan.json').read_text())
for name,digest in plan['source_sha256'].items():assert hashlib.sha256((Path('/source')/name).read_bytes()).hexdigest()==digest,name
if not output.exists():runpy.run_path(str(root/'source/evaluate_target_probes.py'))['evaluate'](root,output)
subprocess.run([sys.executable,'/source/tmp/plot_skip_probe_validation.py',str(root),str(output)],check=True)
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
s=json.loads((output/'summary.json').read_text());fig,axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
axes[0].bar(['Old proxy','Measured hold v4','Actual hold'],[14,0,0],color='white',edgecolor='black',hatch='//')
axes[0].set(title='Development: same-pose hold support',ylabel='Cell instances predicted/recovered')
pairs=s['pairs'];names=sorted({p['pair'] for p in pairs})
for i,name in enumerate(names):
    for j,scope in enumerate(('shared_targets','all_shared_deficient')):
        p=next(p for p in pairs if p['pair']==name and p['scope']==scope)
        axes[1].bar(i+(j-.5)*.35,p['net'],.35,color='white',edgecolor='black',hatch='//' if j else '',label=scope if i==0 else None)
axes[1].set_xticks(range(len(names)),names);axes[1].set(title='Fresh probes: turn minus hold (10s)',ylabel='Net recovered cell instances')
axes[1].axhline(0,color='black',linewidth=.5)
if names:axes[1].legend(fontsize=8)
fig.supxlabel('Development and fresh captures are separate. No current-target improvement does not imply zero global benefit.',fontsize=8)
fig.savefig(output/'results.png',dpi=160);plt.close(fig)
shutil.copyfile(__file__,output/'finish_target_probes.py')
print('TARGET ANALYSIS AND FIGURES FINISHED')
