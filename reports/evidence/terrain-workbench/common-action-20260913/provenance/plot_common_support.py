import sys,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(sys.argv[1]);s=json.loads((root/'summary.json').read_text());rows=s['forecasts']
fig,axes=plt.subplots(1,2,figsize=(12,4.8),layout='constrained');x=np.arange(len(rows));labels=[r['folder'].replace('ramp-repeat','R').replace('_turn','') for r in rows]
for key,offset,hatch in [('tp',-.23,''),('fp',0.,'//'),('fn',.23,'..')]:
    axes[0].bar(x+offset,[r[key] for r in rows],width=.23,color='white',edgecolor='black',hatch=hatch,label=key.upper())
axes[0].set_title('Recovery forecast: same model for every action');axes[0].legend();axes[0].set_ylabel('Observed cell count')
for key,offset,hatch in [('loss_tp',-.12,''),('loss_fn',.12,'//')]:
    axes[1].bar(x+offset,[r[key] for r in rows],width=.24,color='white',edgecolor='black',hatch=hatch,label=key)
axes[1].set_title('Valid-surface loss: detected / missed');axes[1].legend()
for ax in axes:ax.set_xticks(x,labels,rotation=30,ha='right');ax.grid(axis='y',alpha=.2)
fig.supxlabel('Fresh supplemental quartet; correlated cells, not calibrated probabilities or accuracy. Unknown-area forecast is absent.',fontsize=9)
fig.savefig(root/'common-support.png',dpi=170);plt.close(fig)
p=s['paired']
if p:
    fig,ax=plt.subplots(figsize=(9,4.5),layout='constrained');x=np.arange(len(p))
    ax.bar(x-.17,[r['net_utility_vs_hold_m2'] for r in p],width=.34,color='white',edgecolor='black',label='Positive gain difference')
    ax.bar(x+.17,[r['loss_adjusted_difference_m2'] for r in p],width=.34,color='white',edgecolor='black',hatch='//',label='After shared validity losses')
    ax.set_xticks(x,[r['arm'] for r in p]);ax.set_ylabel('Difference from holding (m2)');ax.legend()
    ax.set_title('Same20s, common new/deficient/initially-valid cohorts');ax.axhline(0,color='black',lw=.7)
    fig.supxlabel('Observed projected area metric; not accuracy, risk, or universal optimality.',fontsize=9)
    fig.savefig(root/'loss-adjusted.png',dpi=170);plt.close(fig)
print('Saved forecast errors and loss-adjusted common-cohort chart')
