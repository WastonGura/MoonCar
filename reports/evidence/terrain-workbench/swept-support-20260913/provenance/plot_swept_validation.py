import sys,json,csv
from pathlib import Path
sys.path.insert(0,'/source/scripts')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from swept_resampling import snapshot
captures=Path(sys.argv[1]);output=Path(sys.argv[2]);summary=json.loads((output/'summary.json').read_text())
with (output/'cells.csv').open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
for trial in summary['trials']:
    if trial['status']!='evaluated':continue
    folder=captures/trial['folder'];capture=folder/'approach'
    with np.load(output/(trial['folder']+'-decision.npz'),allow_pickle=False) as data:before=data['cloud'].copy()
    terrain,camera,robot=snapshot(capture,trial['stage_frames']['final']);after=terrain.cloud()
    selected=[r for r in rows if r['folder']==trial['folder']]
    fig,axes=plt.subplots(1,3,figsize=(14,4.5),layout='constrained')
    img=plt.imread(folder/'approach-end.jpg');axes[0].imshow(img[...,:3].mean(axis=2),cmap='gray');axes[0].axis('off');axes[0].set_title('Actual Gazebo capture (end)')
    low=np.minimum(before[:,:2].min(axis=0),after[:,:2].min(axis=0))-.3;high=np.maximum(before[:,:2].max(axis=0),after[:,:2].max(axis=0))+.3
    for ax,cloud,title in zip(axes[1:],(before,after),('Observed points at decision','Observed points at end')):
        ax.scatter(cloud[:,0],cloud[:,1],c=cloud[:,2],cmap='gray',vmin=-.1,vmax=.4,s=.6,rasterized=True)
        ax.set(xlim=(low[0],high[0]),ylim=(low[1],high[1]),xlabel='X (m)',ylabel='Y (m)',title=title);ax.set_aspect('equal')
    for ax,key,label,marker in [(axes[1],'swept_prediction','Predicted support','s'),(axes[2],'recovered','Actual fit recovery','o')]:
        cells=np.array([[(int(r['ix'])+.5)*.25,(int(r['iy'])+.5)*.25] for r in selected if r[key]=='True']).reshape(-1,2)
        ax.scatter(cells[:,0],cells[:,1],facecolors='none',edgecolors='black',s=35,marker=marker,linewidths=.6,label=f'{label}: {len(cells)}');ax.legend(fontsize=7)
    fig.suptitle(trial['folder']);fig.supxlabel('Forecast markers are not measured points. Only the turn arm executes the forecast rotation.',fontsize=8)
    fig.savefig(output/(trial['folder']+'-scene-map.png'),dpi=150);plt.close(fig)
selected=[g for g in summary['metrics'] if g['scope']=='all_decision_deficient' and g['arm']=='turn']
fig,ax=plt.subplots(figsize=(8,4),layout='constrained')
for i,g in enumerate(selected):
    bottom=0
    for key,hatch,label in [('tp','','Correct recovery'),('fp','//','False recovery prediction'),('fn','..','Missed recovery')]:
        ax.bar(i,g[key],bottom=bottom,color='white',edgecolor='black',hatch=hatch,label=label if i==0 else None);bottom+=g[key]
ax.set_xticks(range(len(selected)),['Old v1','Swept visibility','Ray support v2','Marginal support']);ax.set_ylabel('Decision-deficient cell instances');ax.legend(fontsize=8);ax.set_title('Independent captures: prediction errors in executed turn arms')
fig.supxlabel('All observed decision-deficient cells; correlated cells are not independent trials.',fontsize=8)
fig.savefig(output/'prediction-errors.png',dpi=160);plt.close(fig)
pairs=[p for p in summary['pairs'] if p.get('scope')=='all_decision_deficient' and p['status']=='evaluated']
fig,ax=plt.subplots(figsize=(8,4),layout='constrained')
for i,p in enumerate(pairs):
    ax.bar(i-.18,p['turn_only'],.36,color='white',edgecolor='black',hatch='//',label='Turn-only recovery' if i==0 else None)
    ax.bar(i+.18,p['hold_only'],.36,color='white',edgecolor='black',label='Hold-only recovery' if i==0 else None)
ax.set_xticks(range(len(pairs)),[p['pair'].replace('visible_','').replace('_v1','')+'\nshared n='+str(p['common_remaining']) for p in pairs]);ax.set_ylabel('Shared deficient cells');ax.set_title('Recovery after the decision, matched capture budgets');ax.legend(fontsize=8)
fig.supxlabel('Only shared decision-deficient cells; different trajectories limit causal interpretation.',fontsize=8)
fig.savefig(output/'paired-recovery.png',dpi=160);plt.close(fig)
print('Saved observed-map comparisons and prediction/paired-result charts')
