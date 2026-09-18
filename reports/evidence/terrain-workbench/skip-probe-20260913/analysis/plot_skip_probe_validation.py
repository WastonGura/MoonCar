"""Show candidate availability and actual observed maps, including unexecuted probes."""
import sys,json
from pathlib import Path
sys.path.insert(0,'/source/scripts')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from swept_resampling import snapshot
captures=Path(sys.argv[1]);output=Path(sys.argv[2]);plan=json.loads((captures/'plan.json').read_text());counts=[];labels=[]
for trial in plan['trials']:
    folder=captures/trial['folder'];capture=folder/'approach';path=capture/'selection.json'
    if not path.exists():continue
    selection=json.loads(path.read_text());row=[None]*len(plan['deltas'])
    for i,c in enumerate(selection['candidates']):row[i]=c['marginal_cells']
    counts.append(row);labels.append(trial['folder'].replace('visible_','').replace('_v1',''))
    meta=json.loads((capture/'manifest.json').read_text());checkpoint=json.loads((capture/'checkpoint.json').read_text())
    if meta['status']!='captured':continue
    before,_,_=snapshot(capture,checkpoint['frames']);after,_,_=snapshot(capture,meta['frames']);a=before.cloud();b=after.cloud()
    fig,axes=plt.subplots(1,3,figsize=(14,4.5),layout='constrained')
    image=folder/'approach-end.jpg'
    if image.exists():axes[0].imshow(plt.imread(image)[...,:3].mean(axis=2),cmap='gray')
    axes[0].axis('off');axes[0].set_title('Actual Gazebo capture (end)')
    low=np.minimum(a[:,:2].min(axis=0),b[:,:2].min(axis=0))-.3;high=np.maximum(a[:,:2].max(axis=0),b[:,:2].max(axis=0))+.3
    for ax,cloud,title in zip(axes[1:],(a,b),('Observed points before selection','Observed points at end')):
        ax.scatter(cloud[:,0],cloud[:,1],c=cloud[:,2],cmap='gray',vmin=-.1,vmax=.4,s=.6,rasterized=True)
        ax.set(xlim=(low[0],high[0]),ylim=(low[1],high[1]),xlabel='X (m)',ylabel='Y (m)',title=title);ax.set_aspect('equal')
    fig.suptitle(trial['folder']);fig.supxlabel('Observed accumulation is not evidence of a rotation benefit. No invented points in either map.',fontsize=8)
    fig.savefig(output/(trial['folder']+'-scene-map.png'),dpi=150);plt.close(fig)
fig,ax=plt.subplots(figsize=(10,4.8),layout='constrained');ax.axis('off')
table=ax.table(cellText=[['-' if v is None else str(v) for v in row] for row in counts],rowLabels=labels,
    colLabels=[f'{d:+.2f} rad' for d in plan['deltas']],loc='center',cellLoc='center');table.auto_set_font_size(False);table.set_fontsize(9);table.scale(1,1.6)
ax.set_title('Predicted marginal cells at the saved pre-action snapshot')
fig.supxlabel('0 with deficient cells admits a negative probe. Positive counts are hypotheses, not measured gains. - means not searched.',fontsize=8)
fig.savefig(output/'candidate-search.png',dpi=160);plt.close(fig)
print('Saved candidate table and actual scene/observed-map panels')
