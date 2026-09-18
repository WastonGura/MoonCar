import sys,json
from pathlib import Path
sys.path.insert(0,'/source/scripts')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from swept_resampling import snapshot
root,output=map(Path,sys.argv[1:3]);summary=json.loads((output/'summary.json').read_text());plan=json.loads((root/'plan.json').read_text())
valid=[t for t in summary['trials'] if t['status']=='evaluated']
fig,axes=plt.subplots(1,3,figsize=(15,5),layout='constrained')
labels=[t['folder'].replace('ramp-repeat','R').replace('_turn','') for t in valid];x=np.arange(len(valid))
for ax,key,title in zip(axes,('new_observed_m2','recovered_surface_m2','estimated_path_m'),('New observed area (m2)','Recovered old deficient area (m2)','Sampled estimated travel (m)')):
    ax.bar(x,[t[key] for t in valid],color='white',edgecolor='black',hatch='//');ax.set_title(title)
    ax.set_xticks(x,labels,rotation=55,ha='right',fontsize=8);ax.grid(axis='y',alpha=.2)
fig.suptitle('Equal 20 simulation seconds from dispatch, including movement and turn')
fig.supxlabel('Descriptive per-run changes; unequal starting cohorts. Failed runs are retained in the report, not zero gains.',fontsize=9)
fig.savefig(output/'action-value.png',dpi=160);plt.close(fig)
if summary['paired']:
    fig,ax=plt.subplots(figsize=(10,4.5),layout='constrained');rows=summary['paired']
    ax.bar(range(len(rows)),[r['net_utility_vs_hold_m2'] for r in rows],color='white',edgecolor='black',hatch='//')
    ax.set_xticks(range(len(rows)),[f"R{r['repeat']} {r['arm']}" for r in rows],rotation=25,ha='right')
    ax.set(ylabel='Positive utility difference versus hold (m2)',title='Common cohorts across four independently reset arms');ax.axhline(0,color='black',lw=.7)
    fig.supxlabel('Shared new/deficient cells; validity losses reported separately. Not per-cell causal truth.',fontsize=9)
    fig.savefig(output/'shared-cohorts.png',dpi=160);plt.close(fig)
for trial in plan['trials']:
    result=next(t for t in summary['trials'] if t['folder']==trial['folder'])
    if result['status']!='evaluated':continue
    folder=root/trial['folder'];capture=folder/'approach';control=json.loads((folder/'approach-control.json').read_text());event=control['comparison']['decision_stamp']
    with np.load(capture/'samples.npz',allow_pickle=False) as data:
        cuts=[int(np.searchsorted(data['stamp'],s,side='right')) for s in (event,event+20)]
        track=data['estimated_robot'][cuts[0]-1:cuts[1],:2,3]
    before,_,_=snapshot(capture,cuts[0]);after,_,_=snapshot(capture,cuts[1]);clouds=[before.cloud(),after.cloud()]
    fig,axes=plt.subplots(1,3,figsize=(14,4.5),layout='constrained')
    shot=folder/'approach-end.jpg'
    if shot.exists():axes[0].imshow(plt.imread(shot)[...,:3].mean(axis=2),cmap='gray')
    axes[0].axis('off');axes[0].set_title('Actual Gazebo scene (capture end)')
    points=np.vstack(clouds);low=points[:,:2].min(axis=0)-.3;high=points[:,:2].max(axis=0)+.3
    for ax,cloud,title in zip(axes[1:],clouds,('Observed map at action dispatch','Observed map after equal 20 seconds')):
        ax.scatter(cloud[:,0],cloud[:,1],c=cloud[:,2],cmap='gray',s=.7,vmin=-.1,vmax=.3,rasterized=True)
        ax.set(xlim=(low[0],high[0]),ylim=(low[1],high[1]),aspect='equal',xlabel='X (m)',ylabel='Y (m)',title=title)
    axes[2].plot(track[:,0],track[:,1],'k--',lw=1,label='Estimated vehicle track');axes[2].legend(fontsize=8)
    fig.suptitle(trial['folder']);fig.supxlabel('Scene is asynchronous context; both maps contain measured points only. Blank does not mean traversable.',fontsize=9)
    fig.savefig(output/(trial['folder']+'-scene-map.png'),dpi=150);plt.close(fig)
print('Saved measured action charts and scene/observed-map panels')
