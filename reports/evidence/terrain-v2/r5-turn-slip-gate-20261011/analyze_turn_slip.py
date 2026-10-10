"""Read-only post-run summaries; no changes to qualified round results."""
import json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,'/source/scripts')
from r5_origin_analysis import unwrap_yaw

p=Path('/evidence');campaign=p/'campaign'
ledger=json.loads((campaign/'ledger.json').read_text());metrics=[]
fig,axes=plt.subplots(2,2,figsize=(12,8))
for e in ledger:
    folder=campaign/e['name'];path=folder/'result.json'
    if not path.is_file():
        metrics.append(dict(name=e['name'],qualified=False,error=e.get('error')));continue
    r=json.loads(path.read_text());proof=r.get('turn_slip_probe',{})
    item=dict(name=e['name'],profile=e['profile'],scene=e['scene'],step=e['step'],case=e['case'],
        qualified=e.get('qualified',False),wall_seconds=e.get('wall_seconds'),
        error=e.get('error'),local_evidence_bytes=sum(f.stat().st_size for f in folder.rglob('*') if f.is_file()))
    if proof:
        item.update(endpoint_xy_m=proof['endpoint_xy_m'],trajectory=proof['trajectory'],
            gate=proof['gate'],source=proof['source'],allocation=proof['allocation'],
            performance=r.get('formal',{}).get('performance'),
            stop_drift_last_second_m=r.get('stop_drift_last_second_m'),parked=r.get('parked'),max_tilt_rad=r.get('max_tilt_rad'),
            origin_analysis=json.loads((folder/'origin-analysis/analysis.json').read_text())
                if (folder/'origin-analysis/analysis.json').is_file() else None)
        plots_only='--plots-only' in sys.argv
        data=np.genfromtxt(folder/('origin-analysis/trajectory.csv' if plots_only else 'physics-steps.csv'),delimiter=',',names=True)
        m=r['schedule']['motion'];active=(data['t']>=m['before']['sim']-1e-9)&(data['t']<=m['after']['sim']+1e-9)
        x=data['x'][active]-data['x'][active][0];y=data['y'][active]-data['y'][active][0]
        t=data['t'][active]-m['before']['sim']
        yaw=data['yaw_unwrapped'][active].copy() if plots_only else unwrap_yaw(data[active]);yaw-=yaw[0]
        if e['scene']=='ramp' and e['linear']:
            axes[0,0].plot(x,y*1000,label=e['name']);axes[0,1].plot(t,y*1000,label=e['name'])
        if e['angular']:
            axes[1,0].plot(x*1000,y*1000,label=e['name']);axes[1,1].plot(t,yaw,label=e['name'])
    metrics.append(item)
axes[0,0].set(xlabel='World X displacement (m)',ylabel='World Y displacement (mm)',title='Completed ramp straight: always / pure-turn')
axes[0,1].set(xlabel='Motion time (s)',ylabel='World Y displacement (mm)',title='Ramp lateral drift')
axes[1,0].set(xlabel='World X displacement (mm)',ylabel='World Y displacement (mm)',title='True body-center turn trajectories')
axes[1,1].set(xlabel='Motion time (s)',ylabel='Continuous yaw change (rad)',title='Actual rotation retained')
axes[1,0].set_aspect('equal',adjustable='box')
axes[1,0].set_xlim(-.2,.2);axes[1,0].set_ylim(-.2,.2)
for ax in axes.flat:
    ax.grid(True)
    if ax.lines:ax.legend(fontsize=5)
fig.tight_layout();fig.savefig(p/'turn-slip-comparison.png',dpi=180);plt.close(fig)
(p/'round-metrics.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')
print(json.dumps([dict(name=r['name'],qualified=r['qualified'],endpoint=r.get('endpoint_xy_m'),error=r.get('error')) for r in metrics],indent=2))
