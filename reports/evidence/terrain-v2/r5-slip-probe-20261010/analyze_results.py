"""Post-experiment read-only visualization; never executed by control nodes."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

e=Path('/evidence');c=e/'campaign'
ledger=json.loads((c/'ledger.json').read_text());rows=[]
fig,axes=plt.subplots(2,2,figsize=(12,9))
zoom,zaxes=plt.subplots(1,2,figsize=(11,4.5))
for entry in ledger:
    folder=c/entry['name'];path=folder/'result.json'
    result=json.loads(path.read_text()) if path.is_file() else {}
    probe=result.get('slip_probe',{});m=probe.get('trajectory',{})
    rows.append(dict(name=entry['name'],qualified=entry.get('qualified',False),profile=entry['profile'],
        linear=entry['linear'],angular=entry['angular'],trajectory=m,contact=probe.get('contact'),
        slip_commands=probe.get('commands'),wall_seconds=entry.get('wall_seconds'),
        performance=result.get('formal',{}).get('performance'),
        parked=result.get('parked'),stop_drift_last_second_m=result.get('stop_drift_last_second_m'),
        sensor_cadence=result.get('formal',{}).get('sensors'),safe_gap_s=result.get('formal',{}).get('safe_gap_s'),
        error=entry.get('error')))
    if not entry['angular'] or not entry.get('qualified') or not m:continue
    data=np.genfromtxt(folder/'origin-analysis/trajectory.csv',delimiter=',',names=True)
    x=data['x']-data['x'][0];y=data['y']-data['y'][0];t=data['t']-data['t'][0]
    index=0 if entry['angular']>0 else 1
    axes[0,index].plot(x,y,label=entry['name'])
    axes[1,index].plot(t,np.hypot(x,y)*100,label=entry['name'])
    if entry['profile']=='candidate':zaxes[index].plot(x*100,y*100,label=entry['case'])
for index,label in enumerate(('Left turn','Right turn')):
    axes[0,index].set(title=label,xlabel='World X displacement (m)',ylabel='World Y displacement (m)')
    axes[0,index].axis('equal')
    axes[1,index].set(xlabel='Motion time (s)',ylabel='Center displacement (cm)')
    zaxes[index].set(title=label+' candidate',xlabel='World X displacement (cm)',ylabel='World Y displacement (cm)')
    zaxes[index].axis('equal')
for ax in [*axes.flat,*zaxes]:
    ax.grid(True);ax.legend(fontsize=7)
fig.tight_layout();fig.savefig(e/'trajectory-comparison.png',dpi=160);plt.close(fig)
zoom.tight_layout();zoom.savefig(e/'candidate-trajectories.png',dpi=160);plt.close(zoom)
(e/'round-metrics.json').write_text(json.dumps(rows,indent=2)+'\n')
print(json.dumps([dict(name=r['name'],qualified=r['qualified'],max_m=r['trajectory'].get('maximum_displacement_m'),
                     yaw_rad=r['trajectory'].get('unwrapped_turn_rad'),
                     icr_x_m=(r.get('contact') or {}).get('icr_body_x_median_m')) for r in rows],indent=2))
