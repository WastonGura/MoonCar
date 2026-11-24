"""Read-only post-run analysis, outside the frozen command path."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path('/evidence');c=p/'campaign';ledger=json.loads((c/'ledger.json').read_text());rows=[]
fig,axes=plt.subplots(2,2,figsize=(12,8))
worlds,wa=plt.subplots(1,3,figsize=(15,5))
for entry in ledger:
    folder=c/entry['name'];path=folder/'result.json'
    r=json.loads(path.read_text()) if path.is_file() else {}
    probe=r.get('axle_probe',{});trajectory=probe.get('trajectory',{})
    analysis=folder/'origin-analysis/analysis.json'
    odom=json.loads(analysis.read_text()).get('odometry') if analysis.is_file() else None
    rows.append(dict(name=entry['name'],scene=entry['scene'],step=entry['step'],case=entry['case'],
        linear=entry['linear'],angular=entry['angular'],qualified=entry.get('qualified',False),
        error=entry.get('error'),wall_seconds=entry.get('wall_seconds'),trajectory=trajectory,
        contact=probe.get('contact'),allocation=probe.get('allocation'),
        longitudinal_slip_rms_m_s=probe.get('longitudinal_slip_rms_m_s'),
        turn_screen_passed=trajectory.get('maximum_displacement_m',float('inf'))<=.01 if entry['angular'] else None,
        odometry=odom,performance=r.get('formal',{}).get('performance'),
        stop_drift_last_second_m=r.get('stop_drift_last_second_m'),parked=r.get('parked'),
        max_tilt_rad=r.get('max_tilt_rad'),actual_selection=r.get('robustness_probe'),
        sensor_cadence=r.get('formal',{}).get('sensors'),safe_gap_s=r.get('formal',{}).get('safe_gap_s'),
        local_evidence_bytes=sum(x.stat().st_size for x in folder.rglob('*') if x.is_file())))
    source=folder/'origin-analysis/trajectory.csv'
    if not entry['angular'] or not entry.get('qualified') or not source.is_file():continue
    d=np.genfromtxt(source,delimiter=',',names=True);x=(d['x']-d['x'][0])*1000;y=(d['y']-d['y'][0])*1000
    t=d['t']-d['t'][0]
    if entry['scene']=='flat':
        index=0 if entry['angular']>0 else 1;label=f"{entry['step']*1000:g} ms / {entry['case']}"
        axes[0,index].plot(x,y,label=label);axes[1,index].plot(t,np.hypot(x,y),label=label)
    else:wa[('ramp','curve','survey').index(entry['scene'])].plot(x,y,label=entry['case'])
for i,direction in enumerate(('Left','Right')):
    axes[0,i].set(title=direction+' / flat',xlabel='World X displacement (mm)',ylabel='World Y displacement (mm)')
    axes[0,i].axis('equal');axes[1,i].set(xlabel='Motion time (s)',ylabel='Center displacement (mm)')
for ax,scene in zip(wa,('ramp','curve','survey')):
    ax.set(title=scene+' v2',xlabel='World X displacement (mm)',ylabel='World Y displacement (mm)');ax.axis('equal')
for ax in [*axes.flat,*wa]:
    ax.grid(True)
    if ax.lines:ax.legend(fontsize=7)
fig.tight_layout();fig.savefig(p/'flat-step-trajectories.png',dpi=160);plt.close(fig)
worlds.tight_layout();worlds.savefig(p/'v2-trajectories.png',dpi=160);plt.close(worlds)
(p/'round-metrics.json').write_text(json.dumps(rows,indent=2)+'\n')
print(json.dumps([dict(name=x['name'],qualified=x['qualified'],max_m=x['trajectory'].get('maximum_displacement_m'),
    yaw=x['trajectory'].get('unwrapped_turn_rad')) for x in rows],indent=2))
