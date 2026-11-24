"""Post-run read-only analysis. Does not rewrite the failed round verdict."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p = Path('/evidence')
folder = p / 'campaign/15-ramp-1000us-straight'
result = json.loads((folder / 'result.json').read_text())
data = np.genfromtxt(folder / 'physics-steps.csv', delimiter=',', names=True)
commands = np.genfromtxt(folder / '15-ramp-1000us-straight-axle-commands.csv', delimiter=',', names=True)
start = result['schedule']['parked']['after']['sim']
end = result['schedule']['motion']['after']['sim']
moving = data[(data['t'] >= start-1e-9) & (data['t'] <= end+1e-9)]
xy = np.column_stack([moving[k]-moving[k][0] for k in ('x','y')])
wheels = ('front_left','front_right','rear_left','rear_right')
gaps = {}
for wheel in wheels:
    times = []
    with (folder / (wheel+'-contact.jsonl')).open() as stream:
        for line in stream:
            stamp = json.loads(line).get('header',{}).get('stamp',{})
            times.append(int(stamp.get('sec',0))*1_000_000_000 + int(stamp.get('nsec',0)))
    gaps[wheel] = [dict(before_ns=a,after_ns=b,duration_s=(b-a)/1e9)
                   for a,b in zip(times,times[1:]) if b-a > 100_001_000]
allocation_difference = max(float(np.max(np.abs(commands[w+'_native']-commands[w+'_allocated']))) for w in wheels)
summary = dict(original_verdict_passed=result['passed'],original_verdict_unchanged=True,
    motion_start_s=start,motion_end_s=end,endpoint_xy_displacement_m=xy[-1].tolist(),
    allocation_applied_steps=int(np.count_nonzero(commands['applied'])),
    native_allocated_max_difference_rad_s=allocation_difference,
    contact_gaps_above_100ms=gaps,
    interpretation='Contact topic is event-sparse; current data cannot distinguish no-contact from missing reception. Do not impute zero force.',
    causality='No matched no-WheelSlip ramp round in this phase. Lateral displacement cannot be causally attributed to WheelSlip here.')
(p/'failure-analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
fig,axes=plt.subplots(1,2,figsize=(11,4))
axes[0].plot(xy[:,0],xy[:,1]);axes[0].set(xlabel='World X displacement (m)',ylabel='World Y displacement (m)',title='Failed ramp straight round: actual body trajectory')
axes[0].grid(True)
for i,wheel in enumerate(wheels):
    for gap in gaps[wheel]:
        axes[1].plot([gap['before_ns']/1e9,gap['after_ns']/1e9],[i,i],linewidth=5)
axes[1].set(yticks=range(4),yticklabels=wheels,xlabel='Simulation time (s)',title='Contact reception gaps > 100 ms')
fig.tight_layout();fig.savefig(p/'failed-ramp-trajectory.png',dpi=160);plt.close(fig)
print(json.dumps(dict(passed=result['passed'],endpoint_xy_m=xy[-1].tolist(),allocation_applied_steps=summary['allocation_applied_steps'],max_target_difference=allocation_difference)))
