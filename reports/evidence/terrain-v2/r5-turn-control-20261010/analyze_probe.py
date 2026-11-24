"""Post-simulation analysis only; never used by the controller."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from audit_r5_mass import aggregate, read_urdf
from r5_load_diagnostic import diagnose_round

e = Path('/evidence'); campaign = e/'campaign'
mass = aggregate(read_urdf(ET.parse(e/'r5-d0.urdf').getroot())[0])
expected = dict(center_of_mass_m=mass['com_m'])
ledger = json.loads((campaign/'ledger.json').read_text())
analyses = []
fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
for entry in ledger:
    folder = campaign/entry['name']
    audit = diagnose_round(folder, expected, entry['profile'])
    # Existing load tool's old support-exchange field is not this probe's test.
    # Retain it raw, but explicitly analyze axle-load sign and actual ICR here.
    series = {w:np.genfromtxt(folder/'load-analysis'/(w+'-contact-series.csv'), delimiter=',', names=True)
              for w in ('front_left','front_right','rear_left','rear_right')}
    times = series['front_left']['t']
    if any(not np.array_equal(times, a['t']) for a in series.values()):
        raise ValueError('Contact timestamps differ; cannot silently pair')
    delta = series['rear_left']['normal_load_N']+series['rear_right']['normal_load_N']-series['front_left']['normal_load_N']-series['front_right']['normal_load_N']
    sm = (times >= audit['steady_window_s'][0]) & (times <= audit['steady_window_s'][1])
    contact = dict(qualified=audit['qualified'], steady_rear_minus_front_min_N=float(delta[sm].min()),
                   steady_rear_minus_front_max_N=float(delta[sm].max()),
                   steady_front_heavier_fraction=float(np.mean(delta[sm] < -.1)),
                   icr_body_x_median_m=audit['icr_body_x_median_m'],
                   front_lateral_rms_m_s=audit['front_lateral_rms_m_s'],
                   rear_lateral_rms_m_s=audit['rear_lateral_rms_m_s'],
                   interpretation='Load sign reversal is not itself support exchange or a drift repair')
    analyses.append(dict(name=entry['name'], profile=entry['profile'], contact=contact))
    d = np.genfromtxt(folder/'origin-analysis/trajectory.csv', delimiter=',', names=True)
    index = 0 if entry['angular'] > 0 else 1
    axes[index].plot(d['x']-d['x'][0], d['y']-d['y'][0], label=entry['profile'])
    axes[index].set(xlabel='World X displacement (m)', ylabel='World Y displacement (m)',
                    title='Left turn' if index == 0 else 'Right turn')
    axes[index].axis('equal'); axes[index].grid(True); axes[index].legend()
fig.tight_layout(); fig.savefig(e/'trajectory-comparison.png', dpi=160); plt.close(fig)
(e/'contact-findings.json').write_text(json.dumps(analyses, indent=2)+'\n')
print(json.dumps(analyses, indent=2))
