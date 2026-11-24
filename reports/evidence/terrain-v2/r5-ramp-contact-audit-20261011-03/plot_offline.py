"""Scientific plots from verified archived data; no simulation execution."""
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, '/source/scripts')
from audit_r5_ramp_contact import read_obj, classified_edges, normal_angle, digest
p = Path('/probe')
proof = json.loads((p/'analysis.json').read_text())
fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
for item in proof['results']:
    name = item['name']; data = np.genfromtxt(p/(name+'-steps.csv'), delimiter=',', names=True)
    t = data['t'] - 5.2
    label = 'Flat plate' if 'flat' in name else 'Ramp mesh (failed round)'
    axes[0].plot(t, np.hypot(data['dx_m'], data['dy_m'])*1000, label=label)
    axes[1].plot(t, np.degrees(data['max_normal_difference_rad']), label=label, linewidth=.6)
    axes[2].plot(t, data['contact_points'], label=label, linewidth=.4)
axes[0].axhline(10, color='black', linestyle='--', label='Frozen 10 mm screen')
axes[0].set_ylabel('Body-center displacement (mm)')
axes[1].set_ylabel('Contact normal difference (deg)')
axes[2].set_ylabel('Source contact points / step')
axes[2].set_xlabel('Actual motion time (simulation s)')
for ax in axes: ax.grid(True); ax.legend(fontsize=8, loc='upper left')
fig.suptitle('Archived comparison: coplanar support, different contact representation')
fig.tight_layout(); fig.savefig(p/'contact-timeline.png', dpi=170); plt.close(fig)

folder = Path('/source/reports/evidence/terrain-v2/r5-turn-gate-validation-20261011/campaign/14-pure_turn-ramp-1000us-left')
rows = np.genfromtxt(folder/(folder.name+'-contact-steps.csv'), delimiter=',', names=True)
rows = rows[(rows['iteration']>5200)&(rows['iteration']<=25200)&(rows['state']==1)]
points = np.column_stack([rows[k] for k in ('x','y','z')])
normals = np.column_stack([rows[k] for k in ('nx','ny','nz')])
angle = np.degrees(normal_angle(normals, np.tile([0.,0.,1.], (len(rows),1))))
v,f = read_obj('/source/ros2_ws/src/mooncar_gazebo/models/terrain_v2/ramp/regolith_passable.obj')
tri = v[f]; lo,hi = points[:,:2].min(axis=0),points[:,:2].max(axis=0)
select = ((tri[:,:,:2].min(axis=1)<=hi).all(axis=1)&(tri[:,:,:2].max(axis=1)>=lo).all(axis=1))
edges = classified_edges(v, f[select])
fig,ax = plt.subplots(figsize=(8,7))
ax.scatter(points[::35,0],points[::35,1],s=1,color='lightgray',label='Contact point sample')
for kind,color in (('shared_indices','black'),('duplicated_indices','blue')):
    for i,edge in enumerate(edges[kind]):
        ax.plot(edge[:,0],edge[:,1],color=color,linestyle='--',linewidth=.8,label=kind if i==0 else None)
tilted = angle > np.degrees(1e-6)
plot=ax.scatter(points[tilted,0],points[tilted,1],s=9,c=angle[tilted],cmap='inferno',label='All tilted-normal points',zorder=4)
fig.colorbar(plot,ax=ax,label='Difference from geometric surface normal (deg)')
ax.set(xlim=(-.28,.28),ylim=(-.28,.28),xlabel='World X (m)',ylabel='World Y (m)',title='Contact normal deviations near internal triangle seams')
ax.set_aspect('equal');ax.grid(True);ax.legend(fontsize=8)
fig.tight_layout();fig.savefig(p/'contact-normal-map.png',dpi=180);plt.close(fig)
