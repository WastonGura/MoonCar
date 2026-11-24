"""Offline report from frozen round receipts; no control or gate changes."""
import argparse
import json
import math
from pathlib import Path

import numpy as np


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def run(root):
    root=Path(root);campaign=root/'campaign'
    summary=json.loads((campaign/'summary.json').read_text())
    ledger=json.loads((campaign/'ledger.json').read_text())
    if summary['status']!='completed' or len(ledger)!=12 or not all(e['passed'] for e in ledger):
        raise ValueError('Full qualified twelve-round matrix required; stopped results remain in campaign ledger')
    if not json.loads((campaign/'final-freeze-check.json').read_text())['passed']:
        raise ValueError('Final freeze verification failed')
    items=[];paths={};phase_comparisons=[]
    for entry in ledger:
        folder=campaign/entry['name']
        result=json.loads((folder/'result.json').read_text())
        metrics=json.loads((folder/'origin-analysis/analysis.json').read_text())
        audit=json.loads((folder/'load-analysis/audit.json').read_text())
        sensors=result['formal']['sensors'];performance=result['formal']['performance']
        item=dict(name=entry['name'],stage=entry['stage'],angular=entry['angular'],qualified=entry['passed'],
            endpoint_m=metrics['endpoint_displacement_m'],maximum_m=metrics['maximum_displacement_m'],path_length_m=metrics['path_length_m'],
            actual_turn_rad=metrics['unwrapped_turn_rad'],stop_extra_m=metrics['stop_extra_displacement_m'],
            last_second_stop_m=result['stop_drift_last_second_m'],odometry=metrics['odometry'],audit=audit,
            input_pairs=entry.get('input_pairs',[]),image_count=len(result['image_content']['content']),
            sensor_intervals={k:v['max_gap_s'] for k,v in sensors.items()},safe_gap_s=result['formal']['safe_gap_s'],
            paused_flush_passed=result['schedule']['csv_flush']['passed'],queues_drained=result['capture']['drained'],
            images_readable=result['image_content']['passed'],runtime_receivers_verified=result['receiver_verified'],
            wall_seconds=entry['wall_seconds'],performance=performance,
            evidence_bytes=sum(p.stat().st_size for p in folder.rglob('*') if p.is_file()))
        items.append(item)
        trajectory=np.genfromtxt(folder/'origin-analysis/trajectory.csv',names=True,delimiter=',')
        theta=float(trajectory['yaw_unwrapped'][0]);rot=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
        xy=np.column_stack((trajectory['x'],trajectory['y']));paths[entry['name']]=(xy-xy[0])@rot
    for one,two in (('02-baseline-left-1','04-baseline-left-2'),('03-baseline-right-1','05-baseline-right-2'),
                    ('07-mirror-left-1','09-mirror-left-2'),('08-mirror-right-1','10-mirror-right-2'),
                    ('02-baseline-left-1','11-restore-left'),('03-baseline-right-1','12-restore-right')):
        if paths[one].shape!=paths[two].shape:
            raise ValueError('Motion sample grids differ')
        difference=np.linalg.norm(paths[one]-paths[two],axis=1)
        a=next(r for r in items if r['name']==one);b=next(r for r in items if r['name']==two)
        phase_comparisons.append(dict(one=one,two=two,maximum_path_difference_m=float(difference.max()),
            endpoint_difference_m=float(difference[-1]),turn_difference_rad=abs(a['actual_turn_rad']-b['actual_turn_rad'])))
    decision=summary['decision']
    save(root/'decision.json',dict(**decision,attempts=12,qualified=12,
        load_explanation_supported=decision['cause_confirmed'],load_only_cause_proven=False,
        displacement_eliminated=False,next_action='Review physical mass placement and separately plan slip-aware estimation/control; no production publication'))
    save(root/'findings.json',dict(rounds=items,trajectory_comparisons=phase_comparisons,decision=decision,
        source='Fresh matrix only; archived preflight observations excluded',total_images=sum(r['image_count'] for r in items),
        min_rtf=min(r['performance']['realtime_factor'] for r in items),max_rtf=max(r['performance']['realtime_factor'] for r in items),
        maximum_wall_seconds=max(r['wall_seconds'] for r in items)))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(12,9))
    colors={'baseline':'#2764ad','mirror':'#d27815','restore':'#39824b'}
    for side,index in (('left',0),('right',1)):
        ax=axes[0,index]
        for name in (f'{2+index:02d}-baseline-{side}-1',f'{7+index:02d}-mirror-{side}-1',f'{11+index:02d}-restore-{side}'):
            stage=next(e['stage'] for e in ledger if e['name']==name);xy=paths[name]
            ax.plot(xy[:,0],xy[:,1],label=stage,color=colors[stage],linestyle='--' if stage=='restore' else '-',linewidth=2)
        ax.scatter([0],[0],marker='x',color='black',label='start')
        ax.set(title=side.capitalize()+' turn: body-center trajectory',xlabel='Initial body X (m)',ylabel='Initial body Y (m)')
        ax.axis('equal');ax.legend(fontsize=9);ax.grid(alpha=.2)
    phases=['baseline','mirror','restore'];groups=[[r for r in items if r['stage']==p and r['angular']!=0] for p in phases]
    loads=[np.mean([r['audit']['rear_minus_front_N'] for r in group]) for group in groups]
    axes[1,0].bar(phases,loads,color=[colors[p] for p in phases]);axes[1,0].axhline(0,color='black',linewidth=.8)
    axes[1,0].set(title='Static rear minus front normal load',ylabel='Load difference (N)')
    x=np.arange(3)
    for side,offset,color in (('front',-.18,'#5466a5'),('rear',.18,'#b56e55')):
        slips=[np.mean([r['audit'][side+'_lateral_rms_m_s'] for r in group]) for group in groups]
        axes[1,1].bar(x+offset,slips,width=.36,label=side,color=color)
    axes[1,1].set(xticks=x,xticklabels=phases,title='Steady lateral slip: timestamp-first RMS',ylabel='Lateral slip RMS (m/s)')
    axes[1,1].set_ylim(0,.075)
    axes[1,1].legend();fig.suptitle('COM intervention changes pivot axle; net displacement persists',fontsize=13)
    fig.tight_layout();fig.savefig(root/'mass-pivot-comparison.png',dpi=170);plt.close(fig)
    print(json.dumps(dict(decision=decision,rounds=[{k:r[k] for k in ('name','endpoint_m','actual_turn_rad','wall_seconds')} for r in items]),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--evidence',default='/evidence');run(p.parse_args().evidence)
