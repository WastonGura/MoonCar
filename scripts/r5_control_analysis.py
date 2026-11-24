"""Fixed engineering profiles and independent per-physics-step acceptance."""
import math
import numpy as np
from r5_csv import read_numeric_csv

PROFILES={'legacy':(20.,.1,.5,1.,5.),'d0':(20.,.1,0.,1.,5.),
          'A':(.5,.1,0.,.04,.15),'B':(1.,.1,.001,.04,.15),'C':(.5,.2,.001,.04,.15)}

def analyze_steps(path,result):
    data=read_numeric_csv(path,delimiter=',',names=True)
    start,end=result.get('sim_started',0.),result.get('sim_ended',0.)
    active=data[(data['t']>=start)&(data['t']<=end)]
    if len(active)<2:return dict(passed=False,reason='Missing step observation')
    gaps=np.diff(active['iteration'])
    fields=[k for k in active.dtype.names if k.endswith(('_q','_v')) or
            k in ('x','y','z','qw','qx','qy','qz','camera_pan_joint_force_cmd','camera_tilt_joint_force_cmd')]
    complete=bool((gaps==1).all() and all(np.isfinite(active[k]).all() for k in fields))
    complete=bool(complete and active['t'][0]<=start+active['dt'][0]*1.01 and active['t'][-1]>=end-active['dt'][-1]*1.01)
    dt_error=float(np.max(np.abs(np.diff(active['t'])-active['dt'][1:])))
    holds=[]
    scans=result.get('sweep',[])
    if 'static' in result:scans=[dict(target=[0.,0.],finished=end-3)]
    for scan in scans:
        window=data[(data['t']>=scan['finished'])&(data['t']<=scan['finished']+3)]
        axes=[]
        for name,target in zip(('camera_pan_joint','camera_tilt_joint'),scan['target']):
            q,v=window[name+'_q'],window[name+'_v']
            if len(q)<2:axes.append(dict(passed=False,reason='Missing hold'));continue
            error=float(np.max(np.abs(q-target)))
            pp=float(np.ptp(q));rms=float(np.sqrt(np.mean(v*v)));maximum=float(np.max(np.abs(v)))
            axes.append(dict(joint=name,max_error_rad=error,peak_to_peak_rad=pp,velocity_rms=rms,max_velocity=maximum,
                             passed=error<=math.radians(1) and pp<=math.radians(.3) and rms<=.03 and maximum<=.1))
        coverage=len(window)>1 and window['t'][0]<=scan['finished']+window['dt'][0]*1.01 and window['t'][-1]>=scan['finished']+3-window['dt'][-1]*1.01
        holds.append(dict(target=scan['target'],axes=axes,coverage=bool(coverage),passed=bool(coverage) and all(a['passed'] for a in axes)))
    return dict(passed=complete and dt_error<1e-8 and all(h['passed'] for h in holds),
                complete=complete,steps=len(active),missed_steps=int(np.sum(np.maximum(gaps-1,0))),
                maximum_time_error_s=dt_error,holds=holds,
                maximum_gimbal_velocity={n:float(np.max(np.abs(active[n+'_v']))) for n in ('camera_pan_joint','camera_tilt_joint')},
                torque_is_command=True)
