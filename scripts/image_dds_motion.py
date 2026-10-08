"""Test-only input qualification by physical intervals, not callback observation time."""
import numpy as np
from r5_d0_analysis import INPUT_FIELDS

def physical_command_window(data,result,linear,angular,original):
    window=original(data,result,linear,angular)
    if window.get('mode')!='paused_steps' or not (linear or angular):return window
    active=data[(data['t']>=result['sim_started'])&(data['t']<=result['sim_ended'])]
    if not (np.isfinite(active['dt']).all() and (active['dt']>0).all()):return dict(window,passed=False,reason='Invalid physical timestep')
    expected=np.array([linear,0,0,0,0,angular]);values=np.column_stack([active[f] for f in INPUT_FIELDS])
    moving=np.max(np.abs(values-expected),axis=1)<=1e-12
    indices=np.flatnonzero(moving)
    if not len(indices):return dict(window,passed=False,reason='No physical input interval')
    # PostUpdate records the input sampled before the step with endpoint t.
    # Each value therefore applies to (t-dt,t], including paused boundaries.
    t=np.rint(active['t']*1e9).astype(np.int64);dt=np.rint(active['dt']*1e9).astype(np.int64)
    step_ns=round(window['step']*1e9)
    grid=bool((dt==step_ns).all() and (np.diff(t)==dt[1:]).all())
    first,last=int(indices[0]),int(indices[-1]);start_ns=int(t[first]-dt[first]);stop_ns=int(t[last])
    duration_ns=int(dt[moving].sum());onset_ns=start_ns-round((result['sim_started']+5)*1e9)
    zeros=np.max(np.abs(values),axis=1)<=1e-12
    sequence=bool(window['fresh'] and window['contiguous'] and window['stop_zero'] and
                  (zeros|moving).all() and zeros[:first].all() and window['wheel_budget']['passed'])
    # A forged future observation time is still invalid; this field describes
    # when PreUpdate noticed a message, not a transport publication timestamp.
    future=bool(np.max(active['input_seen_sim']-active['t'])>window['step']+1e-9)
    motion=result['schedule']['motion']
    phase_ns=round((motion['after']['sim']-motion['before']['sim'])*1e9)
    timing=bool(grid and not future and abs(onset_ns)<=step_ns and abs(duration_ns-20000000000)<=step_ns and
                stop_ns-start_ns==duration_ns and phase_ns==20000000000)
    raw={key:window[key] for key in ('start','stop','duration_s','onset_error_s')}
    return dict(window,passed=sequence and timing,start=start_ns/1e9,stop=stop_ns/1e9,duration_s=duration_ns/1e9,
                onset_error_s=onset_ns/1e9,observed_message_times=raw,physical_motion_steps=len(indices),
                physical_input_grid_complete=grid,time_basis='Native input sampled for each physical interval (t-dt,t]',
                reason='' if sequence and timing else 'Native physical input sequence, grid or duration anomaly')
