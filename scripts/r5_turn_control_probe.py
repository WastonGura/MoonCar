"""Test-only drive hypothesis; no truth input and no physical parameter edits.

Bounded fore/aft acceleration may exchange the loaded axle during skid turns.
This is an experiment, not a released compensation or a hardware calibration.
"""
import math
import numpy as np
from r5_d0_analysis import INPUT_FIELDS, WHEELS, pose

AMPLITUDE = .025
ACCELERATION = .25
PERIOD = .4
DURATION = 20.


def dither(elapsed, angular):
    if not math.isfinite(elapsed) or not math.isfinite(angular):
        raise ValueError('Nonfinite command clock or angular request')
    if angular == 0. or elapsed <= 0. or elapsed >= DURATION:
        return 0.
    phase = elapsed % PERIOD
    quarter = PERIOD / 4
    if phase < quarter:
        return ACCELERATION * phase
    if phase < 3 * quarter:
        return AMPLITUDE - ACCELERATION * (phase - quarter)
    return -AMPLITUDE + ACCELERATION * (phase - 3 * quarter)


def command_window(data, result, linear, angular, unused_original=None):
    """Qualify intentional dither explicitly; preserve physical input gates.

    Dynamic longitudinal input is not mislabeled as zero linear input. Every
    native sample must be a published candidate command, and the wheel servo
    must retain the existing speed/acceleration limits and stopping behavior.
    """
    if linear != 0 or angular == 0:
        raise ValueError('Probe requires a zero-linear nonzero-angular request')
    schedule = result['schedule']; motion = schedule['motion']; step = schedule['step']
    active = data[(data['t'] >= result['sim_started']) & (data['t'] <= result['sim_ended'])]
    fields = INPUT_FIELDS + ('input_sequence', 'input_seen_sim') + tuple(w+'_wheel_joint_velocity_cmd' for w in WHEELS)
    if len(active) < 2 or any(f not in data.dtype.names for f in fields) or any(not np.isfinite(active[f]).all() for f in fields):
        return dict(passed=False, reason='Missing/nonfinite native command evidence')
    values = np.column_stack([active[f] for f in INPUT_FIELDS])
    zero = np.max(abs(values), axis=1) <= 1e-12
    moving = abs(values[:, 5]-angular) <= 1e-12
    indices = np.flatnonzero(moving)
    if not len(indices):
        return dict(passed=False, reason='No native turn interval')
    first, last = int(indices[0]), int(indices[-1])
    fresh = bool((active['input_sequence'] > 0).all() and (np.diff(active['input_sequence']) >= 0).all() and
                 np.max(active['t']-active['input_seen_sim']) <= .100001 and
                 np.max(active['input_seen_sim']-active['t']) <= step+1e-9)
    contiguous = bool(np.diff(indices).max(initial=1) == 1)
    off_axes = bool(np.max(abs(values[:, 1:5])) <= 1e-12)
    bounds = bool(np.max(abs(values[:, 0])) <= AMPLITUDE+1e-12)
    stop_zero = bool(last+1 < len(active) and zero[last+1:].all())
    t = np.rint(active['t']*1e9).astype(np.int64)
    dt = np.rint(active['dt']*1e9).astype(np.int64)
    grid = bool((dt == round(step*1e9)).all() and (np.diff(t) == dt[1:]).all())
    start_ns = int(t[first]-dt[first]); stop_ns = int(t[last]); duration_ns = int(dt[moving].sum())
    onset_ns = start_ns-round(motion['before']['sim']*1e9)
    timing = bool(grid and abs(onset_ns) <= round(step*1e9) and
                  abs(duration_ns-20_000_000_000) <= round(step*1e9) and stop_ns-start_ns == duration_ns and
                  int(motion['after']['iteration']-motion['before']['iteration']) == round(DURATION/step))
    wheels = np.column_stack([active[w+'_wheel_joint_velocity_cmd'] for w in WHEELS])
    left = (wheels[:, 0]+wheels[:, 2])/2; right = (wheels[:, 1]+wheels[:, 3])/2
    vx = (left+right)*.076/2; omega = (right-left)*.076/.3311
    matched = bool(np.max(abs(wheels[:, 0]-wheels[:, 2])) <= 1e-10 and np.max(abs(wheels[:, 1]-wheels[:, 3])) <= 1e-10)
    accel = [float(np.max(abs(np.diff(v)/active['dt'][1:]), initial=0)) for v in (vx, omega)]
    limits = bool(np.max(abs(vx)) <= AMPLITUDE+1e-8 and np.max(abs(omega)) <= abs(angular)+1e-8 and
                  np.min(omega*np.sign(angular)) >= -1e-8 and accel[0] <= .5+1e-6 and accel[1] <= 1.+1e-6)
    wheel_moving = np.flatnonzero(np.max(abs(wheels), axis=1) > 1e-9)
    wheel_contiguous = bool(len(wheel_moving) and np.diff(wheel_moving).max(initial=1) == 1)
    zero_tail = bool(np.max(abs(wheels[-1])) <= 1e-12)
    budget = dict(passed=matched and limits and wheel_contiguous and zero_tail,
                  continuous=wheel_contiguous, front_rear_targets_match=matched, zero_tail=zero_tail,
                  direction_and_acceleration_valid=limits, acceleration_max_m_s2_rad_s2=accel,
                  linear_integral_m=float(np.sum(vx*active['dt'])), angular_integral_rad=float(np.sum(omega*active['dt'])))
    valid = bool(fresh and contiguous and off_axes and bounds and stop_zero and (zero|moving).all() and
                 zero[:first].all() and timing and budget['passed'])
    return dict(passed=valid, fresh=fresh, contiguous=contiguous, stop_zero=stop_zero,
                mode='paused_steps', step=step, start=start_ns/1e9, stop=stop_ns/1e9,
                duration_s=duration_ns/1e9, onset_error_s=onset_ns/1e9,
                endpoint=pose(motion['last_step']), start_pose=pose(schedule['parked']['last_step']),
                wheel_budget=budget, physical_input_grid_complete=grid, physical_motion_steps=len(indices),
                dynamic_linear_input=True, maximum_native_linear_command_m_s=float(np.max(abs(values[:, 0]))),
                reason='' if valid else 'Dynamic command sequence, timing or original servo limits failed')


def outcome(data, result):
    from r5_origin_analysis import unwrap_yaw, trajectory
    start = result['schedule']['parked']['after']['sim']; end = result['schedule']['motion']['after']['sim']
    rows = data[(data['t'] >= start-1e-9) & (data['t'] <= end+1e-9)]
    xyz = np.column_stack([rows[k] for k in ('x', 'y', 'z')])
    metrics, _, _, _ = trajectory(rows['t'], xyz, unwrap_yaw(rows))
    metrics['maximum_center_excursion_gate_m'] = .01
    metrics['center_excursion_passed'] = metrics['maximum_displacement_m'] <= .01
    return metrics
