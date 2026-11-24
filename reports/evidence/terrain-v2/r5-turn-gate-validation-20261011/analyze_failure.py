"""Read-only exact-step association; no missing forces are replaced by zero."""
import csv, json, math
from pathlib import Path
p = Path(__file__).resolve().parent
c = p / 'campaign'
name = '14-pure_turn-ramp-1000us-left'
f = c / name
r = json.loads((f / 'result.json').read_text())
q = r['turn_slip_probe']
states = {}
with (f / (name + '-contact-steps.csv')).open(newline='') as stream:
    for row in csv.DictReader(stream):
        step, wheel, state = int(row['iteration']), int(row['wheel']), int(row['state'])
        cells = states.setdefault(step, {})
        if wheel in cells and cells[wheel] != state:
            raise ValueError('Conflicting source states')
        cells[wheel] = state
        if state not in (0, 1):
            raise ValueError('Unknown source state')
motion = r['schedule']['motion']
lo, hi = motion['before']['iteration'], motion['after']['iteration']
path = dict(front_empty_m=0., front_present_m=0.)
first_empty = None
counts = [0, 0, 0, 0]
previous = None
origin = None
with (f / 'physics-steps.csv').open(newline='') as stream, (p / 'failure-contact-timeline.csv').open('w', newline='') as out:
    fields = ['iteration', 'sim_ns', 'motion_ns', 'dx_m', 'dy_m', 'displacement_m',
              'vx_world_m_s', 'vy_world_m_s', 'front_left_state', 'front_right_state',
              'rear_left_state', 'rear_right_state']
    writer = csv.writer(out); writer.writerow(fields)
    for row in csv.DictReader(stream):
        i = int(float(row['iteration']))
        if i < lo or i > hi:
            continue
        x, y, dt = float(row['x']), float(row['y']), float(row['dt'])
        if origin is None:
            origin = (x, y)
        if i not in states or set(states[i]) != {0, 1, 2, 3}:
            raise ValueError('Missing exact-step contact association')
        a = [states[i][w] for w in range(4)]
        dx, dy = x - origin[0], y - origin[1]
        distance = math.hypot(dx, dy)
        vx = vy = None
        if previous is not None:
            travel = math.hypot(x - previous[0], y - previous[1])
            vx, vy = (x - previous[0]) / dt, (y - previous[1]) / dt
            empty = not a[0] or not a[1]
            path['front_empty_m' if empty else 'front_present_m'] += travel
            counts = [n + (s == 0) for n, s in zip(counts, a)]
            if empty and first_empty is None:
                first_empty = dict(iteration=i, sim_ns=round(float(row['t']) * 1e9),
                                   center_displacement_m=distance)
        writer.writerow([i, round(float(row['t']) * 1e9), round((float(row['t']) - motion['before']['sim']) * 1e9),
                         dx, dy, distance, vx, vy] + a)
        previous = (x, y)
formal = r['formal']
analysis = dict(name=name, round_passed=False, primary_error=r['turn_slip_error'],
    maximum_center_displacement_m=q['trajectory']['maximum_displacement_m'],
    frozen_limit_m=.01, excess_m=q['trajectory']['maximum_displacement_m'] - .01,
    actual_turn_rad=q['trajectory']['unwrapped_turn_rad'],
    input_and_capture_passed=formal['passed'], data_complete=formal['data_complete'],
    sensors=formal['sensors'], safe_gap_s=formal['safe_gap_s'],
    motion_seconds=formal['command_window']['duration_s'],
    budget=q['allocation'], gate=q['gate'], contact_source=q['source'],
    stop_last_second_displacement_m=r['stop_drift_last_second_m'], stop_command_zero=r['stop_zero'],
    maximum_tilt_rad=r['max_tilt_rad'],
    first_front_empty_motion_step=first_empty, empty_steps_in_motion_by_wheel=counts,
    motion_path_by_source_state=path,
    source_empty_semantics='Explicit source state, not a missing sample; no force value imputed',
    causal_conclusion='Residual drift mechanism not confirmed; association only, one ramp turn, no restore completed',
    no_retest=True, no_parameter_or_threshold_change=True)
(p / 'failure-analysis.json').write_text(json.dumps(analysis, indent=2) + '\n')
print(json.dumps({k: analysis[k] for k in ('maximum_center_displacement_m', 'excess_m',
    'first_front_empty_motion_step', 'empty_steps_in_motion_by_wheel', 'motion_path_by_source_state')}))
