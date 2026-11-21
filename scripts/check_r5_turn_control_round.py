#!/usr/bin/env python3
"""Fresh isolated drive-only hypothesis, reusing qualified recording/ownership."""
import argparse
import json
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True); parser.add_argument('--name', required=True)
    parser.add_argument('--profile', choices=('baseline', 'load_dither'), required=True)
    parser.add_argument('--angular', type=float, required=True)
    args = parser.parse_args()
    if abs(args.angular) != .2:
        parser.error('This frozen probe only permits +/-0.2 rad/s')
    import r5_step_schedule
    import image_dds_motion
    from r5_turn_control_probe import dither, command_window, outcome
    original_heartbeat = r5_step_schedule.PausedSteps.heartbeat
    original_advance = r5_step_schedule.PausedSteps.advance
    original_window = image_dds_motion.physical_command_window

    def advance(schedule, label, seconds):
        if label == 'motion':
            schedule.turn_probe_start = schedule.wait_state()['sim']
        return original_advance(schedule, label, seconds)

    def heartbeat(schedule):
        try:
            while not schedule.stop.is_set():
                with schedule.lock:
                    values = list(schedule.target)
                elapsed = None
                if values[5] and hasattr(schedule, 'turn_probe_start'):
                    elapsed = schedule.node.now()-schedule.turn_probe_start
                    values[0] = dither(elapsed, values[5])
                message = schedule.Twist(); message.linear.x = values[0]; message.angular.z = values[5]
                schedule.node.publisher.publish(message)
                schedule.node.record('source_command', values=values, probe_elapsed_s=elapsed,
                                     control_profile='load_dither')
                schedule.stop.wait(.02)
        except Exception as error:
            schedule.failure = error

    if args.profile == 'load_dither':
        r5_step_schedule.PausedSteps.heartbeat = heartbeat
        r5_step_schedule.PausedSteps.advance = advance
        image_dds_motion.physical_command_window = command_window
    try:
        import check_r5_origin_round
        sys.argv = ['check_r5_origin_round', '--output', args.output, '--name', args.name,
                    '--model', '/evidence/r5-d0.sdf', '--step', '.001', '--linear', '0', '--angular', str(args.angular)]
        code = check_r5_origin_round.main()
    finally:
        r5_step_schedule.PausedSteps.heartbeat = original_heartbeat
        r5_step_schedule.PausedSteps.advance = original_advance
        image_dds_motion.physical_command_window = original_window
    folder = Path(args.output)/args.name
    result = json.loads((folder/'result.json').read_text())
    result['turn_control_profile'] = args.profile
    if result.get('passed'):
        import numpy as np
        from r5_csv import read_numeric_csv
        data = read_numeric_csv(folder/'physics-steps.csv', delimiter=',', names=True)
        result['turn_control_outcome'] = outcome(data, result)
        rows = [json.loads(r) for r in (folder/'observations.jsonl').read_text().splitlines()]
        commands = {round(r['values'][0], 12) for r in rows if r['kind'] == 'source_command'}
        known = all(round(float(v), 12) in commands for v in data['input_lx'])
        result['published_native_values_match'] = known
        if not known:
            result.update(passed=False, turn_control_error='Native longitudinal input not present in published source commands')
            code = 1
    (folder/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('passed', 'turn_control_profile', 'turn_control_outcome') if k in result}), flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
