#!/usr/bin/env python3
"""Send declared saved observations through the live path, or resume a saved uplink queue."""
import argparse
import json
from pathlib import Path
import time
from survey_contract import read_json, validate_task, load_input
from survey_uplink import SurveyUplink


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', type=Path, required=True)
    parser.add_argument('--url', required=True)
    parser.add_argument('--token-file', type=Path, required=True)
    parser.add_argument('--spool', type=Path, required=True)
    parser.add_argument('--interval', type=float, default=.5)
    parser.add_argument('--timeout', type=float, default=30.)
    parser.add_argument('--drain-only', action='store_true')
    args = parser.parse_args()
    if not 0 <= args.interval <= 60 or not 0 < args.timeout <= 3600:
        parser.error('Invalid interval/timeout')
    task = validate_task(read_json(args.task))
    client = SurveyUplink(args.url, args.token_file, task, args.spool, autostart=False)
    try:
        if not args.drain_only:
            if client.count: raise ValueError('Existing queue: use --drain-only, or a new task and queue')
            frames, _, _, _ = load_input(task, args.task)
        else:
            frames = []
        import threading
        client.worker = threading.Thread(target=client.run, daemon=True); client.worker.start()
        for f in frames:
            robot = f.robot
            client.enqueue(f.points, f.stamp, f.translation, f.quaternion,
                           robot[:3] if robot is not None else None, robot[3:] if robot is not None else None)
            time.sleep(args.interval)
        deadline = time.monotonic()+args.timeout
        while client.status()['queued'] and time.monotonic() < deadline:
            time.sleep(.1)
        status = client.status()
        print(json.dumps(dict(input_kind='saved-data replay; not a new sensor acquisition', **status), ensure_ascii=False))
        if status['queued']: raise RuntimeError('Uplink incomplete; original queue retained for --drain-only')
    finally:
        client.close()


if __name__ == '__main__': main()
