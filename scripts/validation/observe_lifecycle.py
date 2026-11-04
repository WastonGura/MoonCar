#!/usr/bin/env python3
"""Read-only lifecycle witness; the readiness predicate remains in the product."""
import argparse
import json
from pathlib import Path
import signal
import time

import rclpy
from lifecycle_msgs.srv import GetState


def main():
    p = argparse.ArgumentParser(); p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init(); node = rclpy.create_node('comprehensive_lifecycle_witness')
    names = ('bt_navigator', 'controller_server', 'planner_server', 'behavior_server')
    clients = {name: node.create_client(GetState, '/'+name+'/get_state') for name in names}
    pending = {}; states = {}; stopped = False; last = 0.
    def stop(*_):
        nonlocal stopped
        stopped = True
    for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, stop)
    try:
        with (args.output/'samples.jsonl').open('x') as log:
            while not stopped and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.05)
                now = time.monotonic()
                for name, future in list(pending.items()):
                    if future.done():
                        try:
                            state = future.result().current_state
                            states[name] = dict(id=state.id, label=state.label, wall=now)
                        except Exception as error: states[name] = dict(error=repr(error), wall=now)
                        del pending[name]
                if now-last < .5: continue
                last = now
                row = dict(wall=now, nodes=states,
                    all_active=all(states.get(n, {}).get('id') == 3 and now-states[n]['wall'] < 3 for n in names))
                text = json.dumps(row)+'\n'; log.write(text); log.flush()
                temp = args.output/'latest.tmp'; temp.write_text(text); temp.replace(args.output/'latest.json')
                for name, client in clients.items():
                    if name not in pending and client.service_is_ready():
                        pending[name] = client.call_async(GetState.Request())
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
