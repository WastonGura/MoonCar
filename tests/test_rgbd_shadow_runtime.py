"""Owned Linux thread-affinity checks using small children, never Gazebo."""
from contextlib import contextmanager
import importlib.util
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import uuid

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('rgbd_shadow_runtime', ROOT/'scripts/rgbd_shadow_runtime.py')
runtime = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runtime)

CHILD = r'''
import json, os, sys, threading
if os.environ.get('RUNTIME_CHILD_SHADOW') == '1':
    os.sched_setaffinity(0, {2, 3, 4, 5})
    os.nice(int(os.environ.get('RUNTIME_CHILD_SHADOW_NICE', '19')))
elif os.environ.get('RUNTIME_CHILD_LOCALIZATION') == '1':
    os.sched_setaffinity(0, {0, 1})
stop = threading.Event()
workers = []
def wait():
    stop.wait()
def spawn():
    thread = threading.Thread(target=wait)
    thread.start()
    workers.append(thread)
    return thread.native_id
spawn(); spawn()
print(json.dumps(dict(ready=True, pid=os.getpid())), flush=True)
for line in sys.stdin:
    if line.strip() == 'spawn':
        tid = spawn()
        print(json.dumps(dict(tid=tid, cpus=sorted(os.sched_getaffinity(tid)))), flush=True)
    elif line.strip() == 'escape':
        os.sched_setaffinity(0, {0, 2})
        print(json.dumps(dict(tid=os.getpid(), cpus=sorted(os.sched_getaffinity(0)))), flush=True)
    else:
        break
stop.set()
for thread in workers:
    thread.join()
'''


def response(child):
    if not select.select([child.stdout], [], [], 5.)[0]:
        raise AssertionError('Owned dummy child did not answer within 5 wall seconds')
    line = child.stdout.readline()
    if not line:
        raise AssertionError('Owned dummy exited before replying')
    return json.loads(line)


@contextmanager
def dummy(experiment, marker, background=False, localization=False, shadow_nice=19):
    child = subprocess.Popen([sys.executable, '-u', '-c', CHILD, marker],
        env=dict(os.environ, GZ_PARTITION='mooncar-'+experiment,
                 RUNTIME_CHILD_SHADOW='1' if background else '0',
                 RUNTIME_CHILD_SHADOW_NICE=str(shadow_nice),
                 RUNTIME_CHILD_LOCALIZATION='1' if localization else '0'),
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert response(child)['ready']
        yield child
    finally:
        if child.poll() is None:
            try:
                child.stdin.write('exit\n'); child.stdin.flush()
                child.wait(timeout=5.)
            except (BrokenPipeError, subprocess.TimeoutExpired):
                child.kill(); child.wait(timeout=5.)
        child.stdin.close(); child.stdout.close(); child.stderr.close()


@unittest.skipUnless(sys.platform == 'linux', 'Linux /proc and sched affinity contract')
class ShadowRuntimeTests(unittest.TestCase):
    def test_wrong_process_start_identity_cannot_write_affinity(self):
        expected = dict(pid=os.getpid(), start_ticks='wrong', argv=['owned'])
        with patch.object(runtime.os, 'sched_setaffinity') as change:
            with self.assertRaisesRegex(RuntimeError, 'identity changed'):
                runtime.set_threads(expected, {0, 1})
            change.assert_not_called()

    def test_unexpected_initial_affinity_cannot_write(self):
        process = dict(pid=123, start_ticks='456', argv=['owned'])
        with patch.object(runtime, 'threads', return_value=[dict(tid=123, cpus=[0, 1], nice=0)]), \
                patch.object(runtime.os, 'sched_setaffinity') as change:
            with self.assertRaisesRegex(RuntimeError, 'initial six-CPU'):
                runtime.set_threads(process, {2, 3, 4, 5})
            change.assert_not_called()
        # Global apply must also validate every assignment before the first
        # write, including a bad second process or a mis-launched shadow.
        local, simulation, shadow = [dict(pid=i, start_ticks='x', argv=['owned']) for i in (123, 124, 125)]
        for bad in ('localization', 'simulation', 'shadow'):
            with self.subTest(bad=bad), tempfile.TemporaryDirectory() as folder:
                def rows(p):
                    cpus, nice = (([2, 3, 4, 5], 19) if p is shadow else
                                  ([0, 1], 0) if p is local else (list(range(6)), 0))
                    if (bad == 'simulation' and p is simulation) or (bad == 'shadow' and p is shadow):
                        cpus = [0, 1]
                    if bad == 'localization' and p is local:
                        cpus = list(range(6))
                    return [dict(tid=p['pid'], cpus=cpus, nice=nice)]
                with patch.object(runtime.os, 'sched_getaffinity', return_value=set(range(6))), \
                        patch.object(runtime.Path, 'read_text', return_value='600000 100000\n'), \
                        patch.object(runtime, 'owned_process', side_effect=[local, simulation, shadow]), \
                        patch.object(runtime, 'threads', side_effect=rows), \
                        patch.object(runtime.os, 'sched_setaffinity') as change:
                    with self.assertRaises(RuntimeError):
                        runtime.apply('owned', Path(folder)/'runtime.json')
                    change.assert_not_called()
                    self.assertFalse((Path(folder)/'runtime.json').exists())

    def test_thread_escaping_assignment_fails_the_runtime_check(self):
        process = dict(pid=123, start_ticks='456', argv=['owned'])
        record = dict(localization=dict(process=process, cpus=[0, 1]),
                      gazebo=dict(process=process, cpus=[2, 3, 4, 5]),
                      shadow=dict(process=process, cpus=[2, 3, 4, 5]), shadow_nice=19)
        with patch.object(runtime, 'threads', return_value=[dict(tid=123, cpus=list(range(6)), nice=0)]):
            with self.assertRaisesRegex(RuntimeError, 'escaped'):
                runtime.check(record)

    def test_narrower_nonempty_thread_assignments_are_legal(self):
        record = {key: dict(process=dict(pid=pid, start_ticks='456', argv=['owned']), cpus=cpus)
                  for key, pid, cpus in [('localization', 123, [0, 1]),
                    ('gazebo', 124, [2, 3, 4, 5]), ('shadow', 125, [2, 3, 4, 5])]}
        record['shadow_nice'] = 19
        def rows(process):
            key = next(k for k, v in record.items() if v['process'] is process)
            return [dict(tid=process['pid'], cpus=[record[key]['cpus'][0]],
                         nice=19 if key == 'shadow' else 0)]
        with patch.object(runtime, 'threads', side_effect=rows):
            runtime.check(record)
        with patch.object(runtime, 'threads', return_value=[dict(tid=123, cpus=[], nice=0)]):
            with self.assertRaisesRegex(RuntimeError, 'escaped'):
                runtime.check(record)

    def test_default_priority_change_is_not_silently_accepted(self):
        record = {key: dict(process=dict(pid=pid, start_ticks='456', argv=['owned']), cpus=cpus)
                  for key, pid, cpus in [('localization', 123, [0, 1]),
                    ('gazebo', 124, [2, 3, 4, 5]), ('shadow', 125, [2, 3, 4, 5])]}
        record['shadow_nice'] = 19
        for bad in ('localization', 'shadow'):
            with self.subTest(bad=bad):
                def rows(process):
                    key = next(k for k, v in record.items() if v['process'] is process)
                    nice = 19 if key == 'shadow' else 0
                    if key == bad:
                        nice = 0 if key == 'shadow' else 19
                    return [dict(tid=process['pid'], cpus=record[key]['cpus'], nice=nice)]
                with patch.object(runtime, 'threads', side_effect=rows):
                    with self.assertRaisesRegex(RuntimeError, 'priority changed'):
                        runtime.check(record)

    def test_real_owned_children_all_threads_and_later_threads_keep_assignment(self):
        experiment, marker = 'runtime-test-'+uuid.uuid4().hex, uuid.uuid4().hex
        with dummy(experiment, marker+'-localization', localization=True) as localization, \
                dummy(experiment, marker+'-simulation') as simulation, \
                dummy(experiment, marker+'-shadow', background=True) as shadow, \
                tempfile.TemporaryDirectory() as folder:
            local_process = runtime.owned_process(experiment, lambda argv: argv[-1:] == [marker+'-localization'])
            sim_process = runtime.owned_process(experiment, lambda argv: argv[-1:] == [marker+'-simulation'])
            shadow_process = runtime.owned_process(experiment, lambda argv: argv[-1:] == [marker+'-shadow'])
            self.assertEqual(local_process['pid'], localization.pid)
            self.assertEqual(sim_process['pid'], simulation.pid)
            local_tids = {row['tid'] for row in runtime.threads(local_process)}
            shadow_tids = {row['tid'] for row in runtime.threads(shadow_process)}
            with patch.object(runtime, 'owned_process', side_effect=[local_process, sim_process, shadow_process]), \
                    patch.object(runtime.os, 'sched_setaffinity', wraps=os.sched_setaffinity) as change:
                record = runtime.apply(experiment, Path(folder)/'runtime.json')
                self.assertEqual(record['shadow_nice'], 19)
                self.assertTrue(change.called)
                self.assertTrue(all(call.args[0] not in local_tids | shadow_tids for call in change.call_args_list))
            local_record, sim_record = record['localization'], record['gazebo']
            self.assertTrue(local_record['applied_at_process_start'])
            self.assertGreaterEqual(len(local_record['after']), 3)
            self.assertTrue(all(t['cpus'] == [0, 1] for t in local_record['after']))
            self.assertTrue(all(t['cpus'] == [2, 3, 4, 5] for t in sim_record['after']))
            runtime.check(record)
            for child, expected in ((localization, [0, 1]), (simulation, [2, 3, 4, 5]), (shadow, [2, 3, 4, 5])):
                child.stdin.write('spawn\n'); child.stdin.flush()
                born = response(child)
                self.assertEqual(born['cpus'], expected)
            runtime.check(record)
            # A real owned thread escapes after successful preflight. The
            # monitor must retain the actual masks, rather than only a label.
            localization.stdin.write('escape\n'); localization.stdin.flush()
            escaped = response(localization)
            self.assertEqual(escaped, dict(tid=localization.pid, cpus=[0, 2]))
            with self.assertRaisesRegex(RuntimeError, 'localization escaped'):
                runtime.check(record)
            violations = [json.loads(line) for line in Path(record['monitor_path']).read_text().splitlines()]
            self.assertEqual(len(violations), 1)
            violation = violations[0]
            self.assertEqual(violation['kind'], 'localization')
            self.assertEqual(violation['process'], local_process)
            self.assertEqual(violation['allowed_cpus'], [0, 1])
            self.assertTrue(any(row['tid'] == localization.pid and row['cpus'] == [0, 2]
                                for row in violation['actual_threads']))
            self.assertIsInstance(violation['wall'], (int, float))
        self.assertEqual(localization.returncode, 0)
        self.assertEqual(simulation.returncode, 0)
        self.assertEqual(shadow.returncode, 0)

    def test_explicit_nice5_uses_real_owned_threads_without_changing_other_priorities(self):
        experiment, marker = 'runtime-test-'+uuid.uuid4().hex, uuid.uuid4().hex
        with dummy(experiment, marker+'-localization', localization=True) as localization, \
                dummy(experiment, marker+'-simulation') as simulation, \
                dummy(experiment, marker+'-shadow', background=True, shadow_nice=5) as shadow, \
                tempfile.TemporaryDirectory() as folder:
            processes = [runtime.owned_process(experiment, lambda argv, suffix=suffix: argv[-1:] == [marker+suffix])
                         for suffix in ('-localization', '-simulation', '-shadow')]
            with patch.object(runtime, 'owned_process', side_effect=processes):
                record = runtime.apply(experiment, Path(folder)/'runtime.json', shadow_nice=5)
            self.assertEqual(record['shadow_nice'], 5)
            self.assertEqual(json.loads((Path(folder)/'runtime.json').read_text())['shadow_nice'], 5)
            runtime.check(record)
            for key, expected_cpus, expected_nice in (('localization', [0, 1], 0),
                    ('gazebo', [2, 3, 4, 5], 0), ('shadow', [2, 3, 4, 5], 5)):
                self.assertTrue(all(row['cpus'] == expected_cpus and row['nice'] == expected_nice
                                    for row in runtime.threads(record[key]['process'])))
            shadow.stdin.write('spawn\n'); shadow.stdin.flush()
            born = response(shadow)
            self.assertEqual(born['cpus'], [2, 3, 4, 5])
            self.assertEqual(os.getpriority(os.PRIO_PROCESS, born['tid']), 5)
            runtime.check(record)
            with self.assertRaisesRegex(RuntimeError, 'shadow priority changed'):
                runtime.check(dict(record, shadow_nice=19))
        self.assertEqual([localization.returncode, simulation.returncode, shadow.returncode], [0, 0, 0])

    def test_invalid_shadow_priority_is_rejected_before_any_process_or_affinity_write(self):
        for value in (0, 4, 6, 18, 20, None, True, 5., '5'):
            with self.subTest(value=value), tempfile.TemporaryDirectory() as folder, \
                    patch.object(runtime, 'owned_process') as owner, \
                    patch.object(runtime, 'threads') as rows, \
                    patch.object(runtime.os, 'sched_setaffinity') as change:
                path = Path(folder)/'runtime.json'
                with self.assertRaisesRegex(ValueError, '5 or 19'):
                    runtime.apply('owned', path, shadow_nice=value)
                with self.assertRaisesRegex(RuntimeError, 'Invalid frozen shadow priority'):
                    runtime.check(dict(shadow_nice=value))
                self.assertFalse(path.exists())
                owner.assert_not_called(); rows.assert_not_called(); change.assert_not_called()

    def test_mismatched_actual_shadow_priority_fails_preflight_without_affinity_writes(self):
        local, simulation, shadow = [dict(pid=i, start_ticks='x', argv=['owned']) for i in (123, 124, 125)]
        for expected, actual in ((5, 19), (19, 5)):
            def rows(process):
                cpus, nice = (([2, 3, 4, 5], actual) if process is shadow else
                              ([0, 1], 0) if process is local else (list(range(6)), 0))
                return [dict(tid=process['pid'], cpus=cpus, nice=nice)]
            with self.subTest(expected=expected), tempfile.TemporaryDirectory() as folder, \
                    patch.object(runtime.os, 'sched_getaffinity', return_value=set(range(6))), \
                    patch.object(runtime.Path, 'read_text', return_value='600000 100000\n'), \
                    patch.object(runtime, 'owned_process', side_effect=[local, simulation, shadow]), \
                    patch.object(runtime, 'threads', side_effect=rows), \
                    patch.object(runtime.os, 'sched_setaffinity') as change:
                path = Path(folder)/'runtime.json'
                with self.assertRaisesRegex(RuntimeError, 'frozen background profile'):
                    if expected == 19:
                        runtime.apply('owned', path)  # Original callers still require 19.
                    else:
                        runtime.apply('owned', path, shadow_nice=expected)
                self.assertFalse(path.exists())
                change.assert_not_called()

    def test_wrong_ownership_and_duplicate_owned_processes_are_rejected(self):
        experiment, marker = 'runtime-test-'+uuid.uuid4().hex, uuid.uuid4().hex
        predicate = lambda argv: argv[-1:] == [marker]
        with dummy(experiment, marker), patch.object(runtime.os, 'sched_setaffinity') as change:
            with self.assertRaisesRegex(RuntimeError, 'exactly one owned'):
                runtime.owned_process(experiment+'-wrong', predicate)
            with dummy(experiment, marker):
                with self.assertRaisesRegex(RuntimeError, 'exactly one owned'):
                    runtime.owned_process(experiment, predicate)
            change.assert_not_called()

    def test_quota_preflight_fails_without_writing_affinity_or_receipt(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'runtime.json'
            with patch.object(runtime.os, 'sched_getaffinity', return_value=set(range(6))), \
                    patch.object(runtime.Path, 'read_text', return_value='800000 100000\n'), \
                    patch.object(runtime.os, 'sched_setaffinity') as change, \
                    patch.object(runtime, 'owned_process') as owner:
                with self.assertRaisesRegex(RuntimeError, 'quota changed'):
                    runtime.apply('owned', path)
                self.assertFalse(path.exists())
                change.assert_not_called()
                owner.assert_not_called()


if __name__ == '__main__':
    unittest.main()
