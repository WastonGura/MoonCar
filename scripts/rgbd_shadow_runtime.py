"""Bound background simulation work away from the default RGBD worker.

Explicit test-runtime helper, not a localization algorithm or quality policy.
The caller must own the exact live workbench experiment. Linux only; affinity
changes apply to its existing threads and are inherited by new threads.
"""
import json
import os
from pathlib import Path
import time


def identity(pid):
    directory = Path('/proc') / str(pid)
    stat = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
    return dict(pid=pid, start_ticks=stat[19],
                argv=[x.decode() for x in (directory / 'cmdline').read_bytes().split(b'\0') if x])


def owned_process(experiment, predicate):
    expected = ('mooncar-' + experiment).encode()
    matches = []
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():
            continue
        try:
            current = identity(int(directory.name))
            if not predicate(current['argv']):
                continue
            env = dict(x.split(b'=', 1) for x in (directory / 'environ').read_bytes().split(b'\0') if b'=' in x)
            if env.get(b'GZ_PARTITION') == expected:
                matches.append(current)
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    if len(matches) != 1:
        raise RuntimeError('Expected exactly one owned process: ' + repr(matches))
    return matches[0]


def threads(process):
    if identity(process['pid']) != process:
        raise RuntimeError('Owned process identity changed')
    values = []
    for directory in (Path('/proc') / str(process['pid']) / 'task').iterdir():
        try:
            tid = int(directory.name)
            values.append(dict(tid=tid, cpus=sorted(os.sched_getaffinity(tid)),
                               nice=os.getpriority(os.PRIO_PROCESS, tid)))
        except (FileNotFoundError, ProcessLookupError):
            continue
    if not values:
        raise RuntimeError('Owned process has no threads')
    return sorted(values, key=lambda row: row['tid'])


def set_threads(process, cpus):
    before = threads(process)
    if any(row['cpus'] != list(range(6)) for row in before):
        raise RuntimeError('Unexpected initial six-CPU affinity')
    # Recheck process identity before every pass; a new thread inherits affinity
    # from its creator. Include threads born during the preceding pass as well.
    for _ in range(4):
        current = threads(process)
        for row in current:
            try:
                os.sched_setaffinity(row['tid'], cpus)
            except ProcessLookupError:
                pass
        after = threads(process)
        if all(row['cpus'] == sorted(cpus) for row in after):
            return dict(process=process, before=before, after=after, cpus=sorted(cpus))
    raise RuntimeError('Owned thread affinity did not settle')


def check(record):
    shadow_nice = record['shadow_nice']
    if type(shadow_nice) is not int or shadow_nice not in (5, 19):
        raise RuntimeError('Invalid frozen shadow priority')
    for key in ('localization', 'gazebo', 'shadow'):
        item = record[key]
        current = threads(item['process'])
        if any(not row['cpus'] or not set(row['cpus']).issubset(item['cpus']) for row in current):
            detail=dict(process=item['process'],allowed_cpus=item['cpus'],actual_threads=current)
            if record.get('monitor_path'):
                with Path(record['monitor_path']).open('a') as stream:
                    stream.write(json.dumps(dict(kind=key,wall=time.monotonic(),**detail))+'\n')
            raise RuntimeError(key + ' escaped the frozen CPU assignment: '+repr(detail))
        if key == 'localization' and any(row['nice'] != 0 for row in current):
            raise RuntimeError('Default localization priority changed')
        if key == 'shadow' and any(row['nice'] != shadow_nice for row in current):
            raise RuntimeError('Background shadow priority changed')


def apply(experiment, receipt, shadow_nice=19):
    if type(shadow_nice) is not int or shadow_nice not in (5, 19):
        raise ValueError('shadow_nice must be 5 or 19')
    receipt = Path(receipt)
    if receipt.exists():
        raise FileExistsError(receipt)
    if sorted(os.sched_getaffinity(0)) != list(range(6)):
        raise RuntimeError('This bounded profile requires the existing six CPUs')
    if Path('/sys/fs/cgroup/cpu.max').read_text().strip() != '600000 100000':
        raise RuntimeError('Total six-CPU quota changed')
    localization = owned_process(experiment, lambda args: any(
        Path(arg).name == 'rgbd_odometry_node.py' for arg in args))
    gazebo = owned_process(experiment, lambda args:
        (len(args) == 1 and args[0].startswith('gz sim ')) or
        (len(args) > 1 and Path(args[0]).name == 'gz' and args[1] == 'sim'))
    shadow = owned_process(experiment, lambda args: any(
        Path(arg).name == 'rgbd_window_shadow.py' for arg in args))
    if any(not row['cpus'] or not set(row['cpus']).issubset({0,1}) for row in threads(localization)):
        raise RuntimeError('Default RGBD must start within CPUs 0-1 before native libraries initialize')
    for process in (gazebo,):
        if any(row['cpus'] != list(range(6)) for row in threads(process)):
            raise RuntimeError('Unexpected initial six-CPU affinity')
    shadow_threads = threads(shadow)
    if any(not row['cpus'] or not set(row['cpus']).issubset({2,3,4,5}) or row['nice'] != shadow_nice for row in shadow_threads):
        raise RuntimeError('Shadow must start with the frozen background profile')
    if any(row['nice'] != 0 for row in threads(localization)):
        raise RuntimeError('Expected original default localization priority')
    record = dict(experiment=experiment, applied_wall=time.monotonic(),
        total_cpu_quota='600000 100000',
        monitor_path=str(receipt.with_name('runtime-affinity-violations.jsonl')),
        localization=dict(process=localization,cpus=[0,1],after=threads(localization),applied_at_process_start=True),
        gazebo=set_threads(gazebo, {2, 3, 4, 5}),
        shadow=dict(process=shadow, cpus=[2,3,4,5], after=shadow_threads),
        shadow_cpus=[2, 3, 4, 5], shadow_nice=shadow_nice,
        quality_and_geometry_unchanged=True,
        scope='Owned rendering and passive shadow restricted to CPUs 2-5; default RGBD uses CPUs 0-1. Other ROS/evidence retain their original affinity.')
    check(record)
    with receipt.open('x') as stream:
        json.dump(record, stream, indent=2); stream.write('\n')
    return record
