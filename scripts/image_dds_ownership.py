"""Read-only Fast DDS segment ownership, based on the creator's exclusive lock.

A remote mapping and an inherited descriptor do not confer ownership. The PID
reported by Linux for the FLOCK must match a live process with the same lock
inode open. No DDS state, lock or file is modified by this module.
"""
import os, re
from pathlib import Path

PAYLOADS = {549408: 524288, 8782368: 8388608}
LOCK = re.compile(r'FLOCK\s+ADVISORY\s+WRITE\s+(\d+)\s+([0-9a-f]+):([0-9a-f]+):(\d+)\s', re.I)

def identity(pid, proc=Path('/proc')):
    stat = (proc / str(pid) / 'stat').read_text()
    return dict(pid=int(pid), start_ticks=int(stat[stat.rindex(')')+2:].split()[19]))

def lock_rows(text):
    return [dict(pid=int(m[1]), device=os.makedev(int(m[2],16),int(m[3],16)), inode=int(m[4]))
            for m in LOCK.finditer(text)]

def lock_owners(locks, processes, device, inode):
    owners=[]
    for (pid,dev,ino),row in locks.items():
        if (dev,ino)!=(device,inode):continue
        process=processes.get(pid)
        if process and any((d['device'],d['inode'])==(dev,ino) for d in process['descriptors']):
            owners.append(dict(pid=pid,start_ticks=process['start_ticks']))
    return owners

def inventory(proc=Path('/proc'), shm=Path('/dev/shm')):
    """Collect fdinfo locks too: /proc/locks may omit locks in PID namespaces."""
    locks = {}; processes = {}; errors = []
    for directory in proc.iterdir():
        if not directory.name.isdigit(): continue
        pid=int(directory.name)
        try:
            before=identity(pid,proc); descriptors=[]
            for fd in (directory/'fd').iterdir():
                try:
                    target=os.readlink(fd)
                    if not target.startswith(str(shm)+'/fastrtps_') or not target.endswith('_el'): continue
                    stat=fd.stat(); info=(directory/'fdinfo'/fd.name).read_text()
                    descriptors.append(dict(fd=int(fd.name), path=target, device=stat.st_dev, inode=stat.st_ino,
                                            locks=lock_rows(info)))
                    for row in lock_rows(info):
                        key=(row['pid'],row['device'],row['inode'])
                        locks[key]=row
                except (FileNotFoundError,ProcessLookupError): continue
            if before!=identity(pid,proc): raise RuntimeError('PID changed during inventory')
            processes[pid]=dict(**before, descriptors=descriptors)
        except (FileNotFoundError,ProcessLookupError): continue
        except (PermissionError,RuntimeError) as error: errors.append(dict(pid=pid,error=repr(error)))
    segments=[]
    for path in sorted(shm.glob('fastrtps_*')):
        if path.name.endswith(('_el','_sl')) or path.name.startswith('fastrtps_port'): continue
        lock=Path(str(path)+'_el')
        try:
            stat=path.stat(); ls=lock.stat()
            owners=lock_owners(locks,processes,ls.st_dev,ls.st_ino)
            segments.append(dict(path=str(path),bytes=stat.st_size,payload_bytes=PAYLOADS.get(stat.st_size),
                metadata_bytes=stat.st_size-PAYLOADS[stat.st_size] if stat.st_size in PAYLOADS else None,
                device=stat.st_dev,inode=stat.st_ino,lock_path=str(lock),lock_device=ls.st_dev,lock_inode=ls.st_ino,
                owners=owners,ownership_resolved=len(owners)==1))
        except FileNotFoundError:
            segments.append(dict(path=str(path),ownership_resolved=False,error='Missing segment or exclusive lock'))
    return dict(segments=segments,processes=processes,errors=errors)

def classify(pid, role, data, proc=Path('/proc'), expected_payload=524288, require_participant=True):
    errors=[]; before=identity(pid,proc); directory=proc/str(pid)
    maps=(directory/'maps').read_text()
    paths={line.split(maxsplit=5)[-1].removesuffix(' (deleted)') for line in maps.splitlines() if '/' in line}
    libraries=sorted(p for p in paths if any(n in p for n in ('librmw_fastrtps','libfastrtps')))
    environment=(directory/'environ').read_bytes().split(b'\0')
    leaked=any(v.startswith((b'MOONCAR_IMAGE_DDS_TX_BYTES=',b'MOONCAR_IMAGE_DDS_RECEIPT=')) for v in environment)
    owned=[]; remote=[]; unresolved=[]
    for segment in data['segments']:
        if segment['path'] not in paths: continue
        if not segment['ownership_resolved']: unresolved.append(segment); continue
        (owned if segment['owners'][0]==before else remote).append(segment)
    known={s['path'] for s in data['segments']}
    for path in paths:
        if path.startswith('/dev/shm/fastrtps_') and not Path(path).name.startswith('fastrtps_port') and not path.endswith(('_el','_sl')) and path not in known:
            unresolved.append(dict(path=path,error='Mapped segment missing from live inventory'))
    if unresolved: errors.append('Mapped segment ownership unresolved')
    if require_participant and not owned: errors.append('Required participant has no owned segment')
    if require_participant and not any(p.endswith('librmw_fastrtps_cpp.so') for p in libraries): errors.append('Required ROS RMW absent')
    if role=='archive-worker' and owned: errors.append('Archive worker unexpectedly created a participant')
    if any(s['payload_bytes']!=expected_payload for s in owned): errors.append('Owned segment capacity mismatch')
    if role=='bridge':
        if not any('/opt/mooncar/image-dds/' in p and p.endswith('librmw_fastrtps_cpp.so') for p in libraries): errors.append('Minimal RMW absent')
    elif libraries:
        if not all(p.startswith('/opt/ros/jazzy/lib/') for p in libraries): errors.append('Receiver uses custom DDS library')
        if leaked: errors.append('Bridge environment leaked')
    if before!=identity(pid,proc): errors.append('PID changed during snapshot')
    return dict(**before,role=role,commandline=(directory/'cmdline').read_bytes().decode(errors='replace').split('\0'),
        libraries=libraries,owned_segments=owned,remote_mappings=remote,
        unresolved_segments=unresolved,participant=bool(owned),capacity_environment_leaked=leaked,
        verified=not errors,errors=errors)

def snapshot(pid=None, role='ros-node', data=None, expected_payload=524288, require_participant=True):
    return classify(pid or os.getpid(),role,data or inventory(),expected_payload=expected_payload,
                    require_participant=require_participant)

def registry(bridge_pid=None, registered=None):
    """Discover actual lock owners, with explicit roles for known processes.

Library-only processes are retained as nonparticipants. Success never depends
on the number of processes or on the size of their remote mappings.
"""
    data=inventory(); registered=dict(registered or {}); registered[os.getpid()]='ros-node'
    if bridge_pid: registered[int(bridge_pid)]='bridge'
    for segment in data['segments']:
        for owner in segment.get('owners',[]): registered.setdefault(owner['pid'],'ros-node')
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():continue
        try:
            if 'librmw_fastrtps_cpp.so' in (directory/'maps').read_text():
                registered.setdefault(int(directory.name),'transient')
        except (FileNotFoundError,ProcessLookupError):continue
    rows={}
    for pid,role in registered.items():
        try:
            rows[str(pid)]=snapshot(pid,role,data,8388608 if role=='bridge' else 524288,
                role not in ('archive-worker','transient','observer-native'))
        except (FileNotFoundError,ProcessLookupError) as error:
            rows[str(pid)]=dict(pid=pid,role=role,verified=False,error='Registered process exited: '+repr(error))
    return dict(passed=bool(rows and not data['errors'] and all(r['verified'] for r in rows.values())),
                processes=rows,inventory=data)
