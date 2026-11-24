"""Read-only Fast DDS segment ownership, based on the creator's exclusive lock.

A remote mapping and an inherited descriptor do not confer ownership. The PID
reported by Linux for the FLOCK must match a live process with the same lock
inode open. No DDS state, lock or file is modified by this module.
"""
import copy, json, os, queue, re, threading, time, traceback
from pathlib import Path

PAYLOADS = {549408: 524288, 8782368: 8388608}
LOCK = re.compile(r'FLOCK\s+ADVISORY\s+WRITE\s+(\d+)\s+([0-9a-f]+):([0-9a-f]+):(\d+)\s', re.I)

def identity(pid, proc=Path('/proc')):
    stat = (proc / str(pid) / 'stat').read_text()
    return dict(pid=int(pid), start_ticks=int(stat[stat.rindex(')')+2:].split()[19]))

def stat_context(pid, proc=Path('/proc')):
    """One stat read, retaining state without changing identity semantics."""
    text=(proc/str(pid)/'stat').read_text();fields=text[text.rindex(')')+2:].split()
    return dict(identity=dict(pid=int(pid),start_ticks=int(fields[19])),
                name=text[text.index('(')+1:text.rindex(')')],state=fields[0],ppid=int(fields[1]))

def error_context(pid, proc=Path('/proc')):
    """Best-effort evidence; an unreadable context never clears the error."""
    directory=proc/str(pid);result={}
    try:result.update(stat_context(pid,proc))
    except (OSError,ValueError,IndexError) as error:result['stat_error']=repr(error)
    try:
        result['status']='\n'.join(row for row in (directory/'status').read_text().splitlines()
            if row.split(':')[0] in ('State','PPid','Uid','Gid','CapPrm','CapEff','TracerPid','CoreDumping','NoNewPrivs'))
    except OSError as error:result['status_error']=repr(error)
    try:result['commandline']=(directory/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
    except OSError as error:result['commandline_error']=repr(error)
    return result

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

def mapped_files(text):
    rows={}
    for line in text.splitlines():
        fields=line.split(maxsplit=5)
        if len(fields)!=6 or not fields[5].startswith('/'):continue
        major,minor=fields[3].split(':');path=fields[5].removesuffix(' (deleted)')
        rows[path]=dict(path=path,device=os.makedev(int(major,16),int(minor,16)),inode=int(fields[4]),
                        deleted=fields[5].endswith(' (deleted)'))
    return rows

def inventory(proc=Path('/proc'), shm=Path('/dev/shm')):
    """Collect fdinfo locks too: /proc/locks may omit locks in PID namespaces."""
    locks = {}; processes = {}; errors = []
    for directory in proc.iterdir():
        if not directory.name.isdigit(): continue
        pid=int(directory.name)
        before_context=None;operation='read_identity_before';path=directory/'stat'
        try:
            before_context=stat_context(pid,proc);before=before_context['identity'];descriptors=[]
            operation='list_fds';path=directory/'fd'
            entries=iter((directory/'fd').iterdir())
            while True:
                operation='list_fds';path=directory/'fd'
                try:fd=next(entries)
                except StopIteration:break
                try:
                    operation='readlink';path=fd
                    target=os.readlink(fd)
                    if not target.startswith(str(shm)+'/fastrtps_') or not target.endswith('_el'): continue
                    operation='stat_lock_fd';path=fd;stat=fd.stat()
                    operation='read_fdinfo';path=directory/'fdinfo'/fd.name;info=path.read_text()
                    descriptors.append(dict(fd=int(fd.name), path=target, device=stat.st_dev, inode=stat.st_ino,
                                            locks=lock_rows(info)))
                    for row in lock_rows(info):
                        key=(row['pid'],row['device'],row['inode'])
                        locks[key]=row
                except (FileNotFoundError,ProcessLookupError): continue
            operation='read_identity_after';path=directory/'stat'
            if before!=identity(pid,proc): raise RuntimeError('PID changed during inventory')
            processes[pid]=dict(**before, descriptors=descriptors)
        except (FileNotFoundError,ProcessLookupError): continue
        except (PermissionError,RuntimeError) as error:
            errors.append(dict(pid=pid,error=repr(error),operation=operation,path=str(path),
                errno=getattr(error,'errno',None),filename=getattr(error,'filename',None),
                before=before_context,after=error_context(pid,proc),wall=time.monotonic(),
                traceback=traceback.format_exc()))
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

def classify(pid, role, data, proc=Path('/proc'), expected_payload=524288, require_participant=True, history=None):
    errors=[]; before=identity(pid,proc); directory=proc/str(pid)
    maps=(directory/'maps').read_text()
    mappings=mapped_files(maps);paths=set(mappings)
    libraries=sorted(p for p in paths if any(n in p for n in ('librmw_fastrtps','libfastrtps')))
    environment=(directory/'environ').read_bytes().split(b'\0')
    leaked=any(v.startswith((b'MOONCAR_IMAGE_DDS_TX_BYTES=',b'MOONCAR_IMAGE_DDS_RECEIPT=')) for v in environment)
    owned=[]; remote=[]; unresolved=[];retired=[]
    for segment in data['segments']:
        if segment['path'] not in paths: continue
        mapping=mappings[segment['path']]
        if not segment['ownership_resolved'] or (mapping['device'],mapping['inode'])!=(segment.get('device',mapping['device']),segment.get('inode',mapping['inode'])):
            unresolved.append(segment); continue
        (owned if segment['owners'][0]==before else remote).append(segment)
    known={s['path'] for s in data['segments']}
    for path in paths:
        if path.startswith('/dev/shm/fastrtps_') and not Path(path).name.startswith('fastrtps_port') and not path.endswith(('_el','_sl')) and path not in known:
            mapping=mappings[path]
            proof=history.retired(mapping,proc) if history else None
            if proof:retired.append(proof)
            else:unresolved.append(dict(**mapping,error='Mapped segment missing from live inventory'))
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
        retired_remote_mappings=retired,
        unresolved_segments=unresolved,participant=bool(owned),capacity_environment_leaked=leaked,
        verified=not errors,errors=errors)

def snapshot(pid=None, role='ros-node', data=None, expected_payload=524288, require_participant=True, history=None):
    return classify(pid or os.getpid(),role,data or inventory(),expected_payload=expected_payload,
                    require_participant=require_participant,history=history)

def registry(bridge_pid=None, registered=None, history=None):
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
                role not in ('archive-worker','transient','observer-native'),history)
        except (FileNotFoundError,ProcessLookupError) as error:
            rows[str(pid)]=dict(pid=pid,role=role,verified=False,error='Registered process exited: '+repr(error))
    return dict(passed=bool(rows and not data['errors'] and all(r['verified'] for r in rows.values())),
                processes=rows,inventory=data)

class Lifecycle:
    """Observe creator locks before transient participants exit; never infer them.

Only an exact kernel mapping inode previously seen with one exclusive owner
may be labelled retired, and only after both unlink and that owner PID's exit
have been independently confirmed. Unknown mappings remain failures.
"""
    def __init__(self,path,period=.02):
        self.path=Path(path);self.period=period;self.history={};self.lock=threading.Lock()
        self.queue=queue.Queue(8192);self.stop=threading.Event();self.errors=[];self.scans=0;self.maximum_scan_s=0.
        self.writer=threading.Thread(target=self.write,name='dds-lifecycle-writer',daemon=True)
        self.reader=threading.Thread(target=self.observe,name='dds-lifecycle-observer',daemon=True)
        self.writer.start();self.reader.start()
    def remember(self,data,wall):
        for segment in data['segments']:
            if not segment.get('ownership_resolved'):continue
            key=(segment['device'],segment['inode'])
            with self.lock:
                existing=self.history.get(key)
                if existing:
                    if existing['segment']['owners']!=segment['owners'] or existing['segment']['path']!=segment['path']:
                        self.errors.append('Conflicting ownership for kernel segment identity')
                    continue
                owner=segment['owners'][0]
                try:command=(Path('/proc')/str(owner['pid'])/'cmdline').read_bytes().decode(errors='replace').split('\0')
                except FileNotFoundError:command=None
                row=dict(event='created_owner',first_seen_wall=wall,segment=copy.deepcopy(segment),owner_commandline=command,
                         owner_lock_descriptors=copy.deepcopy(data['processes'].get(owner['pid'],{}).get('descriptors',[])))
                self.history[key]=row
            self.emit(row)
    def emit(self,row):
        try:self.queue.put_nowait(row)
        except queue.Full:self.errors.append('Lifecycle observation queue overflow');self.stop.set()
    def observe(self):
        try:
            while not self.stop.is_set():
                start=time.monotonic();data=inventory();self.remember(data,start);self.scans+=1
                if data['errors']:self.errors.extend(data['errors']);self.stop.set()
                elapsed=time.monotonic()-start;self.maximum_scan_s=max(self.maximum_scan_s,elapsed)
                self.stop.wait(max(.001,self.period-elapsed))
        except BaseException as error:self.errors.append('Lifecycle observer failed: '+repr(error));self.stop.set()
    def retired(self,mapping,proc=Path('/proc')):
        if not mapping['deleted'] or Path(mapping['path']).exists():return None
        with self.lock:row=copy.deepcopy(self.history.get((mapping['device'],mapping['inode'])))
        if not row or row['segment']['path']!=mapping['path']:return None
        owner=row['segment']['owners'][0]
        try:
            if identity(owner['pid'],proc)==owner:return None
        except (FileNotFoundError,ProcessLookupError):pass
        proof=dict(row,event='retired_remote_mapping',mapping=mapping,owner_exit_confirmed_wall=time.monotonic(),
                   participant_active=False)
        self.emit(proof);return proof
    def write(self):
        try:
            with self.path.open('x') as stream:
                while True:
                    row=self.queue.get()
                    if row is None:break
                    stream.write(json.dumps(row)+'\n')
                stream.flush()
        except BaseException as error:self.errors.append('Lifecycle writer failed: '+repr(error));self.stop.set()
    def quiesce(self):
        self.stop.set();self.reader.join(5)
        if self.reader.is_alive():self.errors.append('Lifecycle reader did not stop')
    def close(self):
        self.quiesce()
        try:self.queue.put(None,timeout=5)
        except queue.Full:self.errors.append('Lifecycle queue did not drain')
        self.writer.join(5)
        if self.writer.is_alive():self.errors.append('Lifecycle writer did not drain')
        return dict(passed=not self.errors,errors=self.errors,scans=self.scans,maximum_scan_s=self.maximum_scan_s,
                    known_segments=len(self.history),drained=not self.writer.is_alive())
