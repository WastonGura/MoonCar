"""Bounded test-only archival workers. No ROS or physics writes."""
import hashlib
from collections.abc import Mapping
import json
import multiprocessing as mp
from pathlib import Path
import queue
import threading
import time

IMAGE_ITEMS=8
IMAGE_BYTES=128*1024*1024
EVENT_ITEMS=8192

def dds_info(info):
    # Jazzy's executor passes the pybind take_message metadata as a dict.
    value=lambda name:info[name] if isinstance(info,Mapping) else getattr(info,name)
    return dict(dds_source_timestamp_ns=int(value('source_timestamp')),
        dds_received_timestamp_ns=int(value('received_timestamp')))

def capture_config(profile,image_trace='off',image_sampling='legacy',image_observer_sha256=None,**delivery):
    if profile not in ('legacy','async'):raise ValueError('Unknown capture profile')
    config=dict(profile=profile,image_items=IMAGE_ITEMS,image_bytes=IMAGE_BYTES,event_items=EVENT_ITEMS,
        rgb_period_s=1.,depth_period_s=1.,scan_period_s=.5,subscription_qos='unchanged')
    if image_trace!='off' or image_sampling!='legacy':
        config.update(image_trace=image_trace,image_sampling=image_sampling,image_observer_sha256=image_observer_sha256)
    if delivery:config.update(delivery)
    return config

def capture_hash(profile,**options):
    return hashlib.sha256(json.dumps(capture_config(profile,**options),sort_keys=True,separators=(',',':')).encode()).hexdigest()

def image_options(args):
    from r5_image_delivery import delivery_options
    from r5_dds_shm import runtime_options
    binary=getattr(args,'image_observer',None)
    return dict(image_trace=getattr(args,'image_trace','off'),image_sampling=getattr(args,'image_sampling','legacy'),
        image_observer_sha256=hashlib.sha256(Path(binary).read_bytes()).hexdigest() if binary else None,**delivery_options(args),**runtime_options(args))

def image_argv(args):
    result=['--image-trace',getattr(args,'image_trace','off'),'--image-sampling',getattr(args,'image_sampling','legacy')]
    if getattr(args,'image_observer',None):result+=['--image-observer',args.image_observer]
    for name in ('image_bridge','image_delivery_profile','traced_bridge','cpp_image_observer','dds_config','dds_runtime','dds_trace','dds_prefix','dds_candidate_evidence'):
        if getattr(args,name,None):result+=['--'+name.replace('_','-'),getattr(args,name)]
    return result

def image_receive_info(info):
    row=dds_info(info)
    value=lambda n:info.get(n) if isinstance(info,Mapping) else getattr(info,n,None)
    row['receiver_delay_ns']=time.time_ns()-row['dds_received_timestamp_ns'] if row['dds_received_timestamp_ns']>0 else None
    for name in ('publication_sequence_number','reception_sequence_number'):
        number=value(name);row[name]=None if number in (None,2**64-1,0) else int(number)
    gid=value('publisher_gid');row['publisher_gid']=list(gid) if gid is not None else None
    return row

def require_image_file(folder,name):
    path=Path(folder)/name
    if not path.is_file() or path.stat().st_size==0:raise RuntimeError('Image completion file missing or empty')

def encode_image(folder,job):
    import numpy as np
    from PIL import Image
    started=time.monotonic();kind=job['kind'];stamp=job['stamp']
    # Used only by unit tests to simulate slow media encoding.
    if job.get('test_delay'):time.sleep(job['test_delay'])
    if kind=='rgb':
        pixels=np.frombuffer(job['data'],dtype=np.uint8).reshape(job['height'],job['step'])[:,:job['width']*3].reshape(job['height'],job['width'],3)
        if job['encoding']=='bgr8':pixels=pixels[:,:,::-1]
        elif job['encoding']!='rgb8':raise ValueError('Unsupported RGB encoding')
        name=f'rgb-{stamp:07.2f}.png'
        with (Path(folder)/name).open('xb') as stream:Image.fromarray(pixels).save(stream,format='PNG')
        meta=dict(encoding=job['encoding'],width=job['width'],height=job['height'],pixel_std=float(pixels.std()))
    else:
        if job['encoding']!='32FC1':raise ValueError('Unsupported depth encoding')
        depth=np.frombuffer(job['data'],dtype='>f4' if job['bigendian'] else '<f4').reshape(job['height'],job['step']//4)[:,:job['width']]
        name=f'depth-{stamp:07.2f}.npz'
        with (Path(folder)/name).open('xb') as stream:np.savez_compressed(stream,depth=depth)
        meta=dict(finite=int((np.isfinite(depth)&(depth>0)).sum()))
    return dict(file=name,archive_started_wall=started,archive_completed_wall=time.monotonic(),**meta)

def image_worker(folder,inbox,outbox):
    try:
        while True:
            job=inbox.get()
            if job is None:break
            meta=encode_image(folder,job)
            outbox.put(dict(id=job['id'],meta=meta))
    except Exception as error:outbox.put(dict(error=repr(error)))
    finally:outbox.put(dict(done=True))

class EventWriter:
    def __init__(self,folder,errors,capacity=EVENT_ITEMS,test_delay=0.,filename='observations.jsonl',receipt_filename='writer-receipts.jsonl',test_close_delay=0.):
        self.folder=Path(folder);self.errors=errors;self.queue=queue.Queue(capacity)
        self.test_delay=test_delay;self.written=0;self.high_water=0
        self.filename=filename;self.receipt_filename=receipt_filename
        self.test_close_delay=test_close_delay;self.lock=threading.Lock()
        self.submitted_ids=[];self.closing=False;self.close_sent=False;self.closed=False
        self.failed=False;self.verified=False;self.timings={}
        self.deadline=None
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()

    def submit(self,row):
        with self.lock:
            if self.closing:raise RuntimeError('Event writer closing')
            if not self.thread.is_alive():raise RuntimeError('Event writer exited')
            try:self.queue.put_nowait(dict(row))
            except queue.Full:raise RuntimeError('Event queue overflow; no silent drop')
            self.submitted_ids.append(row['event_id'])
            self.high_water=max(self.high_water,self.queue.qsize())

    def run(self):
        started=time.monotonic()
        try:
            with (self.folder/self.filename).open('x') as stream,(self.folder/self.receipt_filename).open('x') as receipts:
                while True:
                    row=self.queue.get()
                    if row is None:break
                    if self.test_delay:time.sleep(self.test_delay)
                    row['json_archive_started_wall']=time.monotonic()
                    stream.write(json.dumps(row,allow_nan=False)+'\n')
                    receipts.write(json.dumps(dict(event_id=row['event_id'],archive_completed_wall=time.monotonic()))+'\n')
                    self.written+=1
                self.timings['queue_drained_wall']=time.monotonic()
                if self.test_close_delay:time.sleep(self.test_close_delay)
                stream.flush();receipts.flush()
                self.timings['flushed_wall']=time.monotonic()
            self.closed=True
            self.timings['closed_wall']=time.monotonic()
            # File reads may stall just like writes. Verification belongs in
            # the worker so the caller's join enforces the same absolute limit.
            self.verify_closed(self.deadline)
            self.timings['verified_wall']=time.monotonic()
        except Exception as error:
            self.failed=True;self.errors.append('event writer: '+repr(error))
        finally:self.timings['worker_seconds']=time.monotonic()-started

    def begin_finish(self,deadline):
        # Stop accepting callbacks and enqueue exactly one terminal marker. The
        # worker drains concurrently with the transport reader and child exit.
        with self.lock:
            self.closing=True
            if self.deadline is None:self.deadline=deadline
            deadline=min(deadline,self.deadline)
            if self.close_sent or not self.thread.is_alive():return
            try:self.queue.put(None,timeout=max(0,deadline-time.monotonic()))
            except queue.Full:
                self.failed=True;self.errors.append('event writer drain timeout')
            else:self.close_sent=True

    def verify_closed(self,deadline):
        if self.verified:return
        if not self.closed:raise RuntimeError('Event and receipt files not closed')
        seen=set();count=0
        with (self.folder/self.filename).open() as events,(self.folder/self.receipt_filename).open() as receipts:
            for expected in self.submitted_ids:
                if time.monotonic()>=deadline:raise TimeoutError('Event receipt verification deadline')
                event=json.loads(events.readline());receipt=json.loads(receipts.readline())
                if event['event_id']!=expected or receipt['event_id']!=expected or expected in seen:
                    raise RuntimeError('Event receipt ID mismatch or duplicate')
                seen.add(expected);count+=1
            if events.readline() or receipts.readline() or count!=self.written:
                raise RuntimeError('Event receipt count mismatch')
        self.verified=True

    def finish(self,deadline):
        self.begin_finish(deadline)
        deadline=min(deadline,self.deadline)
        self.thread.join(max(0,deadline-time.monotonic()))
        if self.thread.is_alive():
            self.failed=True;self.errors.append('event writer did not drain')
        elif not self.failed and not self.verified:
            self.failed=True;self.errors.append('event writer closure not verified')
        return dict(written=self.written,submitted=len(self.submitted_ids),high_water=self.high_water,
            closed=self.closed,ids_verified=self.verified,timings=dict(self.timings),
            drained=not self.failed and self.closed and self.verified and not self.thread.is_alive() and self.queue.empty())

class ImageWriter:
    def __init__(self,folder,errors,complete,max_items=IMAGE_ITEMS,max_bytes=IMAGE_BYTES):
        self.folder=Path(folder);self.errors=errors;self.complete=complete;self.max_items=max_items;self.max_bytes=max_bytes
        self.pending={};self.lock=threading.Lock();self.bytes=0;self.next_id=0;self.selected=0;self.completed=0
        self.high_water=0;self.peak_bytes=0;self.maximum_encode_seconds=0.;self.done=False
        ctx=mp.get_context('spawn');self.inbox=ctx.Queue(max_items);self.outbox=ctx.Queue()
        self.process=ctx.Process(target=image_worker,args=(str(folder),self.inbox,self.outbox));self.process.start()
        self.thread=threading.Thread(target=self.receive,daemon=True);self.thread.start()

    def submit(self,job,row):
        if not self.process.is_alive():raise RuntimeError('Image worker exited')
        size=len(job['data'])
        with self.lock:
            if len(self.pending)>=self.max_items or self.bytes+size>self.max_bytes:raise RuntimeError('Image queue overflow; selected sample not discarded')
            self.next_id+=1;key=self.next_id
            self.pending[key]=(dict(row),size);self.bytes+=size;self.selected+=1
            self.high_water=max(self.high_water,len(self.pending));self.peak_bytes=max(self.peak_bytes,self.bytes)
        try:self.inbox.put_nowait(dict(job,id=key))
        except queue.Full:
            self.errors.append('image IPC queue overflow');raise RuntimeError('Image IPC queue overflow')

    def receive(self):
        try:
            while True:
                try:result=self.outbox.get(timeout=.1)
                except queue.Empty:
                    if not self.process.is_alive():
                        # Allow the IPC feeder a bounded interval after child exit.
                        try:result=self.outbox.get(timeout=1.)
                        except queue.Empty:raise RuntimeError('Image worker exited without completion marker')
                    else:continue
                if 'error' in result:self.errors.append('image worker: '+result['error'])
                if result.get('done'):self.done=True;return
                if 'id' not in result:continue
                with self.lock:
                    row,size=self.pending.pop(result['id']);self.bytes-=size
                meta=result['meta']
                require_image_file(self.folder,meta['file'])
                self.maximum_encode_seconds=max(self.maximum_encode_seconds,meta['archive_completed_wall']-meta['archive_started_wall'])
                self.complete(dict(row,**meta));self.completed+=1
        except Exception as error:self.errors.append('image receiver: '+repr(error))

    def finish(self,deadline):
        if self.process.is_alive():
            try:self.inbox.put(None,timeout=max(.001,deadline-time.monotonic()))
            except queue.Full:self.errors.append('image worker drain timeout')
        self.process.join(max(0,deadline-time.monotonic()))
        if self.process.is_alive():self.errors.append('image worker did not drain');self.process.terminate();self.process.join(2)
        self.thread.join(max(0,deadline-time.monotonic()))
        if self.thread.is_alive():self.errors.append('image completion drain timeout')
        with self.lock:pending=len(self.pending)
        if pending or self.selected!=self.completed or not self.done or self.process.exitcode!=0:self.errors.append('image archive incomplete')
        self.inbox.cancel_join_thread();self.inbox.close();self.outbox.close()
        return dict(selected=self.selected,completed=self.completed,pending=pending,high_water=self.high_water,
            peak_bytes=self.peak_bytes,maximum_encode_seconds=self.maximum_encode_seconds,
            drained=pending==0 and self.done and not self.thread.is_alive() and self.process.exitcode==0)
