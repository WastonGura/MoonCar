"""Configure only the owned GUI camera and record its status and pose."""
import argparse
import json
from pathlib import Path
import signal
import threading
import time

from gz.transport13 import Node
from gz.msgs10.cameratrack_pb2 import CameraTrack
from gz.msgs10.pose_pb2 import Pose


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    node=Node()
    stopped=threading.Event()
    lock=threading.Lock()
    state=dict(confirmed=False,poses=0)
    last_pose=0.
    request=CameraTrack()
    request.track_mode=CameraTrack.FOLLOW_LOOK_AT
    request.follow_target.name='mooncar'
    request.track_target.name='mooncar'
    request.follow_offset.x=-2.5
    request.follow_offset.y=-3.
    request.follow_offset.z=2.4
    request.follow_pgain=.1
    request.track_pgain=.1
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda *_:stopped.set())
    with (args.output/'poses.jsonl').open('x',buffering=1) as log:
        def status(message):
            with lock:
                state['confirmed']=(message.track_mode==CameraTrack.FOLLOW_LOOK_AT
                                    and message.follow_target.name=='mooncar'
                                    and message.track_target.name=='mooncar')
        def pose(message):
            nonlocal last_pose
            now=time.monotonic()
            if now-last_pose<.5:return
            last_pose=now
            with lock:state['poses']+=1
            log.write(json.dumps(dict(wall=now,xyz=[message.position.x,message.position.y,message.position.z],
                                      q=[message.orientation.x,message.orientation.y,message.orientation.z,message.orientation.w]))+'\n')
        node.subscribe(CameraTrack,'/gui/currently_tracked',status)
        node.subscribe(Pose,'/gui/camera/pose',pose)
        publisher=node.advertise('/gui/track',CameraTrack)
        deadline=time.monotonic()+120
        ready=False
        while not stopped.wait(.5):
            with lock:confirmed=state['confirmed'] and state['poses']>0
            if confirmed and not ready:
                (args.output/'ready.json').write_text(json.dumps(dict(
                    confirmed=True,wall=time.monotonic(),target='mooncar',mode='FOLLOW_LOOK_AT',
                    follow_offset=[-2.5,-3.,2.4]))+'\n')
                ready=True
            if not confirmed and publisher.has_connections():
                publisher.publish(request)
            if not ready and time.monotonic()>deadline:
                raise TimeoutError('Owned GUI camera tracking did not confirm')
        node.unsubscribe('/gui/currently_tracked')
        node.unsubscribe('/gui/camera/pose')
    (args.output/'closed.json').write_text(json.dumps(dict(wall=time.monotonic(),**state))+'\n')


if __name__=='__main__':
    main()
