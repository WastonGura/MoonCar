"""Capture real X11 screenshots; retain exact capture intervals beside CFR video."""
import argparse
import json
import os
from pathlib import Path
import signal
import shlex
import subprocess
import time
from urllib.request import build_opener, ProxyHandler
from PIL import ImageGrab


def gui_capture_environment(experiment,proc=Path('/proc')):
    expected=('mooncar-'+experiment).encode();found=[]
    for path in proc.iterdir():
        if not path.name.isdigit():continue
        try:
            env=dict(x.split(b'=',1) for x in (path/'environ').read_bytes().split(b'\0') if b'=' in x)
            argv=[x.decode() for x in (path/'cmdline').read_bytes().split(b'\0') if x]
            args=shlex.split(argv[0]) if len(argv)==1 else argv
            if (env.get(b'GZ_PARTITION')==expected and '-g' in args and env.get(b'DISPLAY')):
                found.append({key:env[key.encode()].decode() for key in ('DISPLAY','XAUTHORITY') if key.encode() in env})
        except (OSError,ValueError):continue
    if len(found)>1:raise RuntimeError('More than one owned GUI display')
    return found[0] if found else None


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--follow-workbench',action='store_true',help='Follow the owned GUI nested Xvfb display and Xauthority')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    os.sched_setaffinity(0,{5})
    stopped=False
    def stop(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    experiment=None
    if args.follow_workbench:
        deadline=time.monotonic()+120;opener=build_opener(ProxyHandler({}))
        while not stopped and time.monotonic()<deadline:
            try:
                with opener.open('http://127.0.0.1:8766/api/simulation',timeout=2) as stream:value=json.load(stream)
                current=(value.get('task') or {}).get('id')
                if experiment is not None and current!=experiment:raise RuntimeError('Owned experiment changed')
                if current:
                    experiment=current;capture=gui_capture_environment(experiment)
                    if capture:
                        os.environ.update(capture);break
            except OSError:pass
            time.sleep(.2)
        else:raise RuntimeError('No owned GUI display appeared before recording deadline')
    first=ImageGrab.grab(xdisplay=os.environ['DISPLAY']).convert('RGB')
    w,h=first.size;fps=6
    frames=0;error=None
    with (args.output/'ffmpeg.log').open('x') as log, (args.output/'frames.jsonl').open('x',buffering=1) as timing:
        encoder=subprocess.Popen(['ffmpeg','-nostdin','-hide_banner','-loglevel','warning',
            '-f','rawvideo','-pixel_format','rgb24','-video_size',f'{w}x{h}','-framerate',str(fps),'-i','pipe:0',
            '-an','-c:v','libx264','-threads','1','-preset','ultrafast','-crf','23','-pix_fmt','yuv420p',
            '-movflags','+faststart',str(args.output/'scene-walltime.mp4')],stdin=subprocess.PIPE,stderr=log)
        try:
            while not stopped:
                begun=time.monotonic();picture=ImageGrab.grab(xdisplay=os.environ['DISPLAY']).convert('RGB')
                captured=time.monotonic()
                encoder.stdin.write(picture.tobytes())
                timing.write(json.dumps(dict(frame=frames,capture_start_wall=begun,capture_end_wall=captured,
                                             capture_wall=(begun+captured)/2,wall_epoch_ns=time.time_ns()))+'\n')
                frames+=1
                time.sleep(max(0.,1/fps-(time.monotonic()-begun)))
        except Exception as exception:error=repr(exception)
        finally:
            encoder.stdin.close();code=encoder.wait(timeout=30)
    (args.output/'recording.json').write_text(json.dumps(dict(actual_screen_capture=True,
        frame_count=frames,width=w,height=h,nominal_cfr_fps=fps,encoder_exit=code,error=error,
        experiment=experiment,captured_display=os.environ['DISPLAY'],
        timing='Use frames.jsonl for actual wall timing; CFR video is a frame sequence, not a precise wall clock.',
        truth_used_by_control=False),indent=2)+'\n')
    return int(code!=0 or error is not None)


if __name__=='__main__':raise SystemExit(main())
