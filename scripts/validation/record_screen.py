"""Capture real X11 screenshots; retain exact capture intervals beside CFR video."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from PIL import ImageGrab


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    os.sched_setaffinity(0,{5})
    stopped=False
    def stop(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
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
        timing='Use frames.jsonl for actual wall timing; CFR video is a frame sequence, not a precise wall clock.',
        truth_used_by_control=False),indent=2)+'\n')
    return int(code!=0 or error is not None)


if __name__=='__main__':raise SystemExit(main())
