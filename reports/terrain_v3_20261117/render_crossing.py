"""Present the complete actual Gazebo crossing footage at simulation-time speed."""
import argparse
import bisect
import json
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image,ImageDraw,ImageFont

OUT=Path(__file__).resolve().parent


def rows(path):
    return [json.loads(s) for s in path.read_text(encoding='utf-8').splitlines() if s.strip()]


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--trial',type=Path,required=True)
    args=parser.parse_args();trial=args.trial
    result=json.loads((trial/'case/result.json').read_text())
    recording=json.loads((trial/'video/recording.json').read_text())
    assert recording['encoder_exit']==0 and recording['error'] is None
    states=rows(trial/'case/states.jsonl');captures=rows(trial/'video/frames.jsonl')
    pairs=[(r['wall'],r['state']['motion'].get('diagnostics',{}).get('sim_stamp')) for r in states]
    pairs=[(w,s) for w,s in pairs if s is not None];walls,sims=zip(*pairs)
    assert np.all(np.diff(sims)>=0)
    times=np.interp([r['capture_wall'] for r in captures],walls,sims)
    state_walls=[r['wall'] for r in states]
    fps=20;timeline=np.arange(times[0],times[-1],1/fps).tolist()+[float(times[-1])]*fps*2
    decoder=subprocess.Popen(['ffmpeg','-v','error','-i',str(trial/'video/scene-walltime.mp4'),
                              '-f','rawvideo','-pix_fmt','rgb24','pipe:1'],stdout=subprocess.PIPE)
    output=OUT/'v3-terrain-crossing.mp4'
    encoder=subprocess.Popen(['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s','1120x760',
        '-r',str(fps),'-i','pipe:0','-an','-c:v','libx264','-threads','2','-preset','fast','-crf','20',
        '-pix_fmt','yuv420p','-movflags','+faststart',str(output)],stdin=subprocess.PIPE)
    font='C:/Windows/Fonts/msyh.ttc'
    title_font=ImageFont.truetype(font,25);label_font=ImageFont.truetype(font,18)
    raw_size=recording['width']*recording['height']*3;cursor=-1;picture=None;timing=[]
    try:
        for i,stamp in enumerate(timeline):
            selected=max(0,bisect.bisect_right(times,stamp)-1);capture=captures[selected]
            while cursor<capture['frame']:
                data=decoder.stdout.read(raw_size)
                if len(data)!=raw_size:raise ValueError('Incomplete source recording')
                picture=Image.frombytes('RGB',(recording['width'],recording['height']),data);cursor+=1
            left=(picture.width-560)//2;top=min(max(48,(picture.height+28)//2-180),picture.height-340)
            frame=Image.new('RGB',(1120,760),'#0b1420')
            frame.paste(picture.crop((left,top,left+560,top+340)).resize((1120,680),Image.Resampling.LANCZOS),(0,68))
            draw=ImageDraw.Draw(frame)
            draw.text((20,15),'Terrain V3 · 旧接缝区域通行验证',font=title_font,fill='#edf3f8')
            draw.text((850,22),f'仿真 {stamp:06.2f} s',font=label_font,fill='#a9bac9')
            si=bisect.bisect_right(state_walls,capture['capture_wall'])-1
            motion=states[si]['state']['motion'] if si>=0 else {}
            pose=motion.get('pose');health=motion.get('diagnostics',{}).get('laser_localization',{})
            status='定位健康' if health.get('healthy') else '等待定位就绪'
            label=(f"定位位置 X {pose['x']:+.2f} m  Y {pose['y']:+.2f} m · {status}" if pose else status)
            draw.rectangle((12,695,1108,741),fill='#0b1420')
            draw.text((25,708),label,font=label_font,fill='#edf3f8')
            if i==round(len(timeline)/2):frame.save(OUT/'crossing-preview.png')
            if i==len(timeline)-1:frame.save(OUT/'crossing-end.png')
            encoder.stdin.write(frame.tobytes())
            timing.append(dict(sim=float(stamp),capture_wall=capture['capture_wall'],state_index=si))
        decoder.stdout.read()
    finally:
        decoder.stdout.close();encoder.stdin.close()
    assert decoder.wait(timeout=30)==0 and encoder.wait(timeout=60)==0
    decoded=subprocess.run(['ffmpeg','-v','error','-i',str(output),'-f','null','-'],capture_output=True,text=True,check=True)
    if decoded.stderr.strip():raise RuntimeError(decoded.stderr)
    (OUT/'video-timing.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in timing),encoding='utf-8',newline='\n')
    summary=dict(video=output.name,experiment=result['experiment'],tested_commit=result['software']['git_commit'],
                 frames=len(timeline),fps=fps,seconds=len(timeline)/fps,source_capture_count=len(captures),
                 sim_range=[float(times[0]),float(times[-1])],complete_decode_exit=0,
                 display='Complete actual GUI capture, following camera center crop; simulation-time playback with 2 second final hold',
                 timing='Qualitative interpolation from received wall/sim pairs; each state received by its displayed capture')
    (OUT/'video-summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8',newline='\n')
    print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
