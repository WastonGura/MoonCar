"""Render timestamped diagnostic MP4s from retained evidence, without simulation.

GUI pixels are decoded from real screenshots. Top views are explicitly labelled
log reconstructions; no generated images, resimulation, or truth-based control.
"""
import argparse
import bisect
import json
import math
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def rows(path):return [json.loads(s) for s in path.read_text().splitlines() if s.strip()]
def load(path):return json.loads(path.read_text())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--mode',choices=('candidate','baseline'),required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    candidate=args.mode=='candidate'
    trial=args.root/'trial-run' if candidate else args.root/'fixed'
    result=load(trial/'case/result.json');a=result['autonomy'];hist=a['history']
    turn_start=a['started']+hist[3]['started_seconds']
    turn_end=hist[-1]['end_sim'] if candidate else a['started']+hist[5]['started_seconds']
    truth=rows(trial/'case/observer/truth.jsonl');align=rows(trial/'case/observer/alignment.jsonl')
    commands=[r for r in rows(trial/'case/observer/samples.jsonl') if r.get('kind')=='command'
              and r.get('topic')=='/mooncar/cmd_vel_safe' and r.get('sim') is not None]
    events=rows(trial/'fixture/feedback-turn.jsonl') if candidate else []
    t=np.array([r['stamp'] for r in truth]);ct=[r['sim'] for r in commands]
    at=[r['stamp'] for r in align];et=[r['sim'] for r in events]
    start=max(t[0],turn_start-1.2);end=min(t[-1],turn_end+.3)
    first=truth[min(range(len(t)),key=lambda i:abs(t[i]-turn_start))]
    anchor=first['xyz'][:2]
    selected=[r for r in truth if turn_start<=r['stamp']<=turn_end]
    max_drift=max(math.dist(r['xyz'][:2],anchor) for r in selected)
    # Chinese font is a local rendering dependency, not redistributed in Git.
    font_path=Path('/evidence/feedback-turn-video-20261105/caption-font.ttc')
    if not font_path.exists():font_path=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    fonts={n:ImageFont.truetype(str(font_path),n) for n in (15,18,20,24,30)}
    bg='#111925';white='#e6edf6';red='#ff7770';blue='#62d1eb';yellow='#ffd275'
    def text(draw,xy,value,size=20,color=white):draw.text(xy,str(value),font=fonts[size],fill=color)
    def current(data,times,stamp):return data[max(0,bisect.bisect_right(times,stamp)-1)] if data else None
    def top_view(draw,stamp,box):
        x0,y0,x1,y1=box;cx=(x0+x1)/2;cy=(y0+y1)/2;scale=min(x1-x0,y1-y0)/1.8
        draw.rounded_rectangle(box,radius=10,fill='#182434')
        def project(x,y):return (cx+(x-anchor[0])*scale,cy-(y-anchor[1])*scale)
        for v in np.arange(-.8,.81,.2):
            draw.line([(cx+v*scale,y0+8),(cx+v*scale,y1-8)],fill='#2b3b4d')
            draw.line([(x0+8,cy+v*scale),(x1-8,cy+v*scale)],fill='#2b3b4d')
        p=project(*anchor);draw.ellipse([p[0]-6,p[1]-6,p[0]+6,p[1]+6],outline=yellow,width=2)
        trace=[project(*r['xyz'][:2]) for r in truth if turn_start<=r['stamp']<=stamp]
        if len(trace)>1:draw.line(trace,fill=red,width=3)
        estimated=[project(*r['map_xyz'][:2]) for r in align if turn_start<=r['stamp']<=stamp]
        if len(estimated)>1:draw.line(estimated,fill=blue,width=2)
        r=current(truth,t,stamp);x,y=r['xyz'][:2];yaw=r['yaw'];c,s=math.cos(yaw),math.sin(yaw)
        def point(dx,dy):return project(x+c*dx-s*dy,y+s*dx+c*dy)
        draw.polygon([point(dx,dy) for dx,dy in [(-.16,-.125),(.16,-.125),(.16,.125),(-.16,.125)]],fill='#b4bfce',outline=white)
        for dx in (-.157,.157):
            for dy in (-.16555,.16555):
                draw.polygon([point(dx+u,dy+v) for u,v in [(-.076,-.025),(.076,-.025),(.076,.025),(-.076,.025)]],fill='#1e2228',outline='#abb5c1')
        draw.line([point(0,0),point(.3,0)],fill=yellow,width=4)
        text(draw,(x0+10,y0+8),'俯视轨迹｜日志重建',18)
        text(draw,(x0+10,y1-48),'← 下坡 -X     ↑ +Y',15)
        text(draw,(x0+10,y1-27),'红：实际  青：定位  黄圈：起点',15)
    graphs=[('偏移 / m',0.,.8,lambda r:math.dist(r['xyz'][:2],anchor),truth,t,red),
            ('航向 / °',-30.,115.,lambda r:math.degrees(r['yaw']),truth,t,yellow),
            ('安全线速度 / m/s',-.15,.15,lambda r:r['linear'],commands,ct,blue),
            ('安全角速度 / rad/s',-.35,.35,lambda r:r['angular'],commands,ct,'#c0a1ff')]
    def frame(picture,stamp,held=False):
        im=Image.new('RGB',(1280,800),bg);d=ImageDraw.Draw(im)
        title='位置反馈原型｜Gazebo 实际录屏' if candidate else '旧默认方案｜独立真值日志回放（非现场录屏）'
        text(d,(22,12),title,30)
        text(d,(22,53),f'仿真时间 {stamp:.2f}s   转向起点后 {stamp-turn_start:+.2f}s   '+('0.25×仿真时间慢放' if candidate else '0.5×仿真时间慢放'),18)
        if picture is not None:
            # Digital crop only; no synthetic motion or retouching of GUI pixels.
            view=picture.crop((300,185,850,590)).resize((864,560),Image.Resampling.LANCZOS)
            im.paste(view,(20,89));d=ImageDraw.Draw(im)
            d.rectangle((20,89,884,120),fill=bg)
            text(d,(30,92),'现场画面局部放大｜截图与指标按时间戳近邻对齐',18)
            top_view(d,stamp,(904,89,1260,452))
        else:
            top_view(d,stamp,(20,89,884,649))
            text(d,(908,101),'按已记录车体位姿绘制',20)
            text(d,(908,134),'不是重新运行的仿真',18)
            text(d,(908,167),'轮形示意，不含轮角/悬架回放',15)
        r=current(truth,t,stamp);cmd=current(commands,ct,stamp)
        status=current(events,et,stamp) if events else None
        drift=math.dist(r['xyz'][:2],anchor)
        text(d,(908,472),f'实际偏移：{drift*100:.2f} cm',24,red)
        text(d,(908,510),f'实际航向：{math.degrees(r["yaw"]):.1f}°',20)
        if candidate:
            failed=status is not None and status.get('status')=='failed'
            text(d,(908,546),'已安全停止' if failed else '正在转向',24,yellow)
            if failed:
                text(d,(908,581),'预测扫掠触及地图障碍',18)
                text(d,(908,609),'未完成90°，未进入回位纠偏',15)
        else:
            text(d,(908,227),'两次转向+停留最大偏移',18)
            text(d,(908,261),f'{max_drift*100:.1f} cm',30,red)
            text(d,(908,317),'该段动作均完成',20)
            text(d,(908,353),'位置没有被锁住',20)
        for n,(label,lo,hi,fn,data,times,color) in enumerate(graphs):
            bx=20+n*315;by=681;bw=290;bh=83
            d.rectangle((bx,by,bx+bw,by+bh),fill='#182434')
            text(d,(bx,655),label,15)
            pts=[]
            for j in range(0,len(data),max(1,len(data)//1200)):
                st=times[j]
                if start<=st<=min(stamp,end):
                    pts.append((bx+(st-start)/(end-start)*bw,by+bh-(max(lo,min(hi,fn(data[j])))-lo)/(hi-lo)*bh))
            if len(pts)>1:d.line(pts,fill=color,width=2)
            zero=by+bh-(0-lo)/(hi-lo)*bh;d.line([(bx,zero),(bx+bw,zero)],fill='#546071')
            text(d,(bx,766),f'{lo:g} … {hi:g}',15,'#9fadc0')
        if held:
            d.rectangle((20,605,884,649),fill=bg)
            text(d,(32,612),'记录已结束｜此处停帧便于查看',24,yellow)
        return im
    name='feedback-turn-diagnostic.mp4' if candidate else 'baseline-turn-replay.mp4'
    dest=args.output/name;fps=20;speed=.25 if candidate else .5
    timeline=list(np.arange(start,end, speed/fps))
    if candidate:
        video=trial/'actual-video-run';meta=load(video/'recording.json');captures=rows(video/'frames.jsonl')
        states=rows(trial/'case/states.jsonl')
        pairs=[(r['wall'],r['state']['motion'].get('diagnostics',{}).get('sim_stamp')) for r in states]
        pairs=[(wall,sim) for wall,sim in pairs if sim is not None]
        ctimes=np.interp([r['capture_wall'] for r in captures],[v[0] for v in pairs],[v[1] for v in pairs])
        indexes=[int(np.argmin(abs(ctimes-stamp))) for stamp in timeline]
        decoder=subprocess.Popen(['ffmpeg','-v','error','-i',str(video/'scene-walltime.mp4'),'-f','rawvideo','-pix_fmt','rgb24','pipe:1'],stdout=subprocess.PIPE)
        cursor=-1;picture=None
    encoder=subprocess.Popen(['ffmpeg','-nostdin','-v','error','-y','-f','rawvideo','-pixel_format','rgb24','-video_size','1280x800',
        '-framerate',str(fps),'-i','pipe:0','-an','-c:v','libx264','-threads','2','-preset','fast','-crf','20',
        '-pix_fmt','yuv420p','-movflags','+faststart',str(dest)],stdin=subprocess.PIPE)
    try:
        for i,stamp in enumerate(timeline):
            if candidate:
                while cursor<indexes[i]:
                    blob=decoder.stdout.read(meta['width']*meta['height']*3)
                    if len(blob)!=meta['width']*meta['height']*3:raise RuntimeError('Screenshot video ended before requested frame')
                    picture=Image.frombytes('RGB',(meta['width'],meta['height']),blob);cursor+=1
            composed=frame(picture if candidate else None,float(stamp))
            encoder.stdin.write(composed.tobytes())
            if i==len(timeline)//2:composed.save(args.output/(args.mode+'-preview.png'))
        last=frame(picture if candidate else None,end,held=True)
        for _ in range(fps*2):encoder.stdin.write(last.tobytes())
        last.save(args.output/(args.mode+'-end.png'))
    finally:
        encoder.stdin.close();code=encoder.wait(timeout=60)
        if candidate:
            decoder.stdout.close();decoder.terminate();decoder.wait(timeout=10)
    if code:raise RuntimeError('Video encoder failed')
    summary=dict(mode=args.mode,actual_gui_pixels=candidate,start_sim=float(start),end_sim=float(end),
        turn_start_sim=turn_start,turn_end_sim=turn_end,maximum_drift_m=max_drift,
        simulation_playback_speed=speed,fps=fps,frames=len(timeline)+fps*2,video=str(dest),
        source_experiment=result['experiment'],tested_git_commit=result['software']['git_commit'],
        outcome=result['outcome'],history=hist,screen_alignment='nearest capture matched using sampled workbench wall/sim clocks; qualitative, not exact contact-frame synchronization')
    (args.output/(args.mode+'-summary.json')).write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:summary[k] for k in ['mode','outcome','start_sim','end_sim','maximum_drift_m','video']},ensure_ascii=False))


if __name__=='__main__':main()
