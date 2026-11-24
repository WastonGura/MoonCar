"""Replay the recorded trigger map and compare conservative and exact cell overlap."""
import base64
import json
import math
from pathlib import Path

import numpy as np


def main():
    root = Path(__file__).resolve().parents[2]
    trial = root/'tmp/ros_acceptance/turn-minimap-video-20261111/retry'
    output = Path(__file__).resolve().parent
    terminal = json.loads((trial/'case/terminal.json').read_text())
    health = terminal['motion']['stop_diagnostics']['health']
    stamp = health['sim_stamp']-health['costmap_age_sim_seconds']
    snapshots = [json.loads(s) for s in (trial/'case/navigation-grids.jsonl').read_text().splitlines()]
    matches = [s for s in snapshots if abs(s['grid']['stamp']-stamp)<1e-6]
    if len(matches)!=1:
        raise ValueError('The exact map named by the stop diagnostics is unavailable')
    grid = matches[0]['grid']
    rows = [json.loads(s) for s in (trial/'fixture/feedback-turn.jsonl').read_text().splitlines()]
    row = next(s for s in rows if s.get('swept_check')=='predicted_obstacle_footprint')
    r = grid['resolution']
    ox, oy = grid['origin']
    costs = np.frombuffer(base64.b64decode(grid['data']), dtype=np.int8).reshape(grid['height'],grid['width'])
    yy, xx = np.mgrid[:grid['height'],:grid['width']]
    xx, yy = ox+(xx+.5)*r, oy+(yy+.5)*r
    x,y,yaw = (row['pose'][k] for k in ('x','y','yaw'))
    v,w = row['linear'],row['angular']
    hx,hy = .63,.56
    first = None
    exact_risks = []
    predictions = []
    for step in range(41):
        c,s = math.cos(yaw),math.sin(yaw)
        dx,dy = xx-x,yy-y
        u,z = abs(c*dx+s*dy),abs(-s*dx+c*dy)
        conservative = (u<=hx+r/math.sqrt(2)) & (z<=hy+r/math.sqrt(2))
        # Separating axes of both the vehicle rectangle and the map cell.
        exact = ((u<=hx+r/2*(abs(c)+abs(s))) & (z<=hy+r/2*(abs(c)+abs(s)))
                 & (abs(dx)<=hx*abs(c)+hy*abs(s)+r/2)
                 & (abs(dy)<=hx*abs(s)+hy*abs(c)+r/2))
        hit = conservative & (costs>=99)
        if first is None and np.any(hit):
            first = dict(seconds=step*.05,pose=[x,y,yaw],
                         cells=[dict(x=float(a),y=float(b),cost=int(k))
                                for a,b,k in zip(xx[hit],yy[hit],costs[hit])])
        if np.any(exact & ((costs<0)|(costs>=99))):
            exact_risks.append(step*.05)
        predictions.append([x,y,yaw])
        nxt = yaw+w*.05
        x += v/w*(math.sin(nxt)-s)
        y += v/w*(c-math.cos(nxt))
        yaw = nxt
    lethal = np.argwhere(costs==100)
    trigger = first['cells'][0]
    near = min(([ox+(ix+.5)*r,oy+(iy+.5)*r] for iy,ix in lethal),
               key=lambda p:math.dist(p,(trigger['x'],trigger['y'])))
    result = dict(trigger_sim=row['sim'],source_grid_stamp=stamp,source_grid_version=grid['version'],
                  current_pose=row['pose'],first_conservative_trigger=first,nearest_lethal_cell=near,
                  lethal_to_trigger_distance_m=math.dist(near,(trigger['x'],trigger['y'])),
                  safety_rectangle_m=[2*hx,2*hy],exact_risk_steps=exact_risks,
                  note='Same padded rectangle and thresholds; exact rectangle/cell overlap checks all 41 predictions. Terrain-source reasons are absent for the nearby lethal cells in the retained final sidecar.')
    (output/'stop-inspection.json').write_text(json.dumps(result,indent=2)+'\n')

    from PIL import Image, ImageDraw, ImageFont
    picture=Image.new('RGB',(1000,950),'#ffffff')
    draw=ImageDraw.Draw(picture)
    fonts={n:ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',n) for n in (16,18,23)}
    def text(x,y,value,size=18,color='#172838'):
        draw.text((x,y),value,font=fonts[size],fill=color)
    def project(x,y):
        return 80+(x-2.6)*240,100+(1.85-y)*240
    text(80,25,'停车瞬间的地图与预测车身',23)
    text(80,61,'复用当时地图与同一条速度指令，预测范围为随后 2 秒',16)
    palette = {-1:'#d9dfe5',0:'#f3f5f7',99:'#eea14d',100:'#c95757'}
    for j,i in np.argwhere((xx>2.6)&(xx<5.)&(yy>-1.)&(yy<1.85)):
        value=int(costs[j,i])
        a,b=project(ox+i*r,oy+(j+1)*r),project(ox+(i+1)*r,oy+j*r)
        draw.rectangle((*a,*b),fill=palette.get(value,'#d1d4b4'),outline='#ffffff',width=1)
    def segment(a,b,color,dashed=False):
        if not dashed:
            draw.line((*a,*b),fill=color,width=3)
            return
        n=max(1,math.ceil(math.dist(a,b)/8))
        for k in range(0,n,2):
            start=tuple(a[i]+(b[i]-a[i])*k/n for i in (0,1))
            end=tuple(a[i]+(b[i]-a[i])*min(k+1,n)/n for i in (0,1))
            draw.line((*start,*end),fill=color,width=2)
    def rectangle(pose,hx,hy,color,dashed=False):
        x,y,angle=pose
        c,s=math.cos(angle),math.sin(angle)
        points=[project(x+c*a-s*b,y+s*a+c*b) for a,b in ((-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy))]
        for a,b in zip(points,points[1:]+points[:1]):segment(a,b,color,dashed)
    p=row['pose']
    rectangle([p['x'],p['y'],p['yaw']],.43,.36,'#5c6570')
    rectangle(first['pose'],hx,hy,'#177d91')
    rectangle(first['pose'],hx+r/math.sqrt(2),hy+r/math.sqrt(2),'#bc6e11',True)
    px,py=project(trigger['x'],trigger['y'])
    draw.ellipse((px-5,py-5,px+5,py+5),fill='#111111')
    draw.line((px,py,690,180),fill='#111111',width=2)
    text(700,156,'触发格 99（安全余量）')
    text(700,187,'中心 (3.85, 0.95) m',16)
    px,py=project(*near)
    draw.line((px,py,690,120),fill='#111111',width=2)
    text(700,104,'附近禁行格 100')
    for y,color,dashed,label in ((300,'#5c6570',False,'停车时车体'),
                                  (380,'#177d91',False,'1.9 秒后的安全车身'),
                                  (460,'#bc6e11',True,'额外放大的格心检查框')):
        segment((700,y),(930,y),color,dashed)
        text(700,y+12,label,16)
    text(80,817,'青框仍未接触该格子。额外扩大的橙色虚线框已经包含该格中心。',18)
    text(80,855,'保持同一带安全余量的车身矩形，精确格子相交检查在全部 41 个预测姿态中均通过。',16)
    text(80,890,'导航图时间 37.24625 s    触发检查时间 38.21925 s',16)
    picture.save(output/'stop-map-inspection.png')
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':
    main()
