"""Compose actual turn footage with a causal, vehicle-following observed minimap."""
import argparse
import base64
import bisect
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'feedback_turn_video_20261105'))
from add_wheel_detail import WHEELS, load_wheels, sample

WIDTH, HEIGHT, FPS = 1280, 1072, 20
BG, WHITE, MUTED, CYAN, GOLD = '#0b1420', '#eef4fc', '#9eb0c7', '#77d9e8', '#ffd17e'


def rows(path):
    with path.open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    result = read(args.trial/'case/result.json')
    autonomy = result.get('autonomy', {})
    actions = autonomy.get('history', [])
    states = rows(args.trial/'case/states.jsonl')
    grids = rows(args.trial/'case/navigation-grids.jsonl')
    capture = rows(args.trial/'video/frames.jsonl')
    recording = read(args.trial/'video/recording.json')
    if recording['encoder_exit']:
        raise RuntimeError('Source screen recording encoding failed')
    pairs = [(r['wall'], r['state']['motion'].get('diagnostics', {}).get('sim_stamp')) for r in states]
    pairs = [(wall, stamp) for wall, stamp in pairs if stamp is not None]
    walls, sims = map(np.asarray, zip(*pairs))
    if np.any(np.diff(sims)<0):
        raise ValueError('Simulation clock went backwards')
    capture_times = np.interp([r['capture_wall'] for r in capture], walls, sims)
    start, end = float(capture_times[0]), float(capture_times[-1])
    starts = [autonomy['started']+r['started_seconds'] for r in actions]
    turn = next((s for s, a in zip(starts, actions) if a['kind']=='turn'), None)
    turn_end = next((a['end_sim'] for a in actions if a['kind']=='turn' and 'end_sim' in a), None)
    stopped_sim = autonomy.get('started', 0)+autonomy.get('elapsed_seconds', math.inf)
    event_path = args.trial/'fixture/feedback-turn.jsonl'
    terminals = [row for row in rows(event_path) if row.get('event')=='terminal'] if event_path.exists() else []
    stop_reason = terminals[-1]['reason'] if terminals else None
    stop_label = {'predicted_obstacle_footprint':'预测足迹触及地图障碍或安全余量',
                  'predicted_unknown_footprint':'预测足迹进入未知区域'}.get(stop_reason, autonomy.get('reason','保护停车'))
    truth = rows(args.trial/'case/observer/truth.jsonl')
    truth_times = [r['stamp'] for r in truth]
    commands = [r for r in rows(args.trial/'case/observer/samples.jsonl')
                if r.get('topic')=='/mooncar/cmd_vel_safe' and r.get('sim') is not None]
    command_times = [r['sim'] for r in commands]
    wheels = None
    wheel_error = None
    try:
        reference = turn if turn is not None else starts[0] if starts else truth[0]['stamp']+1.
        wheels, _ = load_wheels(args.trial/'case/observer', reference)
    except ValueError as error:
        wheel_error = str(error)
    font = Path('C:/Windows/Fonts/msyh.ttc')
    if not font.exists():
        font = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    fonts = {size: ImageFont.truetype(str(font), size) for size in (13, 15, 17, 19, 22, 25, 28)}
    state_walls = [r['wall'] for r in states]
    grid_walls = [r['wall'] for r in grids]
    pose_rows = [r for r in states if r['state']['motion'].get('pose')]
    pose_walls = [r['wall'] for r in pose_rows]
    rasters = {}
    alignment = []

    def text(draw, xy, value, size=19, fill=WHITE):
        draw.text(xy, str(value), font=fonts[size], fill=fill)

    def raster(index):
        if index not in rasters:
            grid = grids[index]['grid']
            data = np.frombuffer(base64.b64decode(grid['data']), dtype=np.uint8).reshape(grid['height'], grid['width'])
            rgb = np.empty((*data.shape, 3), dtype=np.uint8)
            rgb[:] = (22, 34, 49)
            rgb[data==0] = (72, 110, 99)
            rgb[(data>0)&(data<99)] = (121, 116, 74)
            rgb[data==99] = (213, 139, 55)
            rgb[(data>=100)&(data!=255)] = (175, 65, 65)
            rasters[index] = Image.fromarray(np.flipud(rgb))
        return rasters[index]

    def minimap(frame, wall, stamp, state):
        d = ImageDraw.Draw(frame)
        box = (25, 83, 335, 461)
        d.rounded_rectangle(box, radius=16, fill='#101d2d', outline='#587084', width=2)
        text(d, (40, 94), '导航小地图', 22)
        text(d, (192, 100), '随车 · 北向上', 15, MUTED)
        x0, y0, size, span = 40, 130, 280, 8.
        pi = bisect.bisect_right(pose_walls, wall)-1
        pose = pose_rows[pi]['state']['motion']['pose'] if pi>=0 else None
        center = (pose['x'], pose['y']) if pose else (0., 0.)
        gi = bisect.bisect_right(grid_walls, wall)-1
        while gi>=0 and grids[gi]['grid'].get('stamp', 0)>stamp+1e-6:
            gi -= 1
        if gi>=0:
            g = grids[gi]['grid']
            r, ox, oy = g['resolution'], *g['origin']
            left = (center[0]-span/2-ox)/r
            top = g['height']-(center[1]+span/2-oy)/r
            scale = span/(size*r)
            crop = raster(gi).transform((size,size), Image.Transform.AFFINE,
                                        (scale,0,left,0,scale,top), Image.Resampling.NEAREST,
                                        fillcolor='#162231')
            frame.paste(crop, (x0, y0))
        else:
            d.rectangle((x0, y0, x0+size, y0+size), fill='#162231')
        d = ImageDraw.Draw(frame)
        def point(x, y):
            return x0+size/2+(x-center[0])*size/span, y0+size/2-(y-center[1])*size/span
        for coordinate in range(math.ceil(center[0]-span/2), math.floor(center[0]+span/2)+1):
            px, _ = point(coordinate, center[1])
            d.line((px, y0, px, y0+size), fill='#45554e', width=1)
        for coordinate in range(math.ceil(center[1]-span/2), math.floor(center[1]+span/2)+1):
            _, py = point(center[0], coordinate)
            d.line((x0, py, x0+size, py), fill='#45554e', width=1)
        path = []
        for row in pose_rows[max(0, pi-1500):pi+1:3]:
            p = row['state']['motion']['pose']
            px, py = point(p['x'], p['y'])
            if x0+3<=px<=x0+size-3 and y0+3<=py<=y0+size-3:
                path.append((px, py))
        if len(path)>1:
            d.line(path, fill=CYAN, width=3)
        goal = state['motion'].get('goal')
        if goal:
            px, py = point(*goal[:2])
            if x0+10<=px<=x0+size-10 and y0+10<=py<=y0+size-10:
                d.ellipse((px-7, py-7, px+7, py+7), outline=GOLD, width=2)
        if pose:
            c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
            def body(dx, dy):
                return point(pose['x']+c*dx-s*dy, pose['y']+s*dx+c*dy)
            d.polygon([body(dx, dy) for dx, dy in ((-.43,-.36),(.43,-.36),(.43,.36),(-.43,.36))],
                      fill='#e9eff5', outline='#0b1420', width=2)
            d.polygon([body(.72,0), body(.2,.21), body(.2,-.21)], fill=GOLD)
        text(d, (x0+size-24, y0+7), 'N', 19, WHITE)
        d.line((x0+10, y0+size-13, x0+45, y0+size-13), fill=WHITE, width=2)
        text(d, (x0+10, y0+size-35), '1 m', 13)
        text(d, (40, 417), '灰 未知   绿 已知   褐 附加代价', 13, MUTED)
        text(d, (40, 439), '橙 安全余量   红 禁行', 13, MUTED)
        if not pose or state['motion'].get('pose') is None:
            d.rectangle((48, 256, 312, 284), fill='#101d2d')
            text(d, (60, 260), '等待有效定位', 17, GOLD)
        if gi<0:
            text(d, (99, 298), '等待观测地图', 17, GOLD)
        return gi, pi

    def stage(stamp):
        if stamp>=stopped_sim and autonomy.get('phase')=='fault':
            return '任务已停车', stop_label
        index = bisect.bisect_right(starts, stamp)-1
        if index<0:
            return '启动观察与建图', '从起点建立已观测区域'
        action = actions[index]
        name = action['action'][0]
        label = (f"前进至 X {action['action'][1]:g} m" if name=='move' else
                 '左转 90°' if abs(action['action'][1]-math.pi/2)<.01 else '回正至 0°')
        done = stamp>=action.get('end_sim', float('inf'))
        status = '已完成' if action.get('success') else '已停止'
        return label, status if done else '执行中'

    def frame(picture, stamp, wall, held=False):
        image = Image.new('RGB', (WIDTH, HEIGHT), BG)
        ti = bisect.bisect_right(truth_times, stamp)-1
        if ti>=0:
            # The recorded GUI camera has a fixed known pose and a 90 degree
            # horizontal field of view. Use the logged body position solely to
            # center a digital crop of the actual pixels.
            position = np.asarray(truth[ti]['xyz'])+np.array([0.,0.,.15])
            delta = position-np.array([2.,-5.,3.2])
            c,s = math.cos(.48),math.sin(.48)
            depth = delta @ np.array([0.,c,-s])
            focal = (picture.width-20)/2
            px = focal+focal*delta[0]/max(depth,.1)
            py = (picture.height+28)/2-focal*(delta @ np.array([0.,s,c]))/max(depth,.1)
            left = min(max(0,round(px-280)),picture.width-560)
            top = min(max(48,round(py-170)),picture.height-340)
            picture = picture.crop((left,top,left+560,top+340))
        fitted = ImageOps.contain(picture, (WIDTH, 720), method=Image.Resampling.LANCZOS)
        image.paste(fitted, ((WIDTH-fitted.width)//2, 68+(720-fitted.height)//2))
        d = ImageDraw.Draw(image)
        text(d, (25, 13), '寻迹者探测车   V2 上坡与转向', 28)
        text(d, (806, 23), f"版本 {result['software']['git_commit'][:7]}    仿真 {stamp:06.2f} s", 17, MUTED)
        si = bisect.bisect_right(state_walls, wall)-1
        state = states[si]['state'] if si>=0 else dict(motion={})
        gi, pi = minimap(image, wall, stamp, state)
        d = ImageDraw.Draw(image)
        label, status = stage(stamp)
        d.rounded_rectangle((894, 88, 1255, 185), radius=13, fill='#101d2d')
        text(d, (911, 99), label, 22)
        color = '#ff998d' if label=='任务已停车' or status=='已停止' else CYAN
        for i in range(0,min(len(status),40),20):
            text(d, (911, 132+(i//20)*23), status[i:i+20], 17, color)
        speed = '.25× 仿真时间' if turn is not None and turn<=stamp<=((turn_end or end)+2.) else '1× 仿真时间'
        text(d, (26, 767), 'Gazebo 现场录屏 · 随车裁切', 17)
        text(d, (826, 767), '记录结束 · 停帧' if held else speed, 17, MUTED)
        d.rectangle((0, 794, WIDTH, HEIGHT), fill=BG)
        text(d, (25, 806), '四轮状态', 22)
        reference_label = '转向起点' if turn is not None else '首段行驶起点'
        text(d, (171, 812), f'根据关节与车体记录计算 · 累计转角以{reference_label}为零', 15, MUTED)
        for i, (name, label, tag, color) in enumerate(WHEELS):
            x = 24+i*314
            d.rounded_rectangle((x, 846, x+298, 1048), radius=13, fill='#172638')
            text(d, (x+15, 857), label+'  '+tag, 22, color)
            rpm, lateral, angle, height = sample(wheels[name], stamp) if wheels else [math.nan]*4
            def number(value, decimals):
                return f'{value:+.{decimals}f}' if math.isfinite(value) else '—'
            cx, cy = x+48, 944
            d.ellipse((cx-27, cy-27, cx+27, cy+27), outline='#8495a8', width=2)
            if math.isfinite(angle):
                theta = math.radians(angle)
                d.line((cx,cy,cx+25*math.sin(theta),cy-25*math.cos(theta)), fill=color, width=3)
            text(d, (x+91, 904), f'轮速  {number(rpm,2)} rpm', 17, color)
            text(d, (x+91, 937), f'滚动  {number(angle,1)}°', 17)
            text(d, (x+91, 970), f'横移  {number(lateral,1)} cm/s', 15)
            text(d, (x+91, 1002), f'悬架  {number(height,2)} mm', 15)
        ci = bisect.bisect_right(command_times, stamp)-1
        if ti>=0 and ci>=0:
            pose, command = truth[ti], commands[ci]
            d.rectangle((355, 706, 1248, 751), fill='#101d2d')
            text(d, (371, 716), f"高度 {pose['xyz'][2]:.3f} m    航向 {math.degrees(pose['yaw']):+.1f}°    安全速度 {command['linear']:+.3f} m/s  {command['angular']:+.3f} rad/s", 17)
        alignment.append(dict(sim=stamp,capture_wall=wall,state_index=si,grid_index=gi,pose_index=pi))
        return image

    timeline = []
    stamp = start
    while stamp<end:
        timeline.append(stamp)
        slow = turn is not None and turn<=stamp<=((turn_end or end)+2.)
        stamp += (.25 if slow else 1.)/FPS
    timeline.extend([end]*(2*FPS))
    raw_size = recording['width']*recording['height']*3
    decoder = subprocess.Popen(['ffmpeg','-v','error','-i',str(args.trial/'video/scene-walltime.mp4'),
                                '-f','rawvideo','-pix_fmt','rgb24','pipe:1'],stdout=subprocess.PIPE)
    target = args.output/'v2-uphill-turn-minimap.mp4'
    encoder = subprocess.Popen(['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','rgb24',
                                '-s',f'{WIDTH}x{HEIGHT}','-r',str(FPS),'-i','pipe:0',
                                '-an','-c:v','libx264','-threads','2','-preset','fast','-crf','20',
                                '-pix_fmt','yuv420p','-movflags','+faststart',str(target)],stdin=subprocess.PIPE)
    cursor = -1
    picture = None
    previews = [(start+.5,'startup.png'), (turn-.3 if turn else (start+end)/2,'approach.png'),
                (min(end,turn+.7) if turn else end,'turn.png'), (end,'end.png')]
    try:
        for i, stamp in enumerate(timeline):
            index = max(0, bisect.bisect_right(capture_times,stamp)-1)
            wanted = capture[index]['frame']
            while cursor<wanted:
                data = decoder.stdout.read(raw_size)
                if len(data)!=raw_size:raise ValueError('Source footage ended before the selected frame')
                picture = Image.frombytes('RGB',(recording['width'],recording['height']),data)
                cursor += 1
            composed = frame(picture,stamp,capture[index]['capture_wall'],i>=len(timeline)-FPS*2)
            while previews and stamp>=previews[0][0]:
                composed.save(args.output/previews.pop(0)[1])
            encoder.stdin.write(composed.tobytes())
        decoder.stdout.read()
    finally:
        decoder.stdout.close()
        encoder.stdin.close()
        decoded, encoded = decoder.wait(timeout=30), encoder.wait(timeout=60)
    if decoded or encoded:raise RuntimeError('Video decoder or encoder failed')
    (args.output/'frame-alignment.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in alignment),encoding='utf-8')
    summary = dict(video=str(target),experiment=result.get('experiment'),tested_commit=result['software']['git_commit'],
                   outcome=result['outcome'],reason=autonomy.get('reason'),actions=actions,
                   controller_stop_reason=stop_reason,
                   sim_range=[start,end],video_seconds=len(timeline)/FPS,frames=len(timeline),fps=FPS,
                   actual_captures=len(capture),navigation_snapshots=len(grids),wheel_error=wheel_error,
                   capture_error=recording.get('error'),
                   encoder_exit=encoded,decoder_exit=decoded,
                   map_policy='Only navigation snapshots received by the displayed capture are used; unknown remains unknown.',
                   vehicle_marker='Latest available localization pose; north-up map follows that position.',
                   display_timing='Screenshots selected causally on interpolated wall/sim pairs; qualitative event alignment.')
    (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False),flush=True)


if __name__=='__main__':
    main()
