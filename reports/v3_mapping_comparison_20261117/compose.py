"""Full two-run footage plus causal reconstruction/reference/error panels."""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from map_reference import build, compare, N

OUTPUT = Path(__file__).resolve().parent
BG, WHITE, MUTED = '#0b1420', '#edf3f8', '#a8b7c6'
BLUE, ORANGE = '#65b8ff', '#ffb366'
UNKNOWN, INVALID, MIXED = (30, 42, 57), (224, 164, 49), (165, 106, 192)
W, H, SIDE = 2560, 820, 440
FONTS = {s:ImageFont.truetype('C:/Windows/Fonts/msyh.ttc', s) for s in (16, 18, 20, 22, 24, 28, 30)}
HEIGHT_COLORS = np.array([[33, 73, 126], [55, 145, 170], [116, 187, 153], [226, 217, 149], [205, 146, 96], [241, 232, 219]], dtype=float)


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def rows(path):
    with path.open(encoding='utf-8') as stream:
        return [json.loads(s) for s in stream if s.strip()]


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8', newline='\n')


def color(values, low, high, error=False):
    t = np.clip((np.nan_to_num(values, nan=low)-low)/(high-low), 0, 1)
    palette = np.array([[56, 118, 215], [243, 244, 243], [214, 63, 54]], float) if error else HEIGHT_COLORS
    scaled = t*(len(palette)-1)
    i = np.minimum(np.floor(scaled).astype(int), len(palette)-2)
    f = (scaled-i)[..., None]
    return np.rint(palette[i]*(1-f)+palette[i+1]*f).astype(np.uint8)


def raster(values, observed, valid, low, high, mixed=None):
    rgb = color(values, low, high, error=mixed is not None)
    rgb[~observed] = UNKNOWN
    rgb[observed & ~valid] = INVALID
    if mixed is not None:
        rgb[valid & mixed] = MIXED
    return Image.fromarray(np.flipud(rgb)).resize((SIDE, SIDE), Image.Resampling.NEAREST)


def load_source(raw, name, reference):
    folder = raw/name
    result = read(folder/'case/result.json')
    mission = result['autonomy']
    if mission['radius'] is not None:
        raise ValueError('Expected uncropped exploration')
    end = mission['started']+mission['elapsed_seconds']
    # Retain compact state and pose only; action histories can be very large.
    states, poses = [], []
    terminal_wall = math.inf
    with (folder/'case/states.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            state = row['state']
            sim = state['motion'].get('diagnostics', {}).get('sim_stamp')
            survey = state.get('autonomy', {})
            states.append(dict(wall=row['wall'], sim=sim, phase=survey.get('phase'), elapsed=survey.get('elapsed_seconds', 0)))
            pose = state['motion'].get('pose')
            if pose:
                poses.append(dict(wall=row['wall'], sim=sim, pose=pose))
            if survey.get('started') == mission['started'] and not survey.get('active') and survey.get('phase') not in (None, 'idle'):
                terminal_wall = min(terminal_wall, row['wall'])
    if not math.isfinite(terminal_wall):
        raise ValueError('No terminal mission state receipt')
    capacity_warning=None
    with (folder/'case/observer/samples.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            event=json.loads(line)
            if event.get('topic')=='/mooncar/map/status' and any('Session capacity reached' in s.get('message','') for s in event.get('status',[])):
                capacity_warning=dict(wall=event['wall'],sim=event.get('sim'));break
    snapshots = []
    map_path=folder/'case/observer/survey-map.jsonl'
    with map_path.open('rb') as stream:
        while True:
            offset=stream.tell();line=stream.readline()
            if not line:break
            row = json.loads(line)
            if row['wall'] <= terminal_wall and row['payload']['stamp'] <= end+1e-6:
                snapshots.append(dict(wall=row['wall'],sim=row['sim'],offset=offset,
                    payload={k:row['payload'][k] for k in ('stamp','frames')},
                    observed_m2=len(row['payload']['cells'])*row['payload']['resolution']**2))
    if not snapshots:
        raise ValueError('No actual reconstruction captured')
    if any(a['wall'] > b['wall'] for a, b in zip(snapshots, snapshots[1:])):
        raise ValueError('Map receipts not monotonic')
    maps, stats = compare(map_payload(dict(map_path=map_path,snapshots=snapshots),-1), reference)
    initial = next((r for r in reversed(snapshots) if r['sim'] is not None and r['sim'] <= mission['started']), None)
    baseline_area = initial['observed_m2'] if initial else None
    alignment = rows(folder/'case/observer/alignment.jsonl')
    errors = [r['xy_error_m'] for r in alignment if mission['started'] <= r['stamp'] <= end]
    truth = [r for r in rows(folder/'case/observer/truth.jsonl') if mission['started'] <= r['stamp'] <= end]
    trail, previous = [], -math.inf
    for row in truth:
        if row['stamp']-previous >= .1:
            trail.append(row['xyz'][:2]); previous = row['stamp']
    stats.update(strategy=name, terrain_strategy='baseline' if name == 'baseline' else 'elevation_neighborhood_v1',
                 experiment=result['experiment'], tested_commit=result['software']['git_commit'],
                 outcome=result['outcome'], reason=mission['reason'], budget_seconds=mission['budget_seconds'],
                 elapsed_seconds=mission['elapsed_seconds'], wall_seconds=result['elapsed_wall_s'],
                 radius=None, terminal_receipt_wall=terminal_wall, snapshot_receipt_wall=snapshots[-1]['wall'],
                 initial_received_observed_m2=baseline_area,
                 new_observed_since_start_m2=stats['total_observed_m2']-baseline_area if baseline_area is not None else None,
                 actions=len(mission['history']), successful_actions=sum(bool(a.get('success')) for a in mission['history']),
                 localization_pauses=len(mission.get('localization_pauses', [])),
                 path_length_m=sum(math.dist(a, b) for a, b in zip(trail, trail[1:])),
                 max_distance_from_start_m=max((math.dist(p, mission['origin']) for p in trail), default=0),
                 localization_xy_p95_m=float(np.percentile(errors, 95)) if errors else None,
                 capture=read(folder/'video/recording.json'),
                 process_exits={k:read(folder/(k+'-exit.json'))['exit_code'] for k in ('driver', 'video', 'backend')})
    return dict(name=name, result=result, mission=mission, snapshots=snapshots,map_path=map_path,capacity_warning=capacity_warning,
                map_walls=[r['wall'] for r in snapshots], poses=poses,
                pose_walls=[r['wall'] for r in poses], states=states,
                state_walls=[r['wall'] for r in states],
                alignment=rows(raw/'rendered'/name/'frame-alignment.jsonl'),
                metadata=read(raw/'rendered'/name/'summary.json'), stats=stats)


def map_payload(source,index):
    header=source['snapshots'][index]
    with source['map_path'].open('rb') as stream:
        stream.seek(header['offset']);row=json.loads(stream.readline())
    if row['wall']!=header['wall'] or row['payload']['stamp']!=header['payload']['stamp']:
        raise ValueError('Reconstruction changed after indexing')
    return row['payload']


def select_map(source, wall, sim):
    index = bisect.bisect_right(source['map_walls'], wall)-1
    while index >= 0:
        row = source['snapshots'][index]
        if row['sim'] is not None and row['sim'] <= sim+1e-6 and row['payload']['stamp'] <= sim+1e-6:
            return index
        index -= 1
    return -1


def main():
    global OUTPUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, required=True)
    parser.add_argument('--output',type=Path,default=OUTPUT)
    parser.add_argument('--crf',type=int,default=21)
    parser.add_argument('--maxrate-kbps',type=int)
    args = parser.parse_args()
    if not 0<=args.crf<=40:raise ValueError('Unsupported video CRF')
    if args.maxrate_kbps is not None and args.maxrate_kbps<100:raise ValueError('Unsupported video bitrate cap')
    OUTPUT=args.output.resolve();OUTPUT.mkdir(parents=True,exist_ok=True)
    reference = build(OUTPUT)
    low, high = -.35, math.ceil(float(reference['z'].max())*10)/10
    sources = [load_source(args.raw, name, reference) for name in ('baseline', 'candidate')]
    final_maps = [compare(map_payload(s,-1), reference)[0] for s in sources]
    common = final_maps[0]['single'] & final_maps[1]['single']
    overlap = dict(common_valid_single_surface_cells=int(common.sum()),
                   common_valid_single_surface_m2=float(common.sum())*.25**2,
                   meaning='Both maps fitted valid heights at exactly the same non-mixed world cells; no registration',
                   strategies=[])
    for source, mapped in zip(sources, final_maps):
        values = mapped['error'][common]
        overlap['strategies'].append(dict(strategy=source['name'],
            rmse_m=float(np.sqrt(np.mean(values**2))) if values.size else None,
            mae_m=float(np.mean(np.abs(values))) if values.size else None,
            p95_abs_m=float(np.percentile(np.abs(values), 95)) if values.size else None))
    write(OUTPUT/'common-area-errors.json', overlap)
    metadata = [s['metadata'] for s in sources]
    fps = metadata[0]['fps']
    origin = min(s['sim_range'][0] for s in metadata)
    leading = [round((s['sim_range'][0]-origin)*fps) for s in metadata]
    count = max(s['frames']+pad for s, pad in zip(metadata, leading))
    trailing = [count-s['frames']-pad for s, pad in zip(metadata, leading)]
    truth_image = raster(reference['z'], np.ones((N, N), bool), np.ones((N, N), bool), low, high)
    cache = [None, None]
    audit = []
    layer = args.raw/'rendered/mapping-overlay.mp4'
    encoder = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(fps), '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-threads', '2', '-preset', 'fast', '-crf', '19', '-pix_fmt', 'yuv420p', str(layer)], stdin=subprocess.PIPE)

    def draw_frame(records, final=False):
        frame = Image.new('RGB', (W, H), BG)
        d = ImageDraw.Draw(frame)
        def text(x, y, value, size=22, fill=WHITE):
            d.text((x, y), value, font=FONTS[size], fill=fill)
        text(28, 12, 'V3 测绘对照  |  同一坐标与比例 · 全场 30 × 30 m · 25 cm 格 · 未测格保留为空', 28)
        titles = ['原策略 · 实测高程', '原策略 · 高度差', '环境真值 · 高程基准', '邻域策略 · 实测高程', '邻域策略 · 高度差']
        positions = [34, 546, 1058, 1570, 2082]
        visuals, summaries = [], []
        for side, (source, record) in enumerate(zip(sources, records)):
            mi = record['map_index']
            if cache[side] is None or cache[side][0] != mi:
                payload = map_payload(source,mi) if mi >= 0 else dict(resolution=.25, frames=0, stamp=0, cells=[])
                maps, stats = compare(payload, reference)
                cache[side] = (mi, raster(maps['z'], maps['observed'], maps['valid'], low, high),
                               raster(maps['error'], maps['observed'], maps['valid'], -.1, .1, reference['mixed']), stats)
            _, zimage, eimage, stats = cache[side]
            visuals.extend([zimage, eimage])
            summaries.append(stats)
        pictures = [visuals[0], visuals[1], truth_image, visuals[2], visuals[3]]
        for i, (x, title, picture) in enumerate(zip(positions, titles, pictures)):
            fill = BLUE if i < 2 else ORANGE if i > 2 else WHITE
            text(x, 63, title, 28, fill)
            frame.paste(picture, (x, 111))
            d.rectangle((x, 111, x+SIDE-1, 111+SIDE-1), outline='#8393a3', width=1)
            for c in (-15, 0, 15):
                px = x+(c+15)/30*(SIDE-1)
                py = 111+(15-c)/30*(SIDE-1)
                text(px-15, 554, str(c), 16, MUTED)
                if i == 0:
                    text(x-30, py-10, str(c), 16, MUTED)
            text(x+SIDE-22, 116, 'N', 20)
            if i in (0, 2, 3):
                gradient = color(np.linspace(low, high, 400)[None, :], low, high)
                frame.paste(Image.fromarray(gradient).resize((SIDE, 13)), (x, 590))
                text(x, 608, f'{low:+.2f} m', 18)
                text(x+SIDE-92, 608, f'{high:+.2f} m', 18)
            else:
                gradient = color(np.linspace(-.1, .1, 400)[None, :], -.1, .1, error=True)
                frame.paste(Image.fromarray(gradient).resize((SIDE, 13)), (x, 590))
                text(x, 608, '≤ −10 cm', 18)
                text(x+SIDE/2-10, 608, '0', 18)
                text(x+SIDE-100, 608, '≥ +10 cm', 18)
        for side, (source, stats) in enumerate(zip(sources, summaries)):
            x = 34 if side == 0 else 1570
            fill = BLUE if side == 0 else ORANGE
            error = stats['single_surface']['rmse_m']
            value = f'{error*100:.2f} cm' if error is not None else '—'
            text(x, 650, f"已观测 {stats['total_observed_m2']:.2f} m²   有效高程 {stats['valid_surface_m2']:.2f} m²", 24, fill)
            text(x, 686, f"单表面高度 RMSE {value}   匹配 {stats['single_surface']['count']} 格", 22)
            record = records[side]
            text(x, 723, f"实际画面 {record['sim']:.2f} s · 测绘源帧 {stats['stamp']:.2f} s", 20, MUTED)
            if record.get('capacity_warning_wall') is not None:
                text(x,751,'测绘会话容量已满，新帧未加入地图',18,'#ff9d85')
            elif source['stats']['outcome'] not in ('passed','budget_completed') and record['capture_wall']>=source['stats']['terminal_receipt_wall']:
                text(x,751,f"本组已提前停车：实际运行 {source['stats']['elapsed_seconds']:.1f} 秒",18,'#ff9d85')
            # Estimated poses only; the environment reference does not correct
            # this path or the reconstructed maps.
            pi = record['pose_index']
            if pi >= 0:
                def point(pose):
                    return (1058+(pose['x']+15)/30*(SIDE-1), 111+(15-pose['y'])/30*(SIDE-1))
                path = [point(p['pose']) for p in source['poses'][:pi+1:3]]
                if len(path) > 1:
                    d.line(path, fill=fill, width=2)
                pose = source['poses'][pi]['pose']
                px, py = point(pose)
                d.ellipse((px-4, py-4, px+4, py+4), fill=fill, outline=WHITE, width=1)
                d.line((px, py, px+11*math.cos(pose['yaw']), py-11*math.sin(pose['yaw'])), fill=fill, width=3)
        text(1058, 650, '真值仅用于离线评价', 24)
        text(1058, 687, '蓝 / 橙：两组各自的估计轨迹', 20, MUTED)
        text(1058, 724, '差值 = 实测高程 − 参考高程', 20, MUTED)
        x = 34
        for rgb, title in ((UNKNOWN, '未观测'), (INVALID, '已观测但拟合无效'), (MIXED, '岩石/地面混合格（误差另计）')):
            d.rectangle((x, 776, x+20, 796), fill=rgb)
            text(x+29, 773, title, 20)
            x += 165 if title == '未观测' else 310 if title == '已观测但拟合无效' else 510
        text(1120, 773, '高程图与导航代价图不同；颜色表示高度，不表示通行风险。', 20, MUTED)
        return frame

    try:
        for index in range(count):
            records = []
            for side, source in enumerate(sources):
                selected = max(0, min(index-leading[side], len(source['alignment'])-1))
                capture = source['alignment'][selected]
                wall, sim = capture['capture_wall'], capture['sim']
                mi = select_map(source, wall, sim)
                pi = bisect.bisect_right(source['pose_walls'], wall)-1
                while pi >= 0 and (source['poses'][pi]['sim'] is None or source['poses'][pi]['sim'] > sim+1e-6):
                    pi -= 1
                snapshot = source['snapshots'][mi] if mi >= 0 else None
                record = dict(source_frame=selected, capture_wall=wall, sim=sim, map_index=mi,
                              capacity_warning_wall=source['capacity_warning']['wall'] if source['capacity_warning'] and source['capacity_warning']['wall']<=wall else None,
                              pose_index=pi, pose_receipt_wall=source['poses'][pi]['wall'] if pi >= 0 else None,
                              pose_sim=source['poses'][pi]['sim'] if pi >= 0 else None,
                              map_receipt_wall=snapshot['wall'] if snapshot else None,
                              map_receipt_sim=snapshot['sim'] if snapshot else None,
                              map_stamp=snapshot['payload']['stamp'] if snapshot else None)
                if snapshot:
                    assert snapshot['wall'] <= wall+1e-6 and snapshot['sim'] <= sim+1e-6 and snapshot['payload']['stamp'] <= sim+1e-6
                records.append(record)
            frame = draw_frame(records)
            if index in (0, count//2, count-1):
                frame.save(OUTPUT/f'mapping-{index:05d}.png')
            if index == count-1:
                frame.save(OUTPUT/'mapping-final.png')
            encoder.stdin.write(frame.tobytes())
            audit.append(dict(frame=index, sides=records))
    finally:
        encoder.stdin.close()
    if encoder.wait(timeout=60):
        raise RuntimeError('Map layer encoder failed')
    videos = [args.raw/'rendered'/name/(name+'.mp4') for name in ('baseline', 'candidate')]
    filters = [f'[{i}:v]setpts=PTS-STARTPTS,tpad=start_mode=clone:start={a}:stop_mode=clone:stop={b}[v{i}]' for i, (a, b) in enumerate(zip(leading, trailing))]
    filters += ['[v0][v1]hstack=inputs=2:shortest=1[top]', '[top][2:v]vstack=inputs=2:shortest=1[out]']
    target = OUTPUT/'v3-exploration-mapping-comparison.mp4'
    rate_control=[] if args.maxrate_kbps is None else ['-maxrate',f'{args.maxrate_kbps}k','-bufsize',f'{2*args.maxrate_kbps}k']
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-i', str(videos[0]), '-i', str(videos[1]), '-i', str(layer), '-filter_complex', ';'.join(filters), '-map', '[out]', '-an', '-c:v', 'libx264', '-threads', '4', '-preset', 'fast', '-crf', str(args.crf), *rate_control, '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(target)], check=True)
    subprocess.run(['ffmpeg', '-v', 'error', '-i', str(target), '-f', 'null', '-'], check=True)
    probe = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries', 'stream=width,height,nb_frames,r_frame_rate:format=duration', '-of', 'json', str(target)]))
    if int(probe['streams'][0]['nb_frames']) != count:
        raise ValueError('Combined video incomplete')
    for name, second in (('start-preview.png', 2), ('comparison-preview.png', count/fps/2), ('end-preview.png', count/fps-1)):
        subprocess.run(['ffmpeg', '-v', 'error', '-y', '-ss', str(second), '-i', str(target), '-frames:v', '1', '-update', '1', str(OUTPUT/name)], check=True)
    (OUTPUT/'overlay-timing.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in audit), encoding='utf-8', newline='\n')
    write(OUTPUT/'summary.json', [s['stats'] for s in sources])
    write(OUTPUT/'video-summary.json', dict(video=target.name, frames=count, fps=fps, seconds=count/fps,
        leading_holds=leading, trailing_holds=trailing, probe=probe, full_decode_exit=0,crf=args.crf,maxrate_kbps=args.maxrate_kbps,
        map_future_data_violations=0, source_metadata=metadata, height_range_m=[low, high], error_range_m=[-.1, .1],
        sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    print(json.dumps(dict(video=str(target), seconds=count/fps, summaries=[s['stats'] for s in sources]), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
