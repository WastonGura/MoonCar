"""Combine full footage with a separate fixed environment map and causal pose trails."""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import subprocess

from PIL import Image, ImageDraw, ImageFont

OUTPUT = Path(__file__).resolve().parent
W, H = 2560, 768
COLORS = ('#1876c9', '#ee8532')


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def rows(path):
    return [json.loads(s) for s in path.read_text(encoding='utf-8').splitlines() if s.strip()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, required=True)
    args = parser.parse_args()
    rendered = args.raw/'rendered'
    names = ('baseline', 'candidate')
    metadata = [read(rendered/name/'summary.json') for name in names]
    fps = metadata[0]['fps']
    origin = min(r['sim_range'][0] for r in metadata)
    leading = [round((r['sim_range'][0]-origin)*fps) for r in metadata]
    count = max(r['frames']+pad for r, pad in zip(metadata, leading))
    trailing = [count-r['frames']-pad for r, pad in zip(metadata, leading)]
    sources = []
    for name in names:
        result = read(args.raw/name/'case/result.json')
        assert result['autonomy']['radius'] is None
        states = rows(args.raw/name/'case/states.jsonl')
        poses = [r for r in states if r['state']['motion'].get('pose')]
        research = args.raw/(name+'-sessions')/result['experiment']/'research'
        telemetry = rows(next(research.glob('autonomy-*/telemetry.jsonl')))
        sources.append(dict(result=result, poses=poses, pose_walls=[r['wall'] for r in poses],
            states=states, state_walls=[r['wall'] for r in states], telemetry=telemetry,
            telemetry_sims=[r['sim_stamp'] for r in telemetry],
            alignment=rows(rendered/name/'frame-alignment.jsonl')))
    ground = Image.open(OUTPUT/'environment-map-base.png').convert('RGB').resize((720, 720), Image.Resampling.LANCZOS)
    fonts = {size: ImageFont.truetype('C:/Windows/Fonts/msyh.ttc', size) for size in (17, 22, 28, 34)}
    layer = rendered/'environment-overlay.mp4'
    encoder = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
        '-s', f'{W}x{H}', '-r', str(fps), '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-threads', '2',
        '-preset', 'fast', '-crf', '20', '-pix_fmt', 'yuv420p', str(layer)], stdin=subprocess.PIPE)
    audit = []
    phases = dict(idle='等待启动', planning='选择位置', moving='自主行驶', turning='车体转向',
                  observing='采集视角', budget_reached='时间预算结束', fault='保护停车',
                  no_candidates='暂无安全候选', localization_paused='等待定位恢复')
    def point(x, y):
        return 920+(x+15)*24, 15+(15-y)*24
    try:
        for index in range(count):
            frame = Image.new('RGB', (W, H), '#0b1420')
            frame.paste(ground, (920, 15))
            draw = ImageDraw.Draw(frame)
            def text(x, y, value, size=28, fill='#edf3f8'):
                draw.text((x, y), value, font=fonts[size], fill=fill)
            for coordinate in range(-15, 16, 5):
                px, py = point(coordinate, coordinate)
                draw.line((px, 15, px, 735), fill='#9aa5a1')
                draw.line((920, py, 1640, py), fill='#9aa5a1')
            text(1604, 25, 'N', 28, '#152e4b')
            record = dict(frame=index, sides=[])
            for side, source in enumerate(sources):
                selected = max(0, min(index-leading[side], len(source['alignment'])-1))
                capture = source['alignment'][selected]
                wall, sim = capture['capture_wall'], capture['sim']
                pi = bisect.bisect_right(source['pose_walls'], wall)-1
                si = bisect.bisect_right(source['state_walls'], wall)-1
                ti = bisect.bisect_right(source['telemetry_sims'], sim)-1
                pose = source['poses'][pi]['state']['motion']['pose'] if pi >= 0 else None
                state = source['states'][si]['state'] if si >= 0 else {}
                telemetry = source['telemetry'][ti] if ti >= 0 else None
                path = [point(r['state']['motion']['pose']['x'], r['state']['motion']['pose']['y'])
                        for r in source['poses'][:pi+1:3]]
                if len(path) > 1:
                    draw.line(path, fill=COLORS[side], width=4)
                if pose:
                    px, py = point(pose['x'], pose['y'])
                    draw.ellipse((px-7, py-7, px+7, py+7), fill=COLORS[side], outline='#ffffff', width=2)
                    draw.line((px, py, px+17*math.cos(pose['yaw']), py-17*math.sin(pose['yaw'])), fill=COLORS[side], width=4)
                x = 38 if side == 0 else 1710
                text(x, 32, '原策略' if side == 0 else '邻域候选策略', 34, COLORS[side])
                area = telemetry['observed_area_m2'] if telemetry else None
                gain = area-source['telemetry'][0]['observed_area_m2'] if area is not None else None
                def number(value):
                    return f'{value:.1f}' if value is not None else '—'
                text(x, 113, f'总观测面积  {number(area)} m²', 34)
                text(x, 177, f'任务新增面积  {number(gain)} m²')
                position_origin = source['result']['autonomy']['origin']
                distance = math.dist((pose['x'], pose['y']), position_origin) if pose else None
                text(x, 239, f'当前距起点  {number(distance)} m')
                mission = state.get('autonomy', {})
                text(x, 307, phases.get(mission.get('phase'), mission.get('phase', '准备中')), 28, COLORS[side])
                text(x, 363, f"探测 {mission.get('elapsed_seconds', 0):.1f} / 180 仿真秒", 22)
                text(x, 448, '不设置起点半径；保留场地边界', 22)
                text(x, 491, '总面积：全部实际观测格去重累计', 22)
                text(x, 534, '新增面积：扣除任务开始时的基线', 22)
                text(x, 626, '底图是环境参考，不是车辆已观测图', 22, '#a8b7c6')
                if pi >= 0:
                    assert source['poses'][pi]['wall'] <= wall+1e-6
                if telemetry:
                    assert telemetry['sim_stamp'] <= sim+1e-6
                record['sides'].append(dict(source_frame=selected, capture_wall=wall, sim=sim,
                    pose_wall=source['poses'][pi]['wall'] if pi >= 0 else None,
                    telemetry_sim=telemetry['sim_stamp'] if telemetry else None))
            text(920, 742, '固定全场 30 × 30 m · 蓝：原策略 · 橙：候选策略', 17)
            audit.append(record)
            if index == count-1:
                frame.save(OUTPUT/'environment-final-tracks.png')
            encoder.stdin.write(frame.tobytes())
    finally:
        encoder.stdin.close()
    if encoder.wait(timeout=60):
        raise RuntimeError('Environment layer encoding failed')
    videos = [rendered/'baseline/baseline.mp4', rendered/'candidate/candidate.mp4']
    filters = [f'[{i}:v]setpts=PTS-STARTPTS,tpad=start_mode=clone:start={a}:stop_mode=clone:stop={b}[v{i}]'
               for i, (a, b) in enumerate(zip(leading, trailing))]
    filters.append('[v0][v1]hstack=inputs=2:shortest=1[top]')
    filters.append('[top][2:v]vstack=inputs=2:shortest=1[out]')
    output = OUTPUT/'v2-unbounded-comparison.mp4'
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-i', str(videos[0]), '-i', str(videos[1]),
        '-i', str(layer), '-filter_complex', ';'.join(filters), '-map', '[out]', '-an',
        '-c:v', 'libx264', '-threads', '4', '-preset', 'fast', '-crf', '21', '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart', str(output)], check=True)
    decoded = subprocess.run(['ffmpeg', '-v', 'error', '-i', str(output), '-f', 'null', '-'],
                             capture_output=True, text=True, check=True)
    if decoded.stderr.strip():
        raise RuntimeError(decoded.stderr)
    probe = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-select_streams', 'v:0',
        '-show_entries', 'stream=width,height,nb_frames,r_frame_rate:format=duration', '-of', 'json', str(output)]))
    assert int(probe['streams'][0]['nb_frames']) == count
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-ss', str(count/fps/2), '-i', str(output),
                    '-frames:v', '1', '-update', '1', str(OUTPUT/'comparison-preview.png')], check=True)
    digest = hashlib.sha256()
    with output.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    (OUTPUT/'overlay-timing.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in audit), encoding='utf-8', newline='\n')
    summary = dict(video=output.name, frames=count, fps=fps, seconds=count/fps, leading_holds=leading,
                   trailing_holds=trailing, source_metadata=metadata, complete_decode_exit=0,
                   overview_future_pose_or_telemetry_violations=0, probe=probe, sha256=digest.hexdigest(),
                   map_policy='Full environment reference plus pose trails received by displayed capture; telemetry sampled no later than displayed sim time')
    (OUTPUT/'video-summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps({k: v for k, v in summary.items() if k != 'source_metadata'}), flush=True)


if __name__ == '__main__':
    main()
