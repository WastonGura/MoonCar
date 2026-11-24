#!/usr/bin/env python3
"""Render individual 4-way Gazebo clips and compose a synchronized 2560x1260 4-way comparison MP4.

Layout (2560 x 1260):
  - Top strip (2560 x 536): 4 Gazebo 3D + minimap clips at exact 1/2 scale (640 x 536 each)
  - Bottom strip (2560 x 724): 4 causal live elevation maps + 1 V3 truth map with 4 overlaid trails
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
REPORTS = HERE.parent
sys.path.insert(0, str(REPORTS / 'v3_mapping_comparison_20261117'))

from map_reference import build as build_reference, compare as compare_map, N  # noqa: E402
from analyze_4way import GROUPS  # noqa: E402

BG, WHITE, MUTED = '#0b1420', '#edf3f8', '#a8b7c6'
UNKNOWN, INVALID = (30, 42, 57), (224, 164, 49)
W, H_BOTTOM, SIDE = 2560, 724, 420
TILE_W, TILE_H = 640, 536
HEIGHT_COLORS = np.array(
    [[33, 73, 126], [55, 145, 170], [116, 187, 153], [226, 217, 149], [205, 146, 96], [241, 232, 219]],
    dtype=float,
)


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def rows(path):
    with path.open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8', newline='\n')


def find_font(explicit=None):
    candidates = [explicit] if explicit else []
    candidates += [
        Path('C:/Windows/Fonts/msyh.ttc'),
        Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'),
        Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'),
    ]
    for path in candidates:
        if path and Path(path).is_file():
            return Path(path)
    raise FileNotFoundError('No usable TTF/TTC font found')


def color(values, low, high):
    t = np.clip((np.nan_to_num(values, nan=low) - low) / (high - low), 0.0, 1.0)
    scaled = t * (len(HEIGHT_COLORS) - 1)
    i = np.minimum(np.floor(scaled).astype(int), len(HEIGHT_COLORS) - 2)
    f = (scaled - i)[..., None]
    return np.rint(HEIGHT_COLORS[i] * (1.0 - f) + HEIGHT_COLORS[i + 1] * f).astype(np.uint8)


def raster(values, observed, valid, low, high):
    rgb = color(values, low, high)
    rgb[~observed] = UNKNOWN
    rgb[observed & ~valid] = INVALID
    return Image.fromarray(np.flipud(rgb)).resize((SIDE, SIDE), Image.Resampling.NEAREST)


def map_payload(source, index):
    header = source['snapshots'][index]
    with source['map_path'].open('rb') as stream:
        stream.seek(header['offset'])
        row = json.loads(stream.readline())
    if row['wall'] != header['wall'] or row['payload']['stamp'] != header['payload']['stamp']:
        raise ValueError('Reconstruction changed after indexing')
    return row['payload']


def select_map(source, wall, sim):
    idx = bisect.bisect_right(source['map_walls'], wall) - 1
    while idx >= 0:
        row = source['snapshots'][idx]
        if row['sim'] is not None and row['sim'] <= sim + 1e-6 and row['payload']['stamp'] <= sim + 1e-6:
            return idx
        idx -= 1
    return -1


def load_source(raw, group, reference):
    name = group['key']
    folder = raw / name
    result = read(folder / 'case/result.json')
    mission = result['autonomy']
    end = mission['started'] + mission['elapsed_seconds']
    poses = []
    terminal_wall = math.inf
    with (folder / 'case/states.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            if not line.strip():
                continue
            row = json.loads(line)
            state = row['state']
            sim = state['motion'].get('diagnostics', {}).get('sim_stamp')
            survey = state.get('autonomy', {})
            pose = state['motion'].get('pose')
            if pose:
                poses.append(dict(wall=row['wall'], sim=sim, pose=pose))
            if (survey.get('started') == mission['started']
                    and not survey.get('active')
                    and survey.get('phase') not in (None, 'idle')):
                terminal_wall = min(terminal_wall, row['wall'])
    if not math.isfinite(terminal_wall):
        raise ValueError(f'{name}: no terminal receipt wall')

    snapshots = []
    map_path = folder / 'case/observer/survey-map.jsonl'
    with map_path.open('rb') as stream:
        while True:
            offset = stream.tell()
            line = stream.readline()
            if not line:
                break
            row = json.loads(line)
            if row['wall'] <= terminal_wall and row['payload']['stamp'] <= end + 1e-6:
                snapshots.append(dict(
                    wall=row['wall'], sim=row['sim'], offset=offset,
                    payload={k: row['payload'][k] for k in ('stamp', 'frames')},
                ))
    if not snapshots:
        raise ValueError(f'{name}: no reconstruction snapshots')

    rendered_dir = raw / 'rendered' / name
    return dict(
        name=name,
        label=group['label'],
        color=group['color'],
        result=result,
        mission=mission,
        terminal_wall=terminal_wall,
        snapshots=snapshots,
        map_path=map_path,
        map_walls=[r['wall'] for r in snapshots],
        poses=poses,
        pose_walls=[r['wall'] for r in poses],
        alignment=rows(rendered_dir / 'frame-alignment.jsonl'),
        metadata=read(rendered_dir / 'summary.json'),
        video_path=rendered_dir / f'{name}.mp4',
    )


def ensure_rendered_clips(raw, fps, font_path, skip_existing=True):
    render_script = REPORTS / 'turn_minimap_20261111' / 'render_video.py'
    for group in GROUPS:
        name = group['key']
        out_dir = raw / 'rendered' / name
        mp4 = out_dir / f'{name}.mp4'
        if skip_existing and mp4.is_file() and (out_dir / 'frame-alignment.jsonl').is_file() and (out_dir / 'summary.json').is_file():
            continue
        cmd = [
            sys.executable, '-B', str(render_script),
            '--trial', str(raw / name),
            '--output', str(out_dir),
            '--autonomy',
            '--label', group['label'],
            '--filename', f'{name}.mp4',
            '--scene-label', 'V3',
            '--fps', str(fps),
            '--font', str(font_path),
        ]
        subprocess.run(cmd, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fps', type=int, default=12)
    parser.add_argument('--crf', type=int, default=22)
    parser.add_argument('--font', type=Path, default=None)
    parser.add_argument('--force-rerender', action='store_true')
    args = parser.parse_args()

    out_dir = args.output.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    font_path = find_font(args.font)
    fonts = {sz: ImageFont.truetype(str(font_path), sz) for sz in (15, 17, 19, 21, 24, 26)}

    ensure_rendered_clips(args.raw, args.fps, font_path, skip_existing=not args.force_rerender)

    reference = build_reference(out_dir)
    low, high = -0.35, math.ceil(float(reference['z'].max()) * 10.0) / 10.0
    sources = [load_source(args.raw, g, reference) for g in GROUPS]

    fps = sources[0]['metadata']['fps']
    starts = [s['metadata']['sim_range'][0] - s['mission']['started'] for s in sources]
    origin = min(starts)
    leading = [round((st - origin) * fps) for st in starts]
    count = max(s['metadata']['frames'] + pad for s, pad in zip(sources, leading))
    trailing = [count - s['metadata']['frames'] - pad for s, pad in zip(sources, leading)]

    truth_img = raster(reference['z'], np.ones((N, N), bool), np.ones((N, N), bool), low, high)
    cache = [None] * len(sources)
    layer_path = args.raw / 'rendered' / 'mapping-overlay-4way.mp4'
    encoder = subprocess.Popen(
        [
            'ffmpeg', '-v', 'error', '-y',
            '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H_BOTTOM}',
            '-r', str(fps), '-i', 'pipe:0', '-an',
            '-c:v', 'libx264', '-threads', '2', '-preset', 'fast', '-crf', '20',
            '-pix_fmt', 'yuv420p', str(layer_path),
        ],
        stdin=subprocess.PIPE,
    )

    xs = [36, 540, 1044, 1548, 2096]
    map_y = 92

    def draw_bottom(records):
        frame = Image.new('RGB', (W, H_BOTTOM), BG)
        d = ImageDraw.Draw(frame)

        def txt(x, y, s, sz=20, fill=WHITE):
            d.text((x, y), s, font=fonts[sz], fill=fill)

        txt(36, 10, 'MoonCar R5 四组 8 分钟 V3 地形自主探索对比  |  全场 30 × 30 m · 25 cm 栅格 · 严格因果地图同步', 26)

        for idx, (src, rec) in enumerate(zip(sources, records)):
            mi = rec['map_index']
            if cache[idx] is None or cache[idx][0] != mi:
                payload = map_payload(src, mi) if mi >= 0 else dict(resolution=0.25, frames=0, stamp=0.0, cells=[])
                maps, stats = compare_map(payload, reference)
                cache[idx] = (mi, raster(maps['z'], maps['observed'], maps['valid'], low, high), stats)
            _, zimg, stats = cache[idx]
            x = xs[idx]
            txt(x, 54, f'组{idx + 1}: {src["label"]}', 24, src['color'])
            frame.paste(zimg, (x, map_y))
            d.rectangle((x, map_y, x + SIDE - 1, map_y + SIDE - 1), outline=src['color'], width=2)
            txt(x + SIDE - 22, map_y + 4, 'N', 17)
            for c in (-15, 0, 15):
                px = x + (c + 15.0) / 30.0 * (SIDE - 1)
                txt(px - 12, map_y + SIDE + 4, str(c), 15, MUTED)
            rmse = stats['single_surface']['rmse_m']
            rmse_s = f'{rmse * 100:.2f} cm' if rmse is not None else '—'
            elapsed = max(0.0, rec['sim'] - src['mission']['started'])
            txt(x, map_y + SIDE + 28, f"观测 {stats['total_observed_m2']:.1f} m²  有效 {stats['valid_surface_m2']:.1f} m²", 21, src['color'])
            txt(x, map_y + SIDE + 58, f"高程 RMSE {rmse_s} · 任务 {elapsed:.1f} s", 19, WHITE)
            if src['result']['outcome'] not in ('passed', 'budget_completed') and rec['capture_wall'] >= src['terminal_wall']:
                txt(x, map_y + SIDE + 86, f"提前结束 ({src['mission']['elapsed_seconds']:.1f}s)", 17, '#ff9d85')

        # Rightmost panel: Truth elevation + all 4 trajectories
        tx = xs[4]
        txt(tx, 54, 'V3 真值高程 + 四组轨迹', 24, WHITE)
        frame.paste(truth_img, (tx, map_y))
        d.rectangle((tx, map_y, tx + SIDE - 1, map_y + SIDE - 1), outline='#8393a3', width=2)
        txt(tx + SIDE - 22, map_y + 4, 'N', 17)
        for src, rec in zip(sources, records):
            pi = rec['pose_index']
            if pi < 0:
                continue
            def pt(pose):
                return (tx + (pose['x'] + 15.0) / 30.0 * (SIDE - 1), map_y + (15.0 - pose['y']) / 30.0 * (SIDE - 1))
            path = [pt(p['pose']) for p in src['poses'][:pi + 1:3]]
            if len(path) > 1:
                d.line(path, fill=src['color'], width=3)
            pose = src['poses'][pi]['pose']
            px, py = pt(pose)
            d.ellipse((px - 5, py - 5, px + 5, py + 5), fill=src['color'], outline=WHITE, width=1)
            d.line((px, py, px + 11 * math.cos(pose['yaw']), py - 11 * math.sin(pose['yaw'])), fill=src['color'], width=3)

        # Colorbar under truth panel
        grad = color(np.linspace(low, high, 400)[None, :], low, high)
        frame.paste(Image.fromarray(grad).resize((SIDE, 14)), (tx, map_y + SIDE + 30))
        txt(tx, map_y + SIDE + 48, f'{low:+.2f} m', 17, MUTED)
        txt(tx + SIDE - 78, map_y + SIDE + 48, f'{high:+.2f} m', 17, MUTED)
        txt(tx, map_y + SIDE + 76, '真值仅用于离线评价；未测格留暗灰', 17, MUTED)

        # Bottom legend strip
        ly = H_BOTTOM - 36
        lx = 36
        for rgb, desc in ((UNKNOWN, '未观测'), (INVALID, '已观测但拟合无效')):
            d.rectangle((lx, ly + 3, lx + 20, ly + 21), fill=rgb)
            txt(lx + 28, ly, desc, 19)
            lx += 160 if desc == '未观测' else 260
        for idx, src in enumerate(sources):
            d.rectangle((lx, ly + 5, lx + 22, ly + 19), fill=src['color'])
            txt(lx + 28, ly, f'组{idx + 1} {src["label"]}', 19, src['color'])
            lx += 270
        return frame

    try:
        for index in range(count):
            records = []
            for src, pad in zip(sources, leading):
                sel = max(0, min(index - pad, len(src['alignment']) - 1))
                cap = src['alignment'][sel]
                wall, sim = cap['capture_wall'], cap['sim']
                mi = select_map(src, wall, sim)
                pi = bisect.bisect_right(src['pose_walls'], wall) - 1
                while pi >= 0 and (src['poses'][pi]['sim'] is None or src['poses'][pi]['sim'] > sim + 1e-6):
                    pi -= 1
                if mi >= 0:
                    snap = src['snapshots'][mi]
                    assert snap['wall'] <= wall + 1e-6 and snap['sim'] <= sim + 1e-6 and snap['payload']['stamp'] <= sim + 1e-6
                records.append(dict(source_frame=sel, capture_wall=wall, sim=sim, map_index=mi, pose_index=pi))
            img = draw_bottom(records)
            if index in (0, count // 2, count - 1):
                img.save(out_dir / f'mapping-4way-{index:05d}.png')
            if index == count - 1:
                img.save(out_dir / 'mapping-4way-final.png')
            encoder.stdin.write(img.tobytes())
    finally:
        encoder.stdin.close()
    if encoder.wait(timeout=120):
        raise RuntimeError('Bottom mapping overlay encoder failed')

    target = out_dir / 'v4-4way-comparison.mp4'
    filters = []
    for i, (lead, trail) in enumerate(zip(leading, trailing)):
        filters.append(
            f'[{i}:v]setpts=PTS-STARTPTS,scale={TILE_W}:{TILE_H},'
            f'tpad=start_mode=clone:start={lead}:stop_mode=clone:stop={trail}[v{i}]'
        )
    filters.append('[v0][v1][v2][v3]hstack=inputs=4:shortest=1[top]')
    filters.append('[top][4:v]vstack=inputs=2:shortest=1[out]')

    cmd = ['ffmpeg', '-v', 'error', '-y']
    for src in sources:
        cmd += ['-i', str(src['video_path'])]
    cmd += [
        '-i', str(layer_path),
        '-filter_complex', ';'.join(filters),
        '-map', '[out]', '-an',
        '-c:v', 'libx264', '-threads', '4', '-preset', 'fast', '-crf', str(args.crf),
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(target),
    ]
    subprocess.run(cmd, check=True)
    subprocess.run(['ffmpeg', '-v', 'error', '-i', str(target), '-f', 'null', '-'], check=True)
    probe = json.loads(subprocess.check_output([
        'ffprobe', '-v', 'error', '-select_streams', 'v:0',
        '-show_entries', 'stream=width,height,nb_frames,r_frame_rate:format=duration',
        '-of', 'json', str(target),
    ]))

    for pname, sec in (('start-preview.png', 2), ('mid-preview.png', count / fps / 2.0), ('end-preview.png', max(1.0, count / fps - 1.0))):
        subprocess.run(['ffmpeg', '-v', 'error', '-y', '-ss', str(sec), '-i', str(target), '-frames:v', '1', '-update', '1', str(out_dir / pname)], check=True)

    summary = dict(
        video=target.name,
        frames=count,
        fps=fps,
        seconds=count / fps,
        leading_holds=leading,
        trailing_holds=trailing,
        probe=probe,
        full_decode_exit=0,
        map_future_data_violations=0,
        height_range_m=[low, high],
        sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
    )
    write(out_dir / 'video-summary.json', summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()
