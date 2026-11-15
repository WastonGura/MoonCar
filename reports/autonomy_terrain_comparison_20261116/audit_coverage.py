"""Audit complete observed maps versus the original start-relative radius metric."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/mooncar_terrain/scripts'))
from terrain_geometry import transform_points


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, required=True)
    args = parser.parse_args()
    output = Path(__file__).resolve().parent
    results = [read(args.raw/name/'case/result.json') for name in ('baseline', 'candidate')]
    common_elapsed = min(r['autonomy']['elapsed_seconds'] for r in results)
    summaries, maps, tracks = [], [], []
    hashes = {}
    for name, result in zip(('baseline', 'candidate'), results):
        survey = result['autonomy']
        research = args.raw/(name+'-sessions')/result['experiment']/'research'
        folder = next(research.glob('autonomy-*/quality-end.json')).parent
        quality, initial = read(folder/'quality-end.json'), read(folder/'quality-start.json')
        resolution = quality['resolution']
        cells = {(c['ix'], c['iy']): c for c in quality['cells']}
        initial_keys = {(c['ix'], c['iy']) for c in initial['cells']}
        def in_world(key):
            return all(-15 <= (v+.5)*resolution <= 15 for v in key)
        def in_radius(key):
            return math.dist(tuple((v+.5)*resolution for v in key), survey['origin']) <= survey['radius']
        full = {k for k in cells if in_world(k)}
        inside = {k for k in full if in_radius(k)}
        archive_keys, matched_keys = set(), set()
        archive = folder/'observations.npz'
        with np.load(archive, allow_pickle=False) as data:
            arrays = {key: data[key] for key in ('stamps', 'points', 'offsets', 'translations', 'quaternions')}
            for i, stamp in enumerate(arrays['stamps']):
                if stamp > quality['stamp']+1e-8:
                    continue
                points = arrays['points'][arrays['offsets'][i]:arrays['offsets'][i+1]]
                world = transform_points(points, arrays['translations'][i], arrays['quaternions'][i])
                keys = {tuple(k) for k in np.unique(np.floor(world[:, :2]/resolution).astype(int), axis=0)}
                archive_keys.update(keys)
                if stamp <= survey['started']+common_elapsed:
                    matched_keys.update(keys)
        if archive_keys != set(cells):
            raise ValueError(f'{name}: archived observations do not reproduce reported coverage')
        truth_path = args.raw/name/'case/observer/truth.jsonl'
        truth = [json.loads(s) for s in truth_path.read_text(encoding='utf-8').splitlines()]
        track = np.array([r['xyz'][:2] for r in truth
                          if survey['started'] <= r['stamp'] <= survey['started']+survey['elapsed_seconds']])
        distances = np.linalg.norm(track-np.asarray(survey['origin']), axis=1)
        summary = dict(strategy=name, experiment=result['experiment'], resolution_m=resolution,
                       world_bounds=[-15, -15, 15, 15], elapsed_seconds=survey['elapsed_seconds'],
                       full_observed_m2=len(full)*resolution**2,
                       inside_start_radius_m2=len(inside)*resolution**2,
                       outside_start_radius_m2=len(full-inside)*resolution**2,
                       newly_observed_since_task_start_m2=len(full-initial_keys)*resolution**2,
                       full_quality_at_least_08_m2=sum(cells[k]['quality'] >= .8 for k in full)*resolution**2,
                       max_distance_from_start_m=float(distances.max()),
                       endpoint_distance_from_start_m=float(distances[-1]),
                       truth_xy_bounds=[track.min(axis=0).tolist(), track.max(axis=0).tolist()],
                       common_elapsed_seconds=common_elapsed,
                       common_elapsed_full_observed_m2=sum(in_world(k) for k in matched_keys)*resolution**2,
                       archive_reproduces_quality_cell_set=True)
        print(json.dumps(summary), flush=True)
        summaries.append(summary)
        maps.append((cells, initial_keys, resolution, survey))
        tracks.append(track[::10])
        for path in (folder/'quality-start.json', folder/'quality-end.json', archive, truth_path):
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024*1024), b''):
                    digest.update(chunk)
            hashes[str(path)] = digest.hexdigest()
    (output/'coverage-audit.json').write_text(json.dumps(dict(
        correction='Previous total-coverage conclusion used only the start-relative 8 m disk.',
        runs=summaries, source_sha256=hashes), indent=2)+'\n', encoding='utf-8', newline='\n')

    image = Image.new('RGB', (1360, 900), '#ffffff')
    draw = ImageDraw.Draw(image)
    font_path = 'C:/Windows/Fonts/msyh.ttc'
    fonts = {size: ImageFont.truetype(font_path, size) for size in (15, 18, 22, 27)}
    def text(x, y, value, size=18, color='#172b42'):
        draw.text((x, y), value, font=fonts[size], fill=color)
    text(30, 16, '寻迹者探测车：完整观测地图与车辆轨迹（同一比例）', 27)
    text(30, 58, '8 m 为车辆活动半径；相机在圈外观测到的地图也应计入总面积。', 18)
    center = np.array(results[0]['autonomy']['origin'])
    extent = max(8., *(abs((np.asarray(k)+.5)*r-center).max()
                       for cells, _, r, _ in maps for k in cells))
    span = 2*math.ceil(extent+1)
    low = center-span/2
    for index, ((cells, initial, resolution, survey), track, summary) in enumerate(zip(maps, tracks, summaries)):
        x0, y0, size = 60+index*675, 155, 590
        scale = size/span
        def point(x, y):
            return x0+(x-low[0])*scale, y0+size-(y-low[1])*scale
        draw.rectangle((x0, y0, x0+size, y0+size), fill='#edf0f3')
        for key in cells:
            x, y = np.asarray(key)*resolution
            a, b = point(x, y+resolution), point(x+resolution, y)
            inside = math.dist((x+resolution/2, y+resolution/2), survey['origin']) <= survey['radius']
            color = '#a7b4c1' if key in initial else '#66ab93' if inside else '#e8b269'
            draw.rectangle((*a, *b), fill=color)
        for value in range(math.ceil(low[0]/2)*2, math.floor((low[0]+span)/2)*2+1, 2):
            px, _ = point(value, 0)
            draw.line((px, y0, px, y0+size), fill='#c8d0d7')
            text(px-9, y0+size+7, str(value), 15)
        for value in range(math.ceil(low[1]/2)*2, math.floor((low[1]+span)/2)*2+1, 2):
            _, py = point(0, value)
            draw.line((x0, py, x0+size, py), fill='#c8d0d7')
            text(x0-32, py-10, str(value), 15)
        cx, cy = point(*survey['origin'])
        radius = survey['radius']*scale
        draw.ellipse((cx-radius, cy-radius, cx+radius, cy+radius), outline='#377cc2', width=2)
        draw.line([point(*p) for p in track], fill='#152e4b', width=4)
        draw.ellipse((cx-5, cy-5, cx+5, cy+5), fill='#ffffff', outline='#152e4b', width=2)
        ex, ey = point(*track[-1])
        draw.rectangle((ex-4, ey-4, ex+4, ey+4), fill='#a02d3b')
        text(x0, 108, '原策略' if index == 0 else '邻域候选策略', 22)
        text(x0+175, 112, f"总观测 {summary['full_observed_m2']:.1f} m²", 18)
        text(x0, 785, f"圈外观测 {summary['outside_start_radius_m2']:.1f} m²", 18)
        text(x0, 817, f"车辆距起点最远 {summary['max_distance_from_start_m']:.2f} m", 18)
    text(30, 865, '灰蓝：初始观测   绿：圈内新增观测   橙：圈外新增观测   深蓝线：车辆真值轨迹   蓝圈：原统计范围', 18)
    image.save(output/'full-map-coverage-comparison.png')


if __name__ == '__main__':
    main()
