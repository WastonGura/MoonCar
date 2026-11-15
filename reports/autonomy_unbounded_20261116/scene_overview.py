"""Offline V2 environment reference; never imported by robot planning or control."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'scripts'))
from experiment_scenes import _v2_geometry, V2_WORLD, PACKAGE, v2_spawn_obstacles

FONT = 'C:/Windows/Fonts/msyh.ttc'


def raster_ground(step=.05):
    triangles, classes, _, _, _ = _v2_geometry()
    axis = np.linspace(-15., 15., round(30/step)+1)
    z = np.full((len(axis), len(axis)), np.nan)
    land = np.zeros_like(z, dtype=np.uint8)
    for triangle, name in zip(triangles, classes):
        low = np.maximum(0, np.ceil((triangle[:, :2].min(axis=0)+15)/step-1e-7).astype(int))
        high = np.minimum(len(axis)-1, np.floor((triangle[:, :2].max(axis=0)+15)/step+1e-7).astype(int))
        if np.any(high < low):
            continue
        a, b, c = triangle
        determinant = (b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
        if abs(determinant) < 1e-12:
            continue
        xx, yy = np.meshgrid(axis[low[0]:high[0]+1], axis[low[1]:high[1]+1])
        u = ((b[1]-c[1])*(xx-c[0])+(c[0]-b[0])*(yy-c[1]))/determinant
        v = ((c[1]-a[1])*(xx-c[0])+(a[0]-c[0])*(yy-c[1]))/determinant
        height = u*a[2]+v*b[2]+(1-u-v)*c[2]
        view = z[low[1]:high[1]+1, low[0]:high[0]+1]
        selected = (u >= -1e-7) & (v >= -1e-7) & (u+v <= 1+1e-7)
        selected &= np.isnan(view) | (height >= view)
        view[selected] = height[selected]
        kind = 1 if 'sand_soft' in name else 2 if 'rubble_gravel' in name else 3 if 'crater' in name else 0
        land[low[1]:high[1]+1, low[0]:high[0]+1][selected] = kind
    if not np.isfinite(z).all():
        raise ValueError('Environment raster contains unsupported holes; do not fill with invented terrain')
    return axis, z, land


def prepare():
    axis, z, land = raster_ground()
    low, high = float(z.min()), float(z.max())
    knots = np.array([low, 0., high])
    colors = np.array([[94, 149, 177], [201, 204, 181], [229, 207, 158]])
    rgb = np.stack([np.interp(z, knots, colors[:, channel]) for channel in range(3)], axis=-1)
    dy, dx = np.gradient(z, axis, axis)
    shade = np.clip((.9-.3*dx+.4*dy)/np.sqrt(1+dx*dx+dy*dy), .55, 1.08)
    rgb *= shade[..., None]
    for kind, color in ((1, [216, 178, 112]), (2, [132, 139, 136])):
        rgb[land == kind] = .55*rgb[land == kind]+.45*np.asarray(color)
    contours = np.floor(z/.05)
    boundary = np.zeros(z.shape, dtype=bool)
    boundary[1:, :] |= contours[1:, :] != contours[:-1, :]
    boundary[:, 1:] |= contours[:, 1:] != contours[:, :-1]
    rgb[boundary] *= .83
    ground = Image.fromarray(np.flipud(np.clip(rgb, 0, 255).astype(np.uint8))).resize((900, 900), Image.Resampling.BICUBIC)
    draw = ImageDraw.Draw(ground)
    def point(x, y):
        return (x+15)*30, (15-y)*30
    for lower, upper in v2_spawn_obstacles(include_markers=False):
        x0, y0 = point(lower[0], upper[1]); x1, y1 = point(upper[0], lower[1])
        draw.rectangle((x0, y0, max(x0+1, x1), max(y0+1, y1)), fill='#384650')
    ground.save(OUTPUT/'environment-map-base.png')
    np.savez_compressed(OUTPUT/'environment-reference.npz', x=axis, y=axis, z=z, land=land)
    metadata = dict(scene='survey_v2_native', bounds=[-15, -15, 15, 15], ground_grid_step_m=.05,
                    ground_min_m=low, ground_max_m=high, actual_collision_triangles=True,
                    scene_sha256=hashlib.sha256(V2_WORLD.read_bytes()).hexdigest(),
                    purpose='Offline environment reference only; not an observed map or navigation input',
                    minor_rock_display='Axis-aligned component bounds; minimum one display pixel')
    (OUTPUT/'environment-metadata.json').write_text(json.dumps(metadata, indent=2)+'\n', encoding='utf-8', newline='\n')
    image = Image.new('RGB', (1540, 1090), '#f8fafb')
    image.paste(ground, (80, 115))
    draw = ImageDraw.Draw(image)
    fonts = {size: ImageFont.truetype(FONT, size) for size in (17, 20, 24, 30)}
    def text(x, y, value, size=20, fill='#203448'):
        draw.text((x, y), value, font=fonts[size], fill=fill)
    text(40, 20, '寻迹者探测车 · V2 仿真环境全场总览', 30)
    text(40, 65, '环境真值参考图｜全场固定坐标｜独立于车辆观测地图', 20)
    for coordinate in range(-15, 16, 5):
        px, py = (coordinate+15)*30+80, (15-coordinate)*30+115
        draw.line((px, 115, px, 1015), fill='#9da9ab', width=1)
        draw.line((80, py, 980, py), fill='#9da9ab', width=1)
        text(px-13, 1022, str(coordinate), 17)
        text(37, py-12, str(coordinate), 17)
    draw.rectangle((80, 115, 980, 1015), outline='#263e50', width=2)
    text(940, 125, 'N', 24)
    sx, sy = 80+(0.5+15)*30, 115+15*30
    draw.ellipse((sx-7, sy-7, sx+7, sy+7), fill='#ffffff', outline='#152e4b', width=3)
    text(sx+10, sy+8, '起点', 17)
    text(1030, 125, '当前场景：30 × 30 m', 24)
    text(1030, 173, '高度着色 + 地形阴影 + 5 cm 等高线', 17)
    text(1030, 211, f'地面高程：{low:.3f} ～ {high:.3f} m', 17)
    for i, (label, color) in enumerate((('一般月壤', '#b5bba4'), ('松软砂地区', '#d8b270'),
                                       ('碎石地区', '#848b88'), ('岩石 / 碎石位置', '#384650'),
                                       ('低洼区域：偏蓝色', '#5e95b1'))):
        yy = 282+i*52
        draw.rectangle((1030, yy+5, 1055, yy+28), fill=color)
        text(1070, yy, label)
    text(1030, 620, '地图复杂度：适中', 24)
    for i, line in enumerate(('保留地形起伏和障碍分布。', '小碎石以位置标记为主。',
                              '不显示真实场景以外的地形。', '不把全场参考图交给车辆。',
                              '对照录像同步显示两组轨迹。')):
        text(1030, 667+i*38, line, 17)
    text(80, 1053, '坐标单位：m；底图来自实际 V2 碰撞网格，岩石标记为组件包围范围。', 17)
    image.save(OUTPUT/'v2-environment-overview.png')
    print(json.dumps(metadata), flush=True)


if __name__ == '__main__':
    prepare()
