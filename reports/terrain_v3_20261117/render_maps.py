"""Plot actual V2/V3 ground with common natural texture/shading, without contours."""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
PACKAGE=ROOT/'ros2_ws/src/mooncar_gazebo'
FONT='C:/Windows/Fonts/msyh.ttc'


def terrain_image(version,size):
    if version=='v2':
        with np.load(ROOT/'reports/autonomy_unbounded_20261116/environment-reference.npz') as data:
            axis,z=data['x'],data['z']
    else:
        with np.load(PACKAGE/'models/terrain_v3/survey/height_grid.npz') as data:
            axis,z=data['x'],data['z']
    gy,gx=np.gradient(z,axis,axis)
    lighting=np.clip((.91-.45*gx+.55*gy)/np.sqrt(1+gx*gx+gy*gy),.45,1.12)
    light=Image.fromarray(np.flipud(lighting).astype(np.float32)).resize((size,size),Image.Resampling.BICUBIC)
    atlas='survey_surface_albedo.png' if version=='v2' else 'survey_v3_surface_albedo.png'
    texture=Image.open(PACKAGE/'materials/textures'/atlas).convert('RGB').resize((size,size),Image.Resampling.LANCZOS)
    pixels=np.asarray(texture).astype(float)*np.asarray(light)[...,None]*1.7
    result=Image.fromarray(np.clip(pixels,0,255).astype(np.uint8))
    draw=ImageDraw.Draw(result)
    manifest=json.loads((PACKAGE/f'models/terrain_{version}/survey/manifest.json').read_text())
    for rock in manifest['boulders']:
        x,y,_=rock['position_m'];rx,ry,_=rock['radii_m']
        px,py=(x+15)*size/30,(15-y)*size/30
        draw.ellipse((px-rx*size/30,py-ry*size/30,px+rx*size/30,py+ry*size/30),fill='#42474b',outline='#30383d')
    pebbles=manifest['pebbles']['instances'] if version=='v2' else manifest['pebbles']
    for rock in pebbles:
        x,y,_=rock['position_m'];px,py=(x+15)*size/30,(15-y)*size/30
        radius=max(.6,rock['diameter_m']*size/60)
        draw.ellipse((px-radius,py-radius,px+radius,py+radius),fill='#51575a')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',type=Path)
    args=parser.parse_args()
    fonts={s:ImageFont.truetype(FONT,s) for s in (17,20,24,30)}
    image=Image.new('RGB',(1570,960),'#f5f7f8');draw=ImageDraw.Draw(image)
    def text(x,y,value,size=20,fill='#203448'):
        draw.text((x,y),value,font=fonts[size],fill=fill)
    text(30,20,'寻迹者探测车 · 地形重建前后对照',30)
    text(30,65,'同一显示方式：实际地面纹理 + 地形阴影；不叠加等高线或材质硬切色块。',20)
    for i,version in enumerate(('v2','v3')):
        left,top,size=65+i*775,155,650
        image.paste(terrain_image(version,size),(left,top));draw=ImageDraw.Draw(image)
        text(left,112,'V2：窄带拼接地形' if i==0 else 'V3：整体连续地形',24)
        for value in range(-15,16,10):
            px=left+(value+15)*size/30;py=top+(15-value)*size/30
            draw.line((px,top,px,top+size),fill='#a9afb0')
            draw.line((left,py,left+size,py),fill='#a9afb0')
            text(px-12,top+size+8,str(value),17)
            text(left-32,py-10,str(value),17)
        draw.rectangle((left,top,left+size,top+size),outline='#40505a',width=2)
        text(left+size-30,top+10,'N',24)
        px=left+(3.183333333+15)*size/30;py=top+(15+7.883333333)*size/30
        color='#aa4240' if i==0 else '#2a7b65'
        draw.line((px-7,py-7,px+7,py+7),fill=color,width=3)
        draw.line((px-7,py+7,px+7,py-7),fill=color,width=3)
        text(left,858,'旧人工陡坡位置：49.8°' if i==0 else '相同位置：6.2°',24,color)
    text(30,915,'V3 保留实测区域的原始起伏与比例，周围地面共同衔接；旧 V2 完整保留。坐标单位：m。',20)
    image.save(OUT/'terrain-v2-v3-comparison.png')

    image=Image.new('RGB',(1280,1190),'#f5f7f8');image.paste(terrain_image('v3',1000),(95,120))
    draw=ImageDraw.Draw(image)
    text(35,20,'寻迹者探测车 · Terrain V3 全场地图',30)
    text(35,67,'30 × 30 m 连续地形 · 环境参考图，未提供给车辆规划或定位',20)
    for value in range(-15,16,5):
        px=95+(value+15)*1000/30;py=120+(15-value)*1000/30
        draw.line((px,120,px,1120),fill='#a9afb0')
        draw.line((95,py,1095,py),fill='#a9afb0')
        text(px-12,1127,str(value),17);text(48,py-10,str(value),17)
    text(1050,135,'N',24)
    px=95+15.5*1000/30;py=120+15*1000/30
    draw.ellipse((px-6,py-6,px+6,py+6),fill='white',outline='#1a4058',width=2)
    text(px+12,py+6,'默认起点',17)
    if args.trial:
        truth=[json.loads(s) for s in (args.trial/'case/observer/truth.jsonl').read_text().splitlines() if s]
        path=[(95+(r['xyz'][0]+15)*1000/30,120+(15-r['xyz'][1])*1000/30) for r in truth[::10]]
        if len(path)>1:draw.line(path,fill='#2376bd',width=4)
        text(95,1160,'蓝线：本次车辆实际通行轨迹（离线真值记录）。环境几何与观测地图分开。',17)
    else:text(95,1160,'保留陨坑、缓丘、砂地和岩石；灰度纹理与阴影表示实际地形，不使用密集等高线。',17)
    image.save(OUT/'terrain-v3-overview.png')


if __name__=='__main__':main()
