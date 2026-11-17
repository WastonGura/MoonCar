"""Report actual sustained exploration, including stops and second-half gain."""
import argparse
import bisect
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import sys
from datetime import datetime,timedelta,timezone

import numpy as np

OUTPUT=Path(__file__).resolve().parent
LEGACY=OUTPUT.parent/'v3_mapping_comparison_20261117'
sys.path.insert(0,str(LEGACY))
from map_reference import compare


def read(path):return json.loads(path.read_text(encoding='utf-8'))


def write(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')


def rows(path):
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            if line.strip():yield json.loads(line)


def analyze(raw,name,summary,reference,common_seconds):
    folder=raw/name
    mission=read(folder/'case/result.json')['autonomy']
    assert mission['budget_seconds']==480. and mission.get('wall_limit_seconds')==7200.
    start=mission['started'];elapsed=mission['elapsed_seconds'];end=start+elapsed
    states=[];phases=defaultdict(float);unhealthy=0;previous=None
    for row in rows(folder/'case/states.jsonl'):
        motion=row['state']['motion'];diagnostics=motion.get('diagnostics',{})
        stamp=diagnostics.get('sim_stamp')
        if stamp is None:continue
        if stamp<=end:states.append(dict(wall=row['wall'],sim=stamp))
        if start<=stamp<=end:
            health=diagnostics.get('laser_localization',{})
            if health.get('required') and not health.get('healthy'):unhealthy+=1
        if previous is not None and stamp>=previous[0]:
            duration=max(0.,min(stamp,end)-max(previous[0],start))
            phases[previous[1]]+=duration
        previous=(stamp,row['state'].get('autonomy',{}).get('phase','unknown'))
    state_sims=[r['sim'] for r in states]
    if any(a>b for a,b in zip(state_sims,state_sims[1:])):raise ValueError('Clock moved backwards')
    map_path=folder/'case/observer/survey-map.jsonl';maps=[]
    with map_path.open('rb') as stream:
        while True:
            offset=stream.tell();line=stream.readline()
            if not line:break
            row=json.loads(line)
            if row['wall']<=summary['terminal_receipt_wall'] and row['payload']['stamp']<=end+1e-6:
                maps.append(dict(wall=row['wall'],sim=row['sim'],stamp=row['payload']['stamp'],offset=offset))
    map_walls=[r['wall'] for r in maps]
    trajectory=[];previous_stamp=-math.inf
    for row in rows(folder/'case/observer/truth.jsonl'):
        if start<=row['stamp']<=end and row['stamp']-previous_stamp>=.1:
            trajectory.append(dict(stamp=row['stamp'],xy=row['xyz'][:2]));previous_stamp=row['stamp']
    track_stamps=[r['stamp'] for r in trajectory]
    lengths=np.r_[0.,np.cumsum([math.dist(a['xy'],b['xy']) for a,b in zip(trajectory,trajectory[1:])])]
    errors=[(r['stamp'],r['xy_error_m']) for r in rows(folder/'case/observer/alignment.jsonl') if start<=r['stamp']<=end]
    def at(seconds):
        stamp=start+seconds
        si=bisect.bisect_right(state_sims,stamp)-1
        wall=states[si]['wall'] if si>=0 else -math.inf
        if seconds==elapsed:wall=summary['snapshot_receipt_wall']
        mi=bisect.bisect_right(map_walls,wall)-1
        while mi>=0 and (maps[mi]['sim'] is None or
                        (seconds!=elapsed and maps[mi]['sim']>stamp+1e-6) or maps[mi]['stamp']>stamp+1e-6):mi-=1
        payload=dict(resolution=.25,cells=[],frames=0,stamp=0.)
        if mi>=0:
            with map_path.open('rb') as stream:
                stream.seek(maps[mi]['offset']);payload=json.loads(stream.readline())['payload']
        _,quality=compare(payload,reference)
        ti=bisect.bisect_right(track_stamps,stamp)-1
        values=[error for t,error in errors if t<=stamp]
        recent=[error for t,error in errors if max(start,stamp-60)<=t<=stamp]
        return dict(elapsed_seconds=seconds,total_observed_m2=quality['total_observed_m2'],
            valid_surface_m2=quality['valid_surface_m2'],map_stamp=payload['stamp'],
            map_receipt_wall=maps[mi]['wall'] if mi>=0 else None,
            path_length_m=float(lengths[ti]) if ti>=0 else 0.,
            max_distance_from_start_m=max((math.dist(r['xy'],mission['origin']) for r in trajectory[:ti+1]),default=0.),
            localization_xy_p95_m=float(np.percentile(values,95)) if values else None,
            recent_minute_xy_p95_m=float(np.percentile(recent,95)) if recent else None,
            localization_xy_max_m=max(values,default=None),
            height_rmse_m=quality['single_surface']['rmse_m'])
    times=sorted(set([0.,elapsed]+[float(t) for t in range(60,481,60) if t<=elapsed]))
    minute=[at(t) for t in times]
    windows=[]
    for lower,upper in ((0.,240.),(240.,480.)):
        if elapsed<=lower:
            windows.append(dict(start_seconds=lower,end_seconds=upper,reached=False));continue
        actual=min(upper,elapsed);before,after=at(lower),at(actual)
        actions=[a for a in mission['history'] if lower<=a['started_seconds']<actual]
        windows.append(dict(start_seconds=lower,end_seconds=actual,reached=True,
            duration_seconds=actual-lower,new_observed_m2=after['total_observed_m2']-before['total_observed_m2'],
            valid_surface_change_m2=after['valid_surface_m2']-before['valid_surface_m2'],
            path_length_m=after['path_length_m']-before['path_length_m'],
            selected_actions=dict(Counter(a['kind'] for a in actions)),
            successful_actions=sum(bool(a.get('success')) for a in actions),
            grouping='Actions grouped by dispatch time; advance can include rotation'))
    final=minute[-1]
    assert final['map_stamp']==summary['stamp']
    assert final['total_observed_m2']==summary['total_observed_m2']
    memory=[r for r in rows(folder/'case/resources.jsonl') if r.get('sim') is not None and start<=r['sim']<=end and 'localization_rss_kib' in r]
    def median_memory(selected):return float(np.median([r['localization_rss_kib']/1024 for r in selected])) if selected else None
    memory_stats=dict(samples=len(memory),first_minute_median_mib=median_memory([r for r in memory if r['sim']<=start+60]),
        last_minute_median_mib=median_memory([r for r in memory if r['sim']>=end-60]),
        peak_mib=max((r['localization_rss_kib']/1024 for r in memory),default=None),
        scope='RSS of primary localization process only, not whole simulation')
    map_warnings=Counter();first_capacity=None
    for row in rows(folder/'case/observer/samples.jsonl'):
        if row.get('topic')!='/mooncar/map/status':continue
        if row.get('sim') is None or not start<=row['sim']<=end:continue
        for status in row.get('status',[]):
            if status.get('level',0)>0:
                message=status.get('message','');map_warnings[message]+=1
                if first_capacity is None and 'Session capacity reached' in message:
                    first_capacity=dict(sim=row.get('sim'),wall=row['wall'],elapsed_seconds=row['sim']-start if row.get('sim') is not None else None,message=message)
    answer=dict(strategy=name,actual_elapsed_seconds=elapsed,declared_budget_seconds=mission['budget_seconds'],
        declared_wall_limit_seconds=mission.get('wall_limit_seconds'),outcome=summary['outcome'],reason=summary['reason'],
        budget_exhausted_normally=summary['outcome']=='budget_completed',minute_metrics=minute,
        half_windows=windows,phase_seconds=dict(phases),active_unhealthy_snapshots=unhealthy,
        localization_pauses=mission.get('localization_pauses',[]),
        final_selected_action_counts=dict(Counter(a['kind'] for a in mission['history'])),localization_memory=memory_stats,
        common_duration_metrics=at(common_seconds),map_warning_counts=dict(map_warnings),first_capacity_warning=first_capacity,
        final_map_elapsed_seconds=summary['stamp']-start,map_age_at_mission_end_sim_seconds=end-summary['stamp'])
    return answer


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--raw',type=Path,required=True)
    args=parser.parse_args();summaries=read(OUTPUT/'summary.json');reference=np.load(OUTPUT/'reference.npz')
    common_seconds=min(s['elapsed_seconds'] for s in summaries)
    result=[analyze(args.raw,name,summary,reference,common_seconds) for name,summary in zip(('baseline','candidate'),summaries)]
    for name,run in zip(('baseline','candidate'),result):
        captures=rows(args.raw/name/'video/frames.jsonl');first=next(captures);last=first
        for event in captures:last=event
        zone=timezone(timedelta(hours=9))
        run['capture_started_at_jst']=datetime.fromtimestamp(first['wall_epoch_ns']/1e9,zone).isoformat()
        run['capture_ended_at_jst']=datetime.fromtimestamp(last['wall_epoch_ns']/1e9,zone).isoformat()
    write(OUTPUT/'long-run-summary.json',result)
    report=OUTPUT/'README.md';text=report.read_text(encoding='utf-8')
    text=text.replace('# V3 地形自主探测与测绘对照','# V3 八分钟自主探测与测绘对照',1)
    conclusions=[]
    for title,run in zip(('原策略','新策略'),result):
        if run['first_capacity_warning'] is not None:
            event=run['first_capacity_warning']
            conclusions.append(f"{title}实际运行{run['actual_elapsed_seconds']:.1f}秒，但测绘约在第{event['elapsed_seconds']:.1f}秒触及会话容量限制，最后写入地图的源帧为任务第{run['final_map_elapsed_seconds']:.1f}秒，之后未接纳新测绘帧。")
        elif run['outcome']!='budget_completed':
            conclusions.append(f"{title}在第{run['actual_elapsed_seconds']:.1f}秒提前停止：{run['reason']}。")
        else:conclusions.append(f"{title}按预算正常收尾，实际运行{run['actual_elapsed_seconds']:.1f}秒。")
    text=text.replace('\n\n2026-11-17。','\n\n**本轮结果：**'+''.join(conclusions)+'车辆持续运动不等于完成8分钟连续测绘。\n\n2026-11-17。',1)
    extra=['\n## 持续探测记录\n',
           '每分钟数据采用该时刻已收到的真实测绘图。累计面积包含尚未拟合成功的格子；有效高程另列。没有把后续地图回填到早期时刻。\n']
    cleanup_path=args.raw/'orphan-cleanup.json'
    if cleanup_path.exists():
        cleanup=read(cleanup_path);write(OUTPUT/'orphan-cleanup.json',cleanup)
        extra += ['\n### 运行隔离与收尾限制\n',
            '原策略任务结束后，一个属于该实验的 Gazebo 进程未完全退出，直到两组采集完成后才发现并清理，常驻内存约1.1 GiB。PID、启动时刻标识和 Gazebo 分区均已核对，清理后容器仅保留初始化与等待进程。两组 Gazebo 使用不同实验分区。\n',
            '两次收尾采样的该进程累计 CPU 时间均为23分56秒；这不足以证明它在新策略测试期间持续占用CPU，也不能把它认定为耗时变化的原因。实际资源环境未完全清空，因此本轮不作为严格资源隔离下的策略性能A/B结论。会话容量告警、实际停止原因及录屏为本轮直接观测，仍如实保留。原策略结束后的进程清理缺陷尚未在产品代码中修复。证据见 `orphan-cleanup.json`。\n']
    extra.append('\n采集时间（Asia/Tokyo）：原策略 '+result[0]['capture_started_at_jst']+' 至 '+result[0]['capture_ended_at_jst']+'；新策略 '+result[1]['capture_started_at_jst']+' 至 '+result[1]['capture_ended_at_jst']+'。目录按启动日2026-11-17命名，跨日完成。\n')
    extra += [f'\n截至两组共同运行的 {common_seconds:.1f} 仿真秒，面积对比如下，避免把不同持续时间混在一起：\n',
              '| 策略 | 总观测 / m² | 有效高程 / m² | 路程 / m |\n','|---|---:|---:|---:|\n']
    for title,run in zip(('原策略','邻域策略'),result):
        value=run['common_duration_metrics']
        extra.append(f"| {title} | {value['total_observed_m2']:.2f} | {value['valid_surface_m2']:.2f} | {value['path_length_m']:.2f} |\n")
    for name,run in zip(('原策略','邻域策略'),result):
        extra+=['\n### '+name+'\n','| 已运行 / 秒 | 总观测 / m² | 有效高程 / m² | 累计路程 / m | 累计定位 XY P95 / cm | 最近一分钟 P95 / cm |\n',
                '|---:|---:|---:|---:|---:|---:|\n']
        for row in run['minute_metrics']:
            error=f"{row['localization_xy_p95_m']*100:.2f}" if row['localization_xy_p95_m'] is not None else '—'
            recent=f"{row['recent_minute_xy_p95_m']*100:.2f}" if row['recent_minute_xy_p95_m'] is not None else '—'
            extra.append(f"| {row['elapsed_seconds']:.1f} | {row['total_observed_m2']:.2f} | {row['valid_surface_m2']:.2f} | {row['path_length_m']:.2f} | {error} | {recent} |\n")
        extra+=['\n| 时间段 | 实际时长 / 秒 | 新增观测 / m² | 有效高程变化 / m² | 路程 / m |\n','|---|---:|---:|---:|---:|\n']
        for i,window in enumerate(run['half_windows']):
            title='前4分钟' if i==0 else '后4分钟'
            extra.append(f"| {title} | {window['duration_seconds']:.1f} | {window['new_observed_m2']:.2f} | {window['valid_surface_change_m2']:.2f} | {window['path_length_m']:.2f} |\n" if window['reached'] else f'| {title} | 未到达 | — | — | — |\n')
        extra.append(f"\n实际运行 {run['actual_elapsed_seconds']:.1f} / 480 仿真秒；结果 `{run['outcome']}`；原因：{run['reason']}。按源状态统计的定位不健康快照 {run['active_unhealthy_snapshots']} 份，定位暂停 {len(run['localization_pauses'])} 次。单独转向/观察与前进动作计数为 `{run['final_selected_action_counts']}`；前进动作也包含导航中的车体转向，不能解释为完全没有原地转向。\n")
        extra.append('\n定位主进程内存（MiB，RSS）：`'+json.dumps(run['localization_memory'],ensure_ascii=False)+'`。只统计该进程，不代表整个仿真内存。\n')
        if run['first_capacity_warning'] is not None:
            event=run['first_capacity_warning']
            extra.append(f"\n测绘容量告警首次出现在任务第 {event['elapsed_seconds']:.2f} 秒（仿真时钟 {event['sim']:.2f} 秒）：`{event['message']}`。`terrain_map.py` 限制每会话5000帧或500万个原始点；本次最终 {summaries[1 if run['strategy']=='candidate' else 0]['frames']} 帧，触及的是点数容量。质量消息仍可发布旧地图，任务因此继续执行，但新帧没有进入测绘图。不能把本轮车辆完成时间当作持续建图完成时间。\n")
    extra.append('\n后4分钟未完成时只统计实际运行部分，不外推到480秒。一次8分钟预算实验只能检验这一时长与场景下的表现，不能证明任意长时间运行。默认定位和地形策略没有替换，测试期间未增加主动回访。\n')
    report.write_text(text+''.join(extra),encoding='utf-8',newline='\n')
    print(json.dumps([{k:r[k] for k in ('strategy','actual_elapsed_seconds','outcome','half_windows')} for r in result],ensure_ascii=False),flush=True)


if __name__=='__main__':main()
