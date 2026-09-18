"""Freeze all declared trials and plot descriptive comparisons without dropping failures."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import statistics
import zipfile
from survey_evaluation import write_json,write_csv


def report(source,output):
    plan=json.loads((source/'plan.json').read_text());trials=json.loads((source/'trials.json').read_text())
    if len(trials)!=len(plan['trials']) or any(t['status']=='starting' for t in trials):
        raise ValueError('Declared trials have not all reached a terminal state')
    output.mkdir(parents=True,exist_ok=False);rows=[];core_hashes=[];predictions=[]
    important=['scripts/autonomous_survey.py','scripts/experiment_motion.py','scripts/survey_evaluation.py',
               'scripts/simulation_experiment.py','containers/experiment.launch.py',
               'ros2_ws/src/mooncar_terrain/scripts/terrain_map.py','ros2_ws/src/mooncar_terrain/scripts/map_recorder.py',
               'ros2_ws/src/mooncar_terrain/scripts/terrain_node.py','ros2_ws/src/mooncar_terrain/scripts/terrain_geometry.py',
               'ros2_ws/src/mooncar_control/scripts/cmd_vel_safety_relay.py']
    for trial in trials:
        row={k:v for k,v in trial.items() if k not in ('summary','folder')}
        row.update(trial.get('summary',{}));rows.append(row)
        if 'folder' not in trial:continue
        folder=Path(trial['folder']);target=output/'runs'/f"{trial['index']:02d}-{folder.name}"
        if not folder.exists():
            row['evidence_error']='Evidence directory missing';continue
        hashes=json.loads((folder/'hashes.json').read_text()) if (folder/'hashes.json').exists() else {}
        if any(hashlib.sha256((folder/name).read_bytes()).hexdigest()!=value for name,value in hashes.items()):
            raise ValueError(f'Frozen report checksum failed: {folder}')
        shutil.copytree(folder,target)
        log=folder.parent.parent/'simulation.log'
        if log.exists():shutil.copyfile(log,target/'simulation.log')
        row['evidence']=str(target.relative_to(output))
        try:
            with zipfile.ZipFile(folder/'source.zip') as archive:
                core_hashes.append({name:hashlib.sha256(archive.read(name)).hexdigest() for name in important})
        except (OSError,KeyError,zipfile.BadZipFile) as error:row['source_error']=str(error)
        mission=json.loads((folder/'mission.json').read_text()) if (folder/'mission.json').exists() else {}
        row['actions_with_resampling_utility']=sum(h.get('expected_improvement_m2',0)>0 for h in mission.get('history',[]))
        row['actions_failed']=sum(h.get('outcome')=='failed' for h in mission.get('history',[]))
        row['actions_interrupted']=sum(h.get('outcome')=='interrupted' for h in mission.get('history',[]))
        for index,action in enumerate(mission.get('history',[]),1):
            actual=action.get('predicted_view_gain',{})
            predictions.append(dict(trial=trial['index'],strategy=trial['strategy'],action=index,
                 outcome=action.get('outcome'),predicted_new_m2=action.get('expected_new_m2'),
                 actual_new_m2=actual.get('new_observed_m2'),predicted_resampling_utility_m2=action.get('expected_improvement_m2'),
                 actual_recovered_surface_m2=actual.get('recovered_surface_m2')))
    matched=bool(core_hashes) and len(core_hashes)==sum('folder' in t for t in trials) and all(h==core_hashes[0] for h in core_hashes)
    write_json(output/'core-source-check.json',dict(all_recorded_core_sources_equal=matched,per_run=core_hashes))
    shutil.copyfile(source/'plan.json',output/'plan.json');write_json(output/'trials.json',trials)
    write_csv(output/'comparison.csv',rows)
    write_csv(output/'prediction-errors.csv',predictions)
    groups=[]
    for start in dict.fromkeys(tuple(r['start']) for r in rows):
        for strategy in dict.fromkeys(r['strategy'] for r in rows):
            selected=[r for r in rows if tuple(r['start'])==start and r['strategy']==strategy]
            group=dict(start=list(start),strategy=strategy,declared_runs=len(selected),
                       reported_runs=sum('new_observed_area_m2' in r for r in selected))
            for key in ('new_observed_area_m2','valid_surface_gain_m2','resampling_success_fraction','path_m','failed_actions'):
                values=[r[key] for r in selected if r.get(key) is not None]
                group[key+'_mean']=statistics.mean(values) if values else None
                group[key+'_min']=min(values) if values else None;group[key+'_max']=max(values) if values else None
            groups.append(group)
    write_csv(output/'grouped.csv',groups)
    groups=[g for g in groups if g['reported_runs']]
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names={'nearest_view_v1':'Nearest','coverage_view_v1':'Coverage','surface_view_v1':'Surface',
           'nearest_view_v2':'Nearest v2','coverage_view_v2':'Coverage v2','surface_view_v2':'Surface v2',
           'coverage_view_v3':'Coverage v3','surface_view_v3':'Surface v3'}
    fig,axes=plt.subplots(2,2,figsize=(11,8),layout='constrained')
    keys=('new_observed_area_m2','valid_surface_gain_m2','resampling_success_fraction','path_m')
    titles=('New observed projected area (m2)','Net valid fitted surface (m2)','Initial deficient cells recovered (fraction)','Sampled localization path (m)')
    positions=list(range(len(groups)));labels=[f"{names[g['strategy']]}\n({g['start'][0]}, {g['start'][1]})" for g in groups]
    for ax,key,title in zip(axes.flat,keys,titles):
        for i,g in enumerate(groups):
            value=g[key+'_mean']
            if value is None:continue
            ax.bar(i,value,color='white',edgecolor='black',hatch=('', '//', '..')[i%3])
            ax.plot([i,i],[g[key+'_min'],g[key+'_max']],'k-',linewidth=1.5)
            selected=[r[key] for r in rows if tuple(r['start'])==tuple(g['start']) and r['strategy']==g['strategy'] and r.get(key) is not None]
            ax.plot([i]*len(selected),selected,'ko',markersize=4)
        ax.set_xticks(positions,labels,fontsize=8);ax.set_title(title,fontsize=10)
    fig.supxlabel('Dots: individual trials; bars: descriptive mean; lines: min/max, not confidence intervals. Failures retained.',fontsize=9)
    fig.savefig(output/'comparison.png',dpi=180);fig.savefig(output/'comparison.pdf');plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(10,4.5),layout='constrained')
    for ax,xkey,ykey,title in zip(axes,('predicted_new_m2','predicted_resampling_utility_m2'),
                               ('actual_new_m2','actual_recovered_surface_m2'),('New observed area','Deficient cells recovered')):
        markers={'nearest_view':'o','coverage_view':'s','surface_view':'^'}
        for strategy in names:
            selected=[p for p in predictions if p['strategy']==strategy and p.get(xkey) is not None and p.get(ykey) is not None]
            if not selected:continue
            marker=markers[strategy.rsplit('_v',1)[0]]
            ax.scatter([p[xkey] for p in selected],[p[ykey] for p in selected],marker=marker,
                       facecolors='none',edgecolors='black',s=25,label=names[strategy])
        upper=max([p[k] for p in predictions for k in (xkey,ykey) if p.get(k) is not None]+[1])
        ax.plot([0,upper],[0,upper],'k--',linewidth=.8)
        ax.set(xlim=(-upper*.03,upper*1.05),ylim=(-upper*.03,upper*1.05),xlabel='Predicted utility (m2)',ylabel='Observed gain in predicted cells (m2)',title=title)
        ax.legend(fontsize=8)
    fig.supxlabel('Includes failed/interrupted actions. Fixed resampling weight is a utility, not a calibrated recovery probability.',fontsize=8)
    fig.savefig(output/'prediction-errors.png',dpi=180);fig.savefig(output/'prediction-errors.pdf');plt.close(fig)
    def value(row,key):return f'{row[key]:.2f}' if row.get(key) is not None else '未报告'
    table='\n'.join(f"| {r['index']} | {r['start'][:2]} | {names[r['strategy']]} | {value(r,'new_observed_area_m2')} | {value(r,'valid_surface_gain_m2')} | {value(r,'path_m')} | {r['status']} |" for r in rows)
    (output/'README.md').write_text(f'''# 三策略同条件可行性实验

{len(plan['trials'])} 个预先列入本地计划的轮次；通过启动校验后才新建仿真和空白 SLAM/采集档案。重要运行源文件在各轮记录中一致：{matched}。
完整的初始基线、失败/中断、补采恢复率和动作见 comparison.csv 与各轮 runs/；grouped.csv 按起点分别汇总。
actions_failed 与 actions_interrupted 分开记录动作失败和预算/停止中断；原始 failed_actions 包含所有未成功动作，不能一概当成导航故障。

| 轮次 | 起点 | 策略 | 新观测 m² | 有效拟合净增 m² | 路程 m | 结束状态 |
| --- | --- | --- | ---: | ---: | ---: | --- |
{table}

![三策略对照](comparison.png)

![预测与实际](prediction-errors.png)

预测散点图包含失败和中断动作；补采权重仍是未标定效用。prediction-errors.csv 可用于后续标定，当前数据不同时充当训练集和独立验证集。

两次重复只描述差异，不做显著性检验或最优性结论。条形均值和极差不是置信区间。
图和均值使用有数值报告的轮次；启动失败不补零，保留在清单中，grouped.csv 同时列出计划轮数和有报告轮数。
Q、有效拟合面积、未知环境完成率和准确率是不同概念。恢复率以本轮开始时采样不足格子为分母，包含途中顺带改善，不能等同专门补采动作成功率。
定位路径包含漂移；起始条件统一但传感器到达时序和初始基线需据记录核对。精度尚未给三策略填入可靠共同真值，不以名义最高表面误差代替。
Nav2 和传感器模型共用，尚不涉及机器学习训练、实物硬件验收、返航或全场覆盖保证。
''',encoding='utf-8')
    write_json(output/'hashes.json',{str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in output.rglob('*') if p.is_file() and p!=output/'hashes.json'})


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();report(a.source,a.output)
