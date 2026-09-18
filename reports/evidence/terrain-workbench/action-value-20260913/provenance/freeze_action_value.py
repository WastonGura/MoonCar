from pathlib import Path
import json,hashlib,shutil
base=Path(r'D:\Document\Repositories\MoonCar-mapping');temp=base/'tmp/ros_acceptance'
root=base/'reports/evidence/terrain-workbench/action-value-20260913';root.mkdir(parents=True,exist_ok=False)
shutil.copytree(temp/'action-value-20260913',root/'captures')
shutil.copytree(temp/'action-value-analysis-20260913',root/'analysis')
(root/'provenance').mkdir()
for name in ('run_action_value.py','evaluate_action_value.py','plot_action_value.py','freeze_action_value.py','finish_action_value.py'):
    shutil.copyfile(base/'tmp'/name,root/'provenance'/name)
for name in ('action-value-20260913.log','action-value-analysis-20260913.log'):
    shutil.copyfile(temp/name,root/'provenance'/name)
shutil.copyfile(base/'tmp/check_action_value.py',root/'check_evidence.py')
s=json.loads((root/'analysis/summary.json').read_text());valid=[t for t in s['trials'] if t['status']=='evaluated']
lines=['# 转向与换位的同预算采集比较','',
'日期：2026-09-13。软件仿真研究，在线三种策略没有替换。','',
f"预定8次独立重置，{len(valid)}次形成可评价窗口；完整性判定：{'满足' if s['complete'] else '未满足，失败或未完成项保留'}。这不是创新效果或在线停止规则验收。",'',
'## 本轮解决的问题','',
'把“某个补采目标有没有恢复”扩展为“在相同时间里，转向或换位获得了什么”。所有动作按命令发出后20仿真秒评价，包含运动、转向和观察；保持组也使用同样时间，避免把几乎零运动成本作为除数。','',
'复用survey_evaluation.gain：收益=新增观测投影面积+原有采样不足格恢复为可拟合的面积。两部分分别列出，新增格不会再算作旧格补采。当前目标内、目标外的恢复，以及原本有效但后来失效的面积另行保存；正收益不能掩盖失效。恢复不是精度改善，新覆盖也不等于可通行地面。','',
'## 事前固定的条件','',
'坡面场景，两次重复，第二次反转四组执行顺序：保持不动、+0.02rad小角度、+0.4rad较大角度、Nav2换位后恢复决策时朝向。起点(-1.4,1.55,-0.3rad)，共同先到(-0.65,1.3)，下一位置(0.1,1.05)m。每次60仿真秒，起步前至少观察5秒并固定不足格，到达后共同保持5秒。','',
'全部四组都查询下一位置的实际Nav2路径，再发出预定动作；路径检查拒绝越过当前地图未知或占据格。区域统计采用起始位置周围半径8m，不是整个50×50m场景的完成率。传感器前缀不超过5秒旧且至少剩余22秒；源代码和协议在新数据产生前冻结。','',
'## 实际结果','',
'| 试验 | 新增观测m² | 旧格恢复m² | 目标内/外恢复格 | 有效面积损失m² | 采样路程m | 动作耗时s |','|---|---:|---:|---:|---:|---:|---:|']
for t in s['trials']:
    if t['status']!='evaluated':lines.append(f"| {t['folder']} | 失败：{t.get('error','未完成')} | — | — | — | — | — |")
    else:
        duration='未完成' if t['action_seconds'] is None else f"{t['action_seconds']:.3f}"
        lines.append(f"| {t['folder']} | {t['new_observed_m2']:.4f} | {t['recovered_surface_m2']:.4f} | {t['target_recovered_cells']}/{t['outside_target_recovered_cells']} | {t['lost_valid_surface_m2']:.4f} | {t['estimated_path_m']:.3f} | {duration} |")
lines+=['','失败记录明细：','']
for t in s['trials']:
    if t['status']=='evaluated':continue
    folder=root/'captures'/t['folder'];errors=[]
    for path in (folder/'approach-control.json',folder/'approach/manifest.json',root/'captures'/(t['folder']+'-failure.json')):
        if path.exists():
            value=json.loads(path.read_text())
            if value.get('error'):errors.append(str(value['error']))
    lines.append(f"- {t['folder']}："+'；'.join(dict.fromkeys(errors or [t.get('error','未完成')])) )
lines+=['','投影参数旧错误文案同时覆盖请求未完成、空响应与类型不匹配；单凭该文案不能认定参数类型真的配错。文件读取错误没有完整调用栈，也不能据此宣布其根因已修复。','',
'以上是每组相对自身动作前地图的变化；独立重置产生的初始观测差异存在，不能直接把两列相减当成因果收益。动作耗时来自控制事件，采样路程来自估计位姿，含定位漂移并可能低估采样间运动。所有收益率统一除以20秒，详见trials.csv。','',
'![四组动作的实测分项](analysis/action-value.png)','',
'## 共同格子比较','',
'只有四个组都有可评价数据的重复才进入此表。共同不足格取四组动作前不足格的交集；共同新格排除四组动作前地图的并集，再统计各组实际新观测。固定目标的交集单列，避免把沿途新发现的不足格误算成原目标收益。','',
'| 重复 | 动作 | 共同新格 | 共同不足格恢复 | 其中共同原目标恢复 | 相对保持正向收益差m² |','|---|---|---:|---:|---:|---:|']
for p in s['paired']:lines.append(f"| {p['repeat']} | {p['arm']} | {p['shared_new_cells']} | {p['shared_recovered_cells']} | {p['shared_target_recovered']} | {p['net_utility_vs_hold_m2']:+.4f} |")
if not s['paired']:lines.append('| — | 没有完整四组对照 | — | — | — | — |')
else:
    lines+=['','本批唯一完整的第二轮，共同不足格34格，其中10格属于共同的原始目标。保持、小转角、较大转角、换位的正向收益差依次为0、0.125、1.1875、4.0000m²。换位与较大转角都恢复12个共同不足格，但换位新增52格，较大转角新增7格；换位还恢复6个共同原目标，三个原地组均为0。这里换位的优势主要体现在新覆盖及原目标恢复，不能推广成所有地形都应优先移动。','',
    '第二轮较大转角另有1个原本有效格失效，小转角有2个，分别为0.0625、0.125m²；保持和换位没有此类损失。正向收益差没有扣除这项分组损失，也不是风险或精度净收益。后续决策必须保留退化项，不能只奖励恢复。','',
    '![共同样本正向收益差](analysis/shared-cohorts.png)']
lines+=['','## 模型与实测仍需对齐','',
'本轮没有把不同模型的分数强行排成统一推荐顺序。转向继续使用v4支撑预测；换位继续用既有路径观察机会模型。二者不是同一标尺，预测均未给出可信的新增覆盖量，所以joint_predicted_rank为空。forecasts.csv保留各自的误报、漏报和命令前已变化的格子，不能称为已经可靠预测总收益。','',
'stages.csv记录换位完成、转向完成相对同一命令前地图的累计变化，不能把这些累计值相加。阶段中的收益率也统一按20秒预算归一化，不是该阶段瞬时速率。scene-map图右侧取精确20秒窗口的观测地图；Gazebo截图来自采集结束时，只用于场景对照。','',
'## 下一步','',
'先处理本批暴露的启动/记录失败并补齐独立对照；根据误报/漏报核对转向和换位模型，使其能估计同一种收益及时间成本。随后扩展曲面、遮挡和不同起点，固定规则后接入surface_view_v1，再做完整三策略比较。不能从两个坡面重复推出普遍最优策略，也不能从目标内零恢复推出应该停止探索。','',
'## 可复核材料','',
'captures保存原始深度、同步位姿、前缀、真实Nav2路径、控制事件、成功及失败记录、冻结协议与源码；analysis保存逐格表、阶段表、预测表、结果图和现场对照图。python check_evidence.py验证哈希和汇总；冻结evaluate_action_value.py在现有NumPy环境中逐数组核对前缀并重建地图评价。','']
(root/'README.md').write_text('\n'.join(lines),encoding='utf-8')
hashes={str(p.relative_to(root)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in root.rglob('*') if p.is_file() and p.name!='hashes.json'}
(root/'hashes.json').write_text(json.dumps(hashes,indent=2)+'\n');print(root)
