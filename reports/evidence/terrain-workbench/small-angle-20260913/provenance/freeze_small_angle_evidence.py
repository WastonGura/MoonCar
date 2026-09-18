from pathlib import Path
import json,shutil,hashlib
base=Path(r'D:\Document\Repositories\MoonCar-mapping');temp=base/'tmp/ros_acceptance'
root=base/'reports/evidence/terrain-workbench/small-angle-20260913';root.mkdir(parents=True,exist_ok=False)
for source,target in [('small-angle-validation-20260913','captures'),('small-angle-validation-analysis-20260913','analysis'),('small-angle-development-20260913','development')]:shutil.copytree(temp/source,root/target)
(root/'provenance').mkdir()
for name in ['run_small_angle_validation.py','analyse_small_angle_validation.py','assess_small_angle_validation.py','measure_small_turns.py','check_small_angle_semantics.py','check_gate_only_change.py','audit_small_angle_coverage.py','plot_swept_validation.py','freeze_small_angle_evidence.py']:
    shutil.copyfile(base/'tmp'/name,root/'provenance'/name)
for name in ['small-angle-validation-20260913.log','small-angle-validation-analysis-20260913.log','small-angle-development-20260913.log','small-angle-semantics-20260913.log','small-angle-old-evidence-check-20260913.log','small-angle-plots-20260913.log','small-angle-rotations-20260913.log']:
    shutil.copyfile(temp/name,root/'provenance'/name)
shutil.copyfile(base/'scripts/survey_evaluation.py',root/'provenance/survey_evaluation.py')
shutil.copyfile(base/'tmp/check_small_angle_evidence.py',root/'check_evidence.py')
s=json.loads((root/'analysis/summary.json').read_text());a=json.loads((root/'analysis/assessment.json').read_text());plan=json.loads((root/'captures/plan.json').read_text())
lines=['# 小角度补采与重复观察：v3决策验证','', '日期：2026-09-13。软件仿真实验；未进行硬件验收或新的三策略总体对照。','',
'## 修改','',
'将“车辆朝向已对齐”与“没有额外补采价值”分开。v3取消0.12rad角度容差对补采建议的直接否决，继续检查预算和预测边际支撑；v2的点云/射线支撑模型及门槛保持不变，v2评估入口仍保留，线上三种策略未改动。','',
'跳过意味着模型没有预测到相对保持朝向的额外支撑，不代表静止观察不会恢复、总信息收益为零或整个环境已完成。当前仍是未部署的旁路候选。','',
'## 预先冻结的验证','',
'两个新起点/目标，两个相对转角，转向与保持各重复两次并交替次序，共8轮。每次重建仿真及采集档案。35仿真秒包括5秒基线、到达后5秒共同观察；“重复观察后低收益”是被检验的假设，不是预先确认的结果。','',
'| 场景 | 起点X/Y/朝向rad | 目标X/Y | 请求相对转角rad |','|---|---|---|---|']
for t in plan['trials'][::2][:2]:lines.append(f"| {t['scene']} | {t['start']} | {t['goal']} | {t['observation_delta']} |")
lines+=['',
'控制脚本在决策时根据估计车体朝向解析绝对目标并记录位姿、目标与时间。两个实验组均解析同样的相对角度规则，只有转向组发送Spin；两组独立运行，绝对朝向/轨迹可能略有不同。预先获取的Nav2路径仍单独保存并检查已知空间。','',
'转向执行门槛：实际估计转角方向正确、至少请求量一半、误差不超过0.02rad+请求量25%。另保存同步传感器帧间的估计/真值转角，仅事后诊断，不更改准入条件。新规则没有削弱在线运动或碰撞保护。','',
'准入沿用precision/recall各至少0.5、不得漏掉配对独有恢复、无正净收益的建议转向配对最多1组，以及完整8轮和真正的转向/无边际支撑两个分支覆盖。零角度合成检查只验证逻辑，不算低收益实跑证据。所有门槛均为探索性工程要求，不是统计显著性标准。','',
'## 实际结果','',f"旁路准入：{'通过' if a['shadow_check_passed'] else '未通过'}。原因：{', '.join(a['rejections']) or '无'}。",'',
'| 实验 | 执行评估 | v2建议 | v3建议 | 转向耗时（仿真秒） |','|---|---|---|---|---|']
for t in s['trials']:lines.append(f"| {t['folder']} | {t['status']} | {t.get('v2_gate',{}).get('reason','—')} | {t.get('gate',{}).get('reason','—')} | {t.get('turn_sim_seconds',0):.3f} |")
lines+=['','| 转向实验 | 请求转角rad | 实际估计转角rad | 执行门槛 |','|---|---:|---:|---|']
for t in a['execution']:lines.append(f"| {t['folder']} | {t['requested']:.6f} | {t['actual']:.6f} | {t['verified']} |")
lines+=['','只对实际转向组评价恢复预测。支撑预测仍是v2几何模型，v3改变的是是否执行建议。','',
'| 统计范围 | 预测规则 | 命中 | 误报 | 漏报 | precision | recall |','|---|---|---:|---:|---:|---:|---:|']
def pct(v):return f'{100*v:.1f}%' if v is not None else '无分母'
for g in s['metrics']:
    if g['arm']=='turn':lines.append(f"| {g['scope']} | {g['rule']} | {g['tp']} | {g['fp']} | {g['fn']} | {pct(g['precision'])} | {pct(g['recall'])} |")
lines+=['','all_decision_deficient是决策时全部不足格，initial_deficient仅含最初已不足且此时仍不足格；不能混用分母。恢复为真实观测累计后TerrainMap拟合有效，不是几何测量准确率。','',
'| 配对 | 范围 | 共同不足格 | 转向独有恢复 | 保持独有恢复 | 净差 | v3是否建议转向 |','|---|---|---:|---:|---:|---:|---|']
for p in s['pairs']:
    if p['status']=='evaluated':lines.append(f"| {p['pair']} | {p['scope']} | {p['common_remaining']} | {p['turn_only']} | {p['hold_only']} | {p['net_recovered']} | {p['would_turn']} |")
lines+=['','格次相互相关，不是独立样本。配对包含共同决策不足格，不能消除轨迹、观测时序和定位误差；净差不是逐格因果真值。未计算新的几何准确率。','',
'![配对恢复](analysis/paired-recovery.png)','', '![预测误差](analysis/prediction-errors.png)','',
'## 旧数据仅作开发回放','',
'上一批swept-support的8轮回放中，原先5.38°转角触发的already_aligned建议已变成try_observation。几何预测逐格结果未因v3门槛分离而改善；这个回放不算独立验证。来源为相邻swept-support-20260913/captures。','',
'## 资料与复核','',
'- captures：原始深度、点云、同步位姿、事前路径/地图、控制事件与现场截图、冻结源码和方案。','- analysis/cells.csv、metrics.csv、pairs.json、assessment.json：逐格记录、统计和准入；rotation-audit.json是事后旋转诊断。','- analysis/*-scene-map.png：实际现场、决策时和最终的观测点云；方框仅为预测标记，没有写入地图。','- development：旧数据v3回放；provenance：运行/分析/绘图脚本与日志。','',
'运行 `python check_evidence.py` 可用标准库校验哈希、8个独立采集、相对目标解析、已知空间事前路径、事件时间边界、统计范围与配对结果。预测逻辑检查在仓库test_swept_resampling.py及provenance/check_small_angle_semantics.py，需要已有NumPy/ROS环境。','',
'模型限制保留：中位高度水平面片未建模遮挡、坡度、噪声和采样时刻；支撑矩阵用样本均值中心，真实地表拟合用格心中心。预算代理仍为角度/0.5+3秒；不等同真实转向耗时。','']
(root/'README.md').write_text('\n'.join(lines),encoding='utf-8')
hashes={str(p.relative_to(root)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in root.rglob('*') if p.is_file() and p.name not in ('hashes.json','check_evidence.py')}
(root/'hashes.json').write_text(json.dumps(hashes,indent=2)+'\n');print(root)
