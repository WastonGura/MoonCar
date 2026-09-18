from pathlib import Path
import json,shutil,hashlib,datetime
base=Path(r'D:\Document\Repositories\MoonCar-mapping');temp=base/'tmp/ros_acceptance'
root=base/'reports/evidence/terrain-workbench/swept-support-20260913';root.mkdir(parents=True,exist_ok=False)
shutil.copytree(temp/'swept-validation-20260913',root/'captures')
shutil.copytree(temp/'swept-validation-analysis-20260913',root/'analysis')
shutil.copytree(temp/'swept-development-scoped-20260913',root/'development')
(root/'provenance').mkdir()
for name in ['run_swept_validation.py','analyse_swept_validation.py','assess_swept_validation.py','plot_swept_validation.py','check_swept_scanline.py','time_swept_model.py','freeze_swept_evidence.py']:
    shutil.copyfile(base/'tmp'/name,root/'provenance'/name)
for name in ['swept-validation-20260913.log','swept-validation-analysis-20260913.log','swept-development-scoped-20260913.log','swept-scanline-check-20260913.log','swept-runtime-20260913.json']:
    shutil.copyfile(temp/name,root/'provenance'/name)
shutil.copyfile(base/'scripts/survey_evaluation.py',root/'provenance/survey_evaluation.py')
shutil.copyfile(base/'tmp/check_swept_evidence.py',root/'check_evidence.py')
s=json.loads((root/'analysis/summary.json').read_text());a=json.loads((root/'analysis/assessment.json').read_text());plan=json.loads((root/'captures/plan.json').read_text())
lines=['# 转向扫掠与点支撑模型：独立验证','', '日期：2026-09-13。全部为软件仿真，不是硬件验收或三策略总体优越性证明。','',
'## 这一轮做了什么','',
'新增离线候选 swept-support-v2。使用决策之前的点云、估计位姿和相机内参，计算最短转向过程中可能新增的体素与XY展布；同时预测保持朝向的支撑，比较是否值得转向。候选射线交点绝不写入采集地图。旧v1及在线三种策略保持原样。','',
'相机按照最多0.25rad角度间隔积分，保留实测偏置与倾斜。点云按原生产端stride=4采样；假设格子高度为已有点中位数，光轴深度0.1至6米，体素0.05米。拟合支撑门槛复用至少6点及最小奇异值/√N≥格长5%。这是水平面片假设，尚未建模遮挡、坡度、噪声和采样相位，不是概率或真实精度。','',
'## 实验与准入','',
'新起点、距离和最终朝向；两个场景×转向/保持两组×两次重复，共8轮，每轮重建仿真及采集档案。统一35仿真秒，含5秒基线及到达后1秒共同静置，组间次序交替。每轮先查询并保存Nav2已知空间路径，再移动。两组均按预定方案执行，模型只给出旁路建议，不左右实验结果。','',
'| 场景 | 起点X/Y/朝向rad | 目标X/Y | 最终观察朝向rad |','|---|---|---|---|']
for t in plan['trials'][::2][:2]:lines.append(f"| {t['scene']} | {t['start']} | {t['goal']} | {t['observation_yaw']} |")
lines+=['','运行前冻结预测源码及准入条件：完整执行8轮；全部决策不足格上支撑预测的precision、recall均至少0.5，且recall超过v1；不得错误跳过配对独有恢复，建议转向却无正净收益的配对最多1组；必须实际覆盖“尝试转向”和“没有边际支撑”两个分支。对齐/预算不足不能替代停止分支验证。门槛是探索性工程要求，不是统计显著性标准。','',
'## 结果','',f"旁路准入检查：{'通过' if a['shadow_check_passed'] else '未通过'}。在线策略未修改，未启动新的完整三策略对照。",'',
'拒绝原因：'+', '.join(a['rejections']), '',
'| 实验 | 采集/导航评估 | 建议 | 实际转向耗时（仿真秒） |','|---|---|---|---|']
for t in s['trials']:lines.append(f"| {t['folder']} | {t['status']} | {t.get('gate',{}).get('reason','—')} | {t.get('turn_sim_seconds',0):.3f} |")
lines+=['','以下仅对实际转向组评价预测。保持组的转向预测为未执行的假设，不用来证明转向有效。','',
'| 统计范围 | 规则 | 命中 | 误报 | 漏报 | precision | recall |','|---|---|---:|---:|---:|---:|---:|']
def pct(v):return f'{100*v:.1f}%' if v is not None else '无分母'
for g in s['metrics']:
    if g['arm']=='turn':lines.append(f"| {g['scope']} | {g['rule']} | {g['tp']} | {g['fp']} | {g['fn']} | {pct(g['precision'])} | {pct(g['recall'])} |")
lines+=['','all_decision_deficient包含沿途新发现的不足格；initial_deficient仅保留最初已不足且决策时仍不足的格。二者不能混用分母。precision是预测恢复中实际恢复的比例；recall是实际恢复中被预测命中的比例；它们均不是测量精度。','',
'| 配对 | 范围 | 共同剩余不足格 | 仅转向组恢复 | 仅保持组恢复 | 净差 |','|---|---|---:|---:|---:|---:|']
for p in s['pairs']:
    if p['status']=='evaluated':lines.append(f"| {p['pair']} | {p['scope']} | {p['common_remaining']} | {p['turn_only']} | {p['hold_only']} | {p['net_recovered']} |")
lines+=['','同一格在两组决策时均不足才进入配对比较。组间实际轨迹和传感器时序略有差异，净差不是逐格因果真值。格次彼此相关，不是独立试验重复。没有计算新的几何精度指标。','',
'![预测误差](analysis/prediction-errors.png)','', '![配对恢复](analysis/paired-recovery.png)','',
'## 开发数据与独立数据','',
'上一批arrival-decision的8轮只作为开发回放，存于development。最初不足格的7次实际恢复，v1命中0次；新支撑规则命中3次、漏4次、误报0次。该结果不属于独立验证。当前新8轮源码冻结后未根据结果修改门槛。','',
'## 文件与复核','',
'- captures：每轮原始深度、点云、同步位姿、控制事件、Nav2事前路径/地图、现场截图、冻结源码与方案。','- analysis/cells.csv：逐格预测与最终标签；metrics.csv、pairs.json、assessment.json：可重算的统计与判定。','- analysis/*-scene-map.png：实际现场、决策时观测点云、最终观测点云；方框是预测标记，不是实测点。','- development：旧批次开发回放，来源为相邻arrival-decision-20260913/captures。','- provenance：运行与分析脚本、日志、环境版本和补充检查；hashes.json保护冻结文件。','',
'在本目录运行 `python check_evidence.py` 可用标准库核对哈希、实际执行、先规划后移动、无未来帧输入边界、两种统计范围和配对结果。模型逻辑检查位于仓库ros2_ws/src/mooncar_terrain/tests/test_swept_resampling.py，在已有ROS容器中运行。','']
(root/'README.md').write_text('\n'.join(lines),encoding='utf-8')
hashes={str(p.relative_to(root)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in root.rglob('*') if p.is_file() and p.name not in ('hashes.json','check_evidence.py')}
(root/'hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
print(root)
