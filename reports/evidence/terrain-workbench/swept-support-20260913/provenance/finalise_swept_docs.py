from pathlib import Path
import json,hashlib,shutil
base=Path(r'D:\Document\Repositories\MoonCar-mapping');root=base/'reports/evidence/terrain-workbench/swept-support-20260913'
p=root/'README.md';text=p.read_text()
text=text.replace('拒绝原因：false_skip_of_paired_recovery, residual_stop_and_turn_not_both_validated','拒绝原因：一次跳过建议会遗漏配对恢复；没有覆盖真正的“无边际支撑”停止分支。对应机器记录为false_skip_of_paired_recovery、residual_stop_and_turn_not_both_validated。')
explanation='''## 结论与后续安排

所有8次采集/导航和4次观察转向均成功，仿真已停止。每次记录62至64帧，丢弃1至2个选中帧；实际首末帧跨度34.2至34.7仿真秒。名义预算一致不表示时间序列完全相同。

实际转向组的全部决策不足格，共82格次恢复。v1命中1、误报1、漏81；新支撑预测命中47、误报17、漏35，precision=73.4%、recall=57.3%。仅判断能否看到则误报126次，说明加入点分布门槛能够排除部分空泛机会，但仍不是可靠的恢复概率。

只看“最初就不足的格”：共4格次恢复，新规则命中2、误报2、漏2。配对共同最初不足格的净差仅0、0、0、1；全部决策不足格的配对净差为曲面7/15、坡面13/17。大多数恢复来自沿途新发现的不足格，不能据此声称最初目标缺口已大幅改善。

曲面第1次转向组的决策转角为0.093957rad（约5.38°），触发原先0.12rad“已经对齐”门槛，尽管模型仍有5格边际机会。按实验计划实际转向耗时0.683仿真秒，配对共有104个剩余不足格，转向组独有恢复7格。这个描述性差异不隔离定位/轨迹差异，但足以说明现有跳过判断未通过预设检查。不能把运动朝向容差直接当作信息收益为零。

本批7次建议观察、1次因为对齐跳过，0次因“无边际支撑”停止。预测指标改善不等于停止规则验证通过，因此不接入在线第三策略，也不把这8轮说成新的三策略对照。

下一步先分开“朝向已对齐”和“无补采价值”两个判断，验证小角度重观测、重复相同视角、支撑饱和与遮挡等低收益条件；比较保持观察的实际恢复、转向增量和耗时，再冻结规则并独立复验。通过后才接入第三策略并重跑统一预算的三策略对照。当前不依据这批结果临时降低门槛或宣称改进已通过。

'''
text=text.replace('## 文件与复核',explanation+'## 文件与复核');p.write_text(text,encoding='utf-8')
shutil.copyfile(base/'tmp/ros_acceptance/swept-plots-20260913.log',root/'provenance/swept-plots-20260913.log')
context=base/'docs/working_context.md';text=context.read_text()
entry='''## Swept support v2 and independent validation 2026-09-13

Added offline scripts/swept_resampling.py and test_swept_resampling.py. Uses only the
pre-decision estimated camera/base transforms, observed voxel cloud and CameraInfo.
Shortest world-Z turn preserves the camera lever arm/tilt, integrates <=0.25rad steps,
and intersects stride-4 rays with sensor-only median-height cell patches. Voxel union
and XY singular-value support replace coarse bearing-bin gating. Hypothesized points
never enter observed maps; no occlusion/tilt/noise/phase calibration. Frozen v1 and online
autonomous_survey.py unchanged. Core, scanline, partial-cell-FOV and no-future checks pass.

Old arrival-decision eight-capture batch is development only: initially deficient
recovery v1 0/7 vs v2 3/7, zero v2 false positives. New eight fresh35s captures use
ramp start[-1.3,1.3,-.4], goal[-.6,1.05], yaw.25; curve start[-1.8,2.,-.25],
goal[-1.05,1.65], yaw-.8. Each arm repeated twice, counterbalanced; baseline5s,
arrival settling1s. Source and admission criteria frozen before outcomes. All8
navigation/captures and4 spins succeeded; captured spans34.2..34.7s,62..64frames,
1..2selected drops. Simulation stopped. No new full three-strategy comparison.

Executed-turn ALL decision-deficient cohort: v1 TP1 FP1 FN81; v2 support TP47 FP17
FN35 (precision73.4%,recall57.3%); visibility-only TP72 FP126 FN10. INITIAL deficient
stratum: v2 TP2 FP2 FN2. Do not mix denominators: most recovery is in transit-discovered
cells. Shared decision-deficient turn-minus-hold recovery: curve R1+7 (n104),R2+15
(n109),ramp R1+13(n65),R2+17(n55). Shared initially deficient net0/0/0/+1.

Admission REJECTED: curve R1 turn shadow skip already_aligned at0.093957rad (5.38deg),
although5 modelled marginal cells remained; actual0.683s turn arm has7 exclusive
recoveries. This is descriptive, not isolated causal truth. Seven suggestions turn,
one alignment skip, ZERO no_modelled_marginal_support; residual-stop branch untested.
Do not lower the threshold on these outcomes and call it independently validated.
Next separate heading tolerance from information stopping; test small-angle changes,
held-view noise, saturated support and occlusion/low-gain cases with time costs.
Only after independent validation integrate into third policy and rerun common-budget
three-policy comparison. No hardware-readiness or overall-strategy advantage claim.

Evidence: reports/evidence/terrain-workbench/swept-support-20260913, including raw
captures, frozen sources/protocol, development vs independent results, per-cell CSV,
scene/observed-map panels, prediction/paired charts, hashes and stdlib checker.
Experiments6ec8ae0c0bc7,10b4f48bd30e,376da1b59ff1,1c504da3bdbf,
577682b19fd6,cc1a461fd8ca,5df595697453,074c5cf51b10.

'''
context.write_text(text.replace('# Working Context\n\n','# Working Context\n\n'+entry,1),encoding='utf-8')
p=base/'docs/resampling_model.md';text=p.read_text().replace('离线验证中','独立验证完成，准入未通过')
text+='''
本轮8次导航/采集与4次转向均成功。全部决策不足格上，旧v1命中1/82；新支撑模型
命中47/82、误报17（precision73.4%，recall57.3%）。最初已不足格单列为命中2、
误报2、漏2。模型准入仍未通过：一次5.38°小转角因“已经对齐”被建议跳过，但配对
实际多恢复7格；另无一次进入“无边际支撑”停止分支。前者是描述性配对结果，不能
单独证明转向因果收益。下一步分开运动朝向容差与信息停止判断，并补低收益、重复
观察和遮挡条件。详见reports/evidence/terrain-workbench/swept-support-20260913/README.md。
'''
p.write_text(text,encoding='utf-8')
hashes={str(p.relative_to(root)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in root.rglob('*') if p.is_file() and p.name not in ('hashes.json','check_evidence.py')}
(root/'hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
