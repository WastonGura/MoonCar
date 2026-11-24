# R5 转向偏移调查离线补充分析归档 (2026-10-11)

本目录为针对 Codex 独立验收追问任务文件 `reports/evidence/terrain-v2/r5-contact-geometry-codex-review-20261011/GEMINI_FOLLOWUP.md` 的独立离线补充分析交付包。

---

## 目录文件清单

| 文件名 | 类型 | 说明 |
| :--- | :--- | :--- |
| `GEMINI_REPLY_SUPPLEMENT.md` | 文档 | 正式答辩与补充分析主报告，逐项回复追问 |
| `ERRATA.md` | 文档 | 容器退出码（137 vs 0）、软件回归项数（509项）及非破坏性复现说明 |
| `supplement_analysis.py` | 脚本 | 只读纯离线分析脚本（加载已冻结的八轮原始 CSV/JSON 数据，生成图表与汇总） |
| `analysis_summary.json` | 数据 | 八轮指标全量汇总（位移、转角、ICR、载荷、ROS/Gazebo 里程计误差、镜像比对） |
| `input_hashes.json` | 校验 | 离线分析所使用全部原始输入数据的 SHA-256 哈希清单 |
| `manifest.json` | 校验 | 本补充分析目录下各生成产物的 SHA-256 哈希清单 |
| `mirror_comparison.png` | 图表 | 对照组 A (04 vs 05) 与对照组 B (03 vs 06) 的全时程轨迹、连续航向与发散曲线 |
| `contact_timeline.png` | 图表 | 接触法向偏角时序、Iter 17034-17036 接触丢失与穿透突变细节图 |
| `load_forces.png` | 图表 | 八轮最大位移、倾斜步数、ROS/Gazebo 里程计航向误差及全车垂直载荷积分图 |

---

## 核心定量发现摘要

1. **镜像对称性复核**：
   - **04 vs 05（强镜像组）**：外后轮于步数 8781~18201 充分跨越对角接缝，全时程 20,000 步位置差 $\le 4.49\ \mu\text{m}$，航向差 $\le 13.2\ \mu\text{rad}$，倾斜步数均为 204 步。
   - **03 vs 06（分岔发散组）**：前 12 秒严格吻合（$\le 4.5\ \mu\text{m}$）；在 Iter 17034，06 外后轮（RL）处于掠过临界深度（0.000 m），于 17035 发生 1 ms 瞬时失触（0点），随后在 17036 撞击网格斜面陷入 $2.49\ \mu\text{m}$ 深度穿透，诱发持续 247 步的倾斜扰动序列，累积导致末端 2.35 mm 差异与 0.89° 航向偏差。
2. **三级偏移量级明确区分**：
   - 原 31 cm 偏移：轴间速度分配不当所致，已校准修复；
   - 本网格夹具偏移（0.89 ~ 2.17 mm）：0.25 m 水平网格内部接缝接触偏转及穿透突变引发；
   - 原 v2 地形残余偏移（~10.29 mm）：本水平网格夹具未复现该量级，现有夹具尚未定量解释原 v2 结果。
3. **载荷与侧滑平衡**：
   - 全车四轮总垂直载荷均值严格稳定在 $14.8813\text{ N}$，与月球重力（$9.186\text{ kg} \times 1.62\text{ m/s}^2$）误差小于 $4.6\times 10^{-5}\%$；
   - 质心后偏（$X_{\text{CoM}} = -0.00624\text{ m}$）导致后轴单轮载荷（$3.87\text{ N}$）高于前轴（$3.57\text{ N}$）约 8.3%，是外后轮主导接缝穿透扰动的物理基础。

---

## 零仿真离线复现方法

复核人员只需执行本目录下的只读脚本，即可重新生成所有图表与 JSON：

```powershell
# 1. 验证输入哈希
python -c "
import hashlib, json
from pathlib import Path
root = Path(r'D:\Documents\Projects\Repositories\MoonCar-mapping')
manifest = json.loads((root / 'reports/evidence/terrain-v2/r5-contact-geometry-supplement-20261011/input_hashes.json').read_text())
for rel, h in manifest.items():
    p = root / 'reports/evidence/terrain-v2' / rel
    assert hashlib.sha256(p.read_bytes()).hexdigest() == h
print('All inputs verified.')
"

# 2. 运行分析
python supplement_analysis.py
```
