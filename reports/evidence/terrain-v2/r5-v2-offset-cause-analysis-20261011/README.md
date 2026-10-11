# 原 v2 约 10.29 mm 转向偏移原因链离线分析交付包

- **报告编号**：`r5-v2-offset-cause-analysis-20261011`
- **对应任务**：原 v2 测试中 10.286 mm 转向偏移原因链排查 (Step 1 ~ Step 4)
- **分析目标轮次**：`reports/evidence/terrain-v2/r5-turn-gate-validation-20261011/campaign/14-pure_turn-ramp-1000us-left`
- **平板基准轮次**：`reports/evidence/terrain-v2/r5-turn-gate-validation-20261011/campaign/03-pure_turn-flat-1000us-left-1`

---

## 目录文件清单

| 文件名 | 类型 | 说明 |
| :--- | :--- | :--- |
| `V2_OFFSET_CAUSE_REPORT.md` | Markdown 报告 | 完整的因果链深度剖析报告，涵盖起点定位、四轮动力学闭合、平板对照与单因素方案 |
| `analyze_r14_cause.py` | Python 脚本 | 离线分析脚本，重构时序、计算四轮接触与载荷、执行水平冲量积分与动量闭合检验 |
| `causal_timeline.png` | 图像 | 8 面板高清动力学与几何时序图，展示轨迹发散、速度爆发、载荷转移、FR 腾空与冲量积分 |
| `cause_summary.json` | JSON 数据 | 结构化关键量化指标（时间戳、载荷、冲量、动量误差率、对比基准等） |
| `manifest.json` | 校验哈希 | 本目录下所有交付文件的 SHA-256 校验清单 |
| `README.md` | Markdown 引导 | 本交付包说明文档 |

---

## 核心分析结果概要

1. **偏移起点**：前 11.8 秒位移 $\le 0.35\text{ mm}$。突变发生于 **$t = 11.820\text{ s}$（Iter 17020）**，在随后的 0.5 秒内，车体平移速度骤增至 **$11.53\text{ mm/s}$**，向西南方向（$-125.03^\circ$）爆发位移 5.12 mm。
2. **四轮连锁反应**：
   - FL 轮跨越网格对角接缝，穿透深度跃升至 $2.51\ \mu\text{m}$，法向载荷暴增至 $12.94\text{ N}$；
   - 刚性底盘微幅翻滚，导致对角轮 **FR 腾空（出现多达 241 步 0 接触）**，驱动牵引力彻底归零；
   - 左右牵引力平衡瓦解，地面产生净冲量 $(J_x = -0.0471\text{ N}\cdot\text{s}, J_y = -0.0944\text{ N}\cdot\text{s})$；
   - 预测速度增量 $\Delta v = 11.49\text{ mm/s}$，实测 $\Delta v = 12.03\text{ mm/s}$，**误差仅 5.14%**，动力学严格闭合。
3. **平板对比**：平板全时程保持单轮 2 点接触（全车 8 点），穿透恒定 $1.1\text{ nm}$，无任何车轮腾空，水平合力处处为 0，位移仅 0.17 mm。
4. **单因素方案**：平移初始生成点（+5 cm），预算为 2 轮仿真（1 轮测试 + 1 轮恢复），已备妥方案待授权。
