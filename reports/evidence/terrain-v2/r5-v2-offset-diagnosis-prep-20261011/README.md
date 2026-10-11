# 原 v2 约 1 厘米转向偏移：两轮诊断执行工具与方案交付包

- **报告编号**：`r5-v2-offset-diagnosis-prep-20261011`
- **任务依据**：`reports/evidence/terrain-v2/r5-v2-offset-tools-review-20261011/GEMINI_FIX_EXECUTION.md`
- **复核输入基线**：`reports/evidence/terrain-v2/r5-v2-offset-tools-review-20261011`
- **状态声明**：**工具链已完整接入真实 ROS 控制链，全部离线与替身测试通过，具备执行两轮条件；未启动物理仿真，等待授权。**

---

## 目录文件清单

| 文件名 | 类型 | 说明 |
| :--- | :--- | :--- |
| `DIAGNOSIS_PLAN.md` | Markdown 报告 | 包含控制链接入、独立世界副本差异、资格与停止规则、因果证据提取与判定准则的完整方案 |
| `campaign_plan.json` | JSON 配置 | 包含 28.2 秒完整调度、出生位姿（$Z=0.03\text{ m}$）及输出路径的机器可读配置 |
| `worlds/world_shift_x05.sdf` | SDF 世界副本 | 独立平移世界文件（$X_0 = +0.05\text{ m}, Z_0 = 0.03\text{ m}$） |
| `worlds/world_restore_x00.sdf`| SDF 世界副本 | 独立恢复世界文件（$X_0 = 0.00\text{ m}, Z_0 = 0.03\text{ m}$） |
| `run_diagnosis_campaign.py` | Python 脚本 | 两轮调度主入口，支持 `--dry-run` 离线预检、`--run-round`、`--run-all`，内嵌停止规则与授权安全锁 |
| `check_r5_v2_offset_round.py` | Python 脚本 | 单轮物理执行入口，注入独立世界副本至 `check_r5_turn_gate_validation`，正确区分几何筛选与数据资格 |
| `evaluate_diagnosis_run.py` | Python 脚本 | 支持接触资格严格检查、共线重叠修复、四轮邻边求交、保存全部 4244 事件、输出 17019 突变与拒绝覆盖机制 |
| `test_execution_mock.py` | Python 测试 | 离线替身执行测试，全面验证世界消费、控制链传参、停止规则、覆盖保护及事件提取 |
| `manifest.json` | 校验哈希 | 本目录下所有交付文件的 SHA-256 校验清单 |
| `README.md` | Markdown 引导 | 本交付包说明与操作指引 |

---

## 常用验证命令

1. **执行调度器离线预检（验证容器、环境、世界哈希与控制链命令）**：
   ```bash
   python run_diagnosis_campaign.py --dry-run
   ```
2. **执行离线替身子进程与资格/停止规则完整自检**：
   ```bash
   python test_execution_mock.py
   ```
3. **执行评估工具自测分析（以原轮次 14 为例，提取全部事件与 17019 突变）**：
   ```bash
   python evaluate_diagnosis_run.py --run-dir ../r5-turn-gate-validation-20261011/campaign/14-pure_turn-ramp-1000us-left
   ```
