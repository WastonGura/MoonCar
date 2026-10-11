# 原 v2 约 1 厘米转向偏移：两轮诊断执行工具与方案交付包

- **报告编号**：`r5-v2-offset-diagnosis-prep-20261011`
- **任务依据**：`reports/evidence/terrain-v2/r5-v2-offset-prep-codex-review-20261011/GEMINI_IMPLEMENTATION_TASK.md`
- **复核输入基线**：`reports/evidence/terrain-v2/r5-v2-offset-prep-codex-review-20261011`
- **状态声明**：**工具与副本准备完毕，离线测试全部通过，具备执行两轮条件；未启动物理仿真，等待授权。**

---

## 目录文件清单

| 文件名 | 类型 | 说明 |
| :--- | :--- | :--- |
| `DIAGNOSIS_PLAN.md` | Markdown 报告 | 包含独立世界副本差异、资格检查升级、四轮因果证据提取与判定准则的完整方案 |
| `campaign_plan.json` | JSON 配置 | 包含 28.2 秒完整调度、出生位姿（$Z=0.03\text{ m}$）及输出路径的机器可读配置 |
| `worlds/world_shift_x05.sdf` | SDF 世界副本 | 独立平移世界文件（$X_0 = +0.05\text{ m}, Z_0 = 0.03\text{ m}$） |
| `worlds/world_restore_x00.sdf`| SDF 世界副本 | 独立恢复世界文件（$X_0 = 0.00\text{ m}, Z_0 = 0.03\text{ m}$） |
| `run_diagnosis_campaign.py` | Python 脚本 | 消费独立世界副本的调度入口，支持 `--dry-run` 离线校验与授权安全锁 |
| `evaluate_diagnosis_run.py` | Python 脚本 | 支持接触资格严格检查、共线重叠修复、四轮母线求交与因果事件输出的评估工具 |
| `manifest.json` | 校验哈希 | 本目录下所有交付文件的 SHA-256 校验清单 |
| `README.md` | Markdown 引导 | 本交付包说明与操作指引 |

---

## 常用验证命令

1. **执行调度器离线预检（无需 Gazebo）**：
   ```bash
   python run_diagnosis_campaign.py --dry-run
   ```
2. **执行评估工具自测分析（以原轮次 14 为例）**：
   ```bash
   python evaluate_diagnosis_run.py --run-dir ../r5-turn-gate-validation-20261011/campaign/14-pure_turn-ramp-1000us-left
   ```
