# 原 v2 约 1 厘米转向偏移：两轮诊断方案与测试准备交付包

- **报告编号**：`r5-v2-offset-diagnosis-prep-20261011`
- **任务依据**：`reports/evidence/terrain-v2/r5-v2-offset-refinement-codex-review-20261011/GEMINI_NEXT_TASK.md`
- **复核输入基线**：`reports/evidence/terrain-v2/r5-v2-offset-refinement-codex-review-20261011`
- **交付内容**：
  1. 完整两轮诊断方案文档（`DIAGNOSIS_PLAN.md`）：涵盖两轮配置差异、实际轮胎最低母线与有限接缝求交模型、关键事件动态定位方法及四项审慎判定准则。
  2. 机器可读运行配置（`campaign_plan.json`）：严格保留原世界出生 $Z = 0.03\text{ m}$，声明平移轮（X=+0.05m）与恢复轮（X=0.00m）的精确执行参数。
  3. 自动化后处理分析工具（`evaluate_diagnosis_run.py`）：基于轮胎实际位姿与最低母线几何的动态评估脚本，准备就绪。
  4. 完整性校验清单（`manifest.json`）。

---

## 目录文件清单

| 文件名 | 类型 | 说明 |
| :--- | :--- | :--- |
| `DIAGNOSIS_PLAN.md` | Markdown 报告 | 完整的两轮诊断方案、动态事件判定准则与解释边界说明 |
| `campaign_plan.json` | JSON 配置 | 两轮独立测试副本机器可读运行配置清单 |
| `evaluate_diagnosis_run.py` | Python 脚本 | 基于轮胎最低母线求交的自动化后处理与判定分析工具 |
| `manifest.json` | 校验哈希 | 本目录下所有交付文件的 SHA-256 校验清单 |
| `README.md` | Markdown 引导 | 本交付包说明文档 |

---

## 状态声明
> **本任务仅完成方案准备、测试配置副本与评估工具构建，绝不启动新仿真，不修改任何正式库或模型文件。待用户与 Codex 独立授权后执行。**
