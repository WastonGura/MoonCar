# MoonCar（寻迹者探测车）

MoonCar（寻迹者探测车）是一套基于 **ROS 2 Jazzy + Gazebo** 的四轮模拟月面地形勘测车原型，以“仿真先行、后续迁移树莓派 + STM32 实物平台”为路线。系统以可测量的地表覆盖率与几何精度为主要输出，整合定位、地形可通行性评估、自主观测规划与 Nav2 导航避障。

---

## 当前基线与最新进展（2026-11）

当前默认仿真基线由 [config/simulation_baseline.json](config/simulation_baseline.json) 定义：

* **车型与支撑配置**：`r5_support_v1`（`independent_linear_v1` 独立悬架支撑，`0.25 ms` 物理步长）
* **默认场景**：`survey_v3_native`（30 × 30 m 连续重构 V3 地形，无接缝人工坡度）
* **定位后端**：`rgbd_window`（多关键帧滑动窗口 RGBD + IMU 定位，关闭主动回访）
* **地形测绘**：邻域高程可通行性判断 + 点云自动分段存档（突破 500 万点单段限制）

### 近期关键验证节点

* **2026-11-22 — V3 四组 8 分钟自主探索与云台消融对照（含 `v4.1` 迭代）**：见 [reports/v4_4way_comparison_20261122/README.md](reports/v4_4way_comparison_20261122/README.md)。在抬升云台 + 单 RGBD 渲染 + R5 Nav2 足迹基线上，`hybrid_frontier_v4.1`（有云台）以 **550.75 m² 总观测 / 495.06 m² 有效高程** 全面刷新纪录（较 `surface_v3` 高 +9.2%、较无云台版高 +15.1%、较 `v4.0` 提升 +33.4%），实现 **0 次底盘原地自转**、**27/27 (100%) 动作成功**、**0 定位暂停**、**XY P95 5.94 cm** 与 **高程 RMSE 0.15 cm**（四路 2560×1260 并排对比视频归档至本地 `output/v4-4way-comparison-20261122/`）。
* **2026-11-18 — V3 自主探测固定版 vs 云台版 8 分钟对照**：见 [reports/v3_camera_comparison_20261118/README.md](reports/v3_camera_comparison_20261118/README.md)。在 480 仿真秒预算内，固定版与云台版分别观测 488.44 / 499.88 m²（有效高程面积 432.12 / 456.25 m²），XY 定位误差 P95 分别为 5.37 / 8.23 cm，共同区域高程 RMSE 0.22 / 0.65 cm，全程 0 次定位暂停。
* **2026-11-17 — V3 连续地形与 8 分钟自主探索**：重建无接缝连续表面（[docs/terrain_v3_20261117.md](docs/terrain_v3_20261117.md)），并记录 8 分钟长跑表现与测绘容量边界（[docs/v3_eight_minute_comparison_20261117.md](docs/v3_eight_minute_comparison_20261117.md)）。
* **2026-11-04 — 固定视向 / 云台观察对照与全面复测**：验证云台可无位移完成侧向观察（[reports/CAMERA_MODE_COMPARISON_20261104.md](reports/CAMERA_MODE_COMPARISON_20261104.md)），并汇总剩余问题与修复清单（[docs/repair_checklist_20261103.md](docs/repair_checklist_20261103.md)）。
* **2026-11-03 — 中期研究报告**：已交付 9 页中文论文式中期报告（本地 `output/pdf/寻迹者探测车_中期研究报告_20261103.pdf`）。

> 历史工作日志与早期验证报告保留在 [docs/archive/working_context_20261105.md](docs/archive/working_context_20261105.md)、[docs/work_logs/](docs/work_logs/) 与 [reports/](reports/)。

---

## 仓库结构

```text
config/        默认仿真基线配置（simulation_baseline.json）
ros2_ws/       ROS 2 Jazzy 工作空间（description / gazebo / control / slam / navigation / terrain / bringup / msgs）
scripts/       工作台服务、自主勘测、滑动窗口定位（offline_rgbd_window/）与验证工具
web/           基于 Three.js 的三维地形工作台前端
containers/    Docker 验证环境、R5 物理与 DDS 运行库构建脚本
firmware/      可移植 STM32 控制核心与主机单测（板级 HAL 待实物联调）
tests/         仓库级 Python 单元测试（窗口定位、导航与验证工具）
docs/          架构设计、工作流说明、修复清单与历史归档
reports/       实验报告、对照结果与评分脚本
```

核心技术文档入口：
* 文献与算法溯源：[docs/references.md](docs/references.md)
* 系统架构与仿真计划：[docs/architecture.md](docs/architecture.md) · [docs/simulation_plan.md](docs/simulation_plan.md) · [docs/simulation_validation_scope.md](docs/simulation_validation_scope.md)
* 定位与测绘工作流：[docs/window_localization.md](docs/window_localization.md) · [docs/offline_survey.md](docs/offline_survey.md) · [docs/live_survey.md](docs/live_survey.md) · [docs/mapping_acceptance.md](docs/mapping_acceptance.md)
* 问题修复清单与硬件交接：[docs/repair_checklist_20261103.md](docs/repair_checklist_20261103.md) · [docs/hardware_handoff.md](docs/hardware_handoff.md) · [docs/uart_protocol.md](docs/uart_protocol.md)
* 阶段成果汇报与实车议题：[docs/PROGRESS_BRIEFING_20261124.md](docs/PROGRESS_BRIEFING_20261124.md) · 研发历程与技术演进：[docs/PROJECT_STORY_AND_EVOLUTION.md](docs/PROJECT_STORY_AND_EVOLUTION.md) · 学术文献溯源：[docs/references.md](docs/references.md)

---

## 快速启动

### 1. Windows + Docker 地形工作台

前置条件：已启动 Docker Desktop，且本地已安装 `uv`（或 Python 3 + `numpy`）。

```powershell
# 启动默认工作台（网页 http://127.0.0.1:8765，容器后端 http://127.0.0.1:8766）
./scripts/start_workbench.ps1

# 启动独立 R5 支撑候选工作台（网页 http://127.0.0.1:8775）
./scripts/start_workbench.ps1 -R5Candidate
```

启动脚本优先通过 `uv run --no-project` 运行宿主机侧脚本（也支持 `MOONCAR_PYTHON` 环境变量或系统 `python` / `py`）。容器镜像构建与详细用法见 [containers/README.md](containers/README.md) 与 [web/README.md](web/README.md)。

### 2. 软件检查与回归（Ubuntu 24.04 / ROS 2 Jazzy 容器内）

```bash
# 软件与固件单元测试
bash scripts/check_software.sh

# M6 / M5 / M4 一键验收（支持 --rviz-only 或 --server-only）
bash scripts/accept_m6.sh --server-only
```

---

## 仿真阶段里程碑与后续事项

详见 [docs/repair_checklist_20261103.md](docs/repair_checklist_20261103.md)、[docs/references.md](docs/references.md)（16 篇顶会/顶刊论文溯源与 BibTeX）与 [docs/working_context.md](docs/working_context.md)：

- **已完成（仿真与算法全链路闭环）**：
  1. **自主探索双层规划（`hybrid_frontier_v4.1`）**：8 分钟 30×30 m 真实 Gazebo 四路对比中观测面积达 **550.75 m²**（有效高程 495.06 m²）、0 次底盘原地转、0 次定位暂停、高程 RMSE 0.15 cm，见 [reports/v4_4way_comparison_20261122/README.md](reports/v4_4way_comparison_20261122/README.md)。
  2. **坡面安全换向（R1）**：实现意图分离（云台观察 + 平缓区过渡 / 有界曲率缓弯）、`SlopeFollowPath` 与无 `Spin` 行为树，在 `mooncar_ramp_v2.sdf` 8.83° 坡道 7/7 通过、换向位移降至 **6.29 cm**（下降 82.2%），见 [reports/R1_SLOPE_SAFE_VALIDATION_20261123.md](reports/R1_SLOPE_SAFE_VALIDATION_20261123.md)。
  3. **定位不确定度样本外校准（R4/R5）与弱特征接续（R7）**：接入基于累计估计里程的随机游走漂移校准模型，样本外 95% 覆盖率达 **95.9%–100.0%**（NEES 1.41–2.13），6 次全程 278 m 实测 0 次定位暂停，见 [reports/R4_R5_R7_UNCERTAINTY_CALIBRATION_20261123.md](reports/R4_R5_R7_UNCERTAINTY_CALIBRATION_20261123.md)。
  4. **启动就绪、足迹对齐与全量回归**：Nav2 生命周期 `/get_state` 就绪门、R5 足迹/膨胀对齐（`0.50×0.44 m` / `0.45 m`）、全量单元测试与 C11 固件 UBSan 纳入 `scripts/check_software.sh` 与 CI。
- **后续事项**：
  - **实物硬件集成（H1）**：待设备齐备后开展树莓派 5、RGBD/IMU 标定、二轴云台外参验证与 STM32 联调（参考 [docs/hardware_handoff.md](docs/hardware_handoff.md)）。
