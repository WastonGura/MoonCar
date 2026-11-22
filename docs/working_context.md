# Working Context

> 2026-11-05 及之前的详细历史记录已归档至 [docs/archive/working_context_20261105.md](archive/working_context_20261105.md)。本文件仅保留当前有效状态与待办事项。

## 当前有效基线（2026-11-21）

* **分支**：`dev/terrain-mapping-workbench`
* **默认配置**：`config/simulation_baseline.json`（`r5_support_v1` / `survey_v3_native` / `elevation_neighborhood_v1` / `rgbd_window` / `active_revisit: false` / `independent_linear_v1` / `0.25 ms`）
* **最近一次完整实验（2026-11-18）**：V3 地形 8 分钟固定版 vs 云台版自主探测对比，见 [reports/v3_camera_comparison_20261118/README.md](../reports/v3_camera_comparison_20261118/README.md)。
  * 固定版：477.6 仿真秒，28/28 动作，观测 488.44 m²（有效高程 432.12 m²），XY P95 5.37 cm，高程 RMSE 0.22 cm，0 暂停。
  * 云台版：480.0 仿真秒，22/23 动作，观测 499.88 m²（有效高程 456.25 m²），XY P95 8.23 cm，高程 RMSE 0.65 cm，0 暂停。
  * 两组均成功跨过原 500 万点上限实现 2 段连续测绘；均未触发弱几何重选（该恢复路径尚待定向复现验证）。
  * 成品视频与预览图已归档至本地 `output/v3-camera-comparison-20261118/`。

## 2026-11-21 仓库整理与环境清理

1. **磁盘瘦身**：归档 11-18 对比视频（`output/v3-camera-comparison-20261118/`）与 11-04 GitHub Issue 备份（`output/backups/github-issues-20261104/`）后，清理了 `reports/evidence/` 和 `tmp/` 下未跟踪的历史原始物理日志与旧 ROS bag，释放约 332.7 GB 磁盘空间；完整保留全部 Git 跟踪文件及测试引用的小型夹具。
2. **Git 卫生**：清理 12 个已失效的旧 `codex/*` worktree 记录、15 个过时本地 `codex/*` 分支及 worktree 空 `refs` 警告。
3. **启动脚本解耦**：[scripts/start_workbench.ps1](../scripts/start_workbench.ps1) 和 [scripts/start_r5_workbench.ps1](../scripts/start_r5_workbench.ps1) 移除对 Codex 内部缓存 Python 的硬编码，改为优先使用 `uv run --no-project`（支持 `MOONCAR_PYTHON` 及系统 `python`/`py` 回退）。
4. **文档清理**：修复 README 与文档中 110+ 处 `D:/Documents/Projects/Repositories/MoonCar-mapping/` 硬编码绝对路径，重写精简 [README.md](../README.md)，并将 2,608 行旧日志归档。
5. **测试与 CI 收口**：
   * 根目录 `tests/`（212 项，已将 `test_survey_wall_limit.py` 统一为 `unittest`）正式接入 [scripts/check_software.sh](../scripts/check_software.sh) 与 [.github/workflows/software-checks.yml](../.github/workflows/software-checks.yml)（补齐缺失依赖并监听 `dev/terrain-mapping-workbench`）。
   * 修复 `mooncar_gazebo` 历史遗留的 20 处 fail/error（NumPy `trapezoid`/`trapz` 兼容、R5 0.25 ms 步长断言、`image_dds_guard.py` 基线哈希同步、外部 `/diagnosis` 夹具显式 `skipUnless`、ROS witness 隔离本地发现），并将 5 个依赖 `reports/evidence/` 的测试夹具迁入 `ros2_ws/src/mooncar_gazebo/tests/fixtures/`；顺手修复 `scripts/check_r5_turn_control_round.py` 的缩进语法错误。
   * 容器全量回归结果：1,403 项 Python 单测（control 13 / navigation 10 / terrain 404 / slam 270 / gazebo 494 / root 212）+ C11 固件测试 + 全仓库 Python/Shell 语法检查 **全部通过（0 fail, 0 error）**。

## 2026-11-21 产品稳定性修复（三项高优 Bug）

1. **修复 Nav2 启动就绪竞态**：[scripts/experiment_motion.py](../scripts/experiment_motion.py) 现周期性异步轮询 `bt_navigator` / `controller_server` / `planner_server` / `behavior_server` 的 `/get_state` 生命周期服务（含超时取消与重试），仅当四个节点全部新鲜处于 `ACTIVE` 且动作服务就绪时才上报 `ready`（`spin_ready` 同步要求 `behavior_server` `ACTIVE`）；`reset()` 时清空状态并重建客户端，避免跨实验复用旧就绪态。
2. **修复实验退出残留孤儿 Gazebo 进程**：[scripts/simulation_experiment.py](../scripts/simulation_experiment.py) 的 `_stop()` 除信号原 launch 进程组外，还按实验唯一环境变量 `GZ_PARTITION=mooncar-<id>` 扫描 `/proc`，发现并按 `SIGINT → SIGTERM → SIGKILL` 清理独立会话（`setsid` 脱离原 PGID）的残留 `gz sim` 子进程，未清干净则明确拒绝启动新实验。
3. **切换默认地形策略为 `elevation_neighborhood_v1`**：统一更新 [terrain_node.py](../ros2_ws/src/mooncar_terrain/scripts/terrain_node.py)、[simulation_experiment.py](../scripts/simulation_experiment.py)、[config/simulation_baseline.json](../config/simulation_baseline.json) 与验证脚本默认值，保留 `MOONCAR_TERRAIN_STRATEGY=baseline` 显式回退通道。

## 2026-11-21 R5 模型清理与文献驱动自主探索策略 `hybrid_frontier_v4`

1. **R5 车体传感器与桥接清理**（不改动已冻结的悬架/轴距/摩擦与控制参数）：
   * [mooncar_r5.urdf.xacro](../ros2_ws/src/mooncar_description/urdf/mooncar_r5.urdf.xacro)：将 `camera_mount_joint` 抬升至 `z=0.145`（相机光心世界高度 `0.285 m`，高于电池顶 `0.227 m` 逾 5.8 cm、高于雷达扫描面 `0.2013 m` 逾 8.4 cm，消除前后向穿模与遮挡，保持质心 `x` 坐标不变）；`rgbd_v1` 模式下禁用重复的独立 `mooncar_rgb_camera` CPU 渲染（`legacy` 保持原样）。
   * [camera_sensor_profile.py](../ros2_ws/src/mooncar_gazebo/launch/camera_sensor_profile.py)：`rgbd_v1` 模式下将 `/mooncar/rgb/image` 与 `/mooncar/rgb/camera_info` 桥接重定向至 `/mooncar/localization_rgbd/*`；新增单测验证云台净空与单相机桥接（[test_camera_sensor_profile.py](../ros2_ws/src/mooncar_gazebo/tests/test_camera_sensor_profile.py)）。
2. **双层前沿探索 + 底盘/云台真解耦策略 `hybrid_frontier_v4`**：
   * 文献依据：Yamauchi (*CIRA 1997*, Frontier)、González-Baños & Latombe (*IJRR 2002*, Safe-Region NBV)、Bircher et al. (*ICRA 2016*, RH-NBV)、Selin et al. (*IEEE RA-L 2019*, AEP 局部 NBV + 全局前沿双层切换)、Zhou et al. (*IEEE RA-L 2021*, FUEL 前沿聚类 + 开放式 TSP 巡航) / Cao et al. (*RSS 2021*, TARE 分层探索)。
   * 实现：[scripts/frontier_planner.py](../scripts/frontier_planner.py) + [scripts/autonomous_survey.py](../scripts/autonomous_survey.py)（并在 [web/index.html](../web/index.html)、[web/app.js](../web/app.js) 与 [run_online_matrix_case.py](../scripts/validation/run_online_matrix_case.py) 接入选项，完整保留 `surface_view_v2` / `surface_view_v3` 基线）。关键改进：对齐 R5 真实相机位姿（`forward=0.183 m`, `z=0.285 m`, `HFOV=73°`）；同时提取占据未知前沿与雷达已清障但深度未观测前沿；全图降采样 Dijkstra + 2-opt TSP 前沿排序打破原 6 m 视野墙；移动后 `|arrival_pan| <= 135°` 直接使用云台偏航替代底盘原地 Spin、移动前若云台偏转超过 10° 自动回中保 VIO 前视、并用自适应驻留（0.8–3.0 s）替换固定 3 s 等待。
   * 单测与 30×30 m 离线闭环 480 s 对比基准（合成运动学基准，非 Gazebo 实测）：[test_hybrid_frontier_v4.py](../tests/test_hybrid_frontier_v4.py)（5/5 通过）；`LiDAR+Depth` 双传感器模式下 8 分钟覆盖面积达 **527.3 m²（58.6%）**，较 `surface_view_v3`（382.1 m² / 42.5%）提升 **+38.0%**、较 `surface_view_v2`（407.9 m² / 45.3%）提升 **+29.3%**，底盘原地旋转由 12 次降至 **0 次**，规划中位耗时由 1136 ms 降至 **633.5 ms（-44.2%）**；`Depth-only` 模式下达 **459.2 m²（51.0%）**，较 `v3`（364.4 m²）提升 **+26.0%**，旋转 15 → **0 次**。

3. **原始文献归档与 Nav2 R5 足迹对齐**：
   * [docs/references.md](references.md)：完整沉淀 9 篇核心原始论文（Yamauchi 1997 / González-Baños & Latombe 2002 / Bircher 2016 / Fankhauser 2018 / Selin 2019 / Macenski 2020 & 2023 / Zhou 2021 / Cao 2021）的已核实 DOI、BibTeX 与代码映射。
   * [nav2_profile.py](../ros2_ws/src/mooncar_navigation/launch/nav2_profile.py) + [nav2_params_r5.yaml](../ros2_ws/src/mooncar_navigation/config/nav2_params_r5.yaml)：按 `simulation_profile` 区分 `legacy`（`0.86×0.72 m` / 膨胀 `0.65 m`）与 `r5_support_v1`（`0.50×0.44 m` 与 `laser_filter_r5.yaml` 对齐 / 膨胀 `0.45 m > 外接半径 0.333 m` / `cost_scaling_factor = 4.0` / RPP `cost_scaling_dist = 0.45 m`），同步接入 `terrain_navigation.launch.py`、`navigation.launch.py`、`experiment_motion.py`（`within_region` 与半径护栏）、`simulation_experiment.py`、`observation_path.py` 与 `frontier_planner.py`（`0.45 m` 净空与边界裕度）。

## 2026-11-22 V3 地形四组 8 分钟全物理 Gazebo + 3D 录屏对比（含 `v4.1` 迭代）

- 完整报告与图表：[reports/v4_4way_comparison_20261122/README.md](../reports/v4_4way_comparison_20261122/README.md)（测试提交 `c86f127` / `0778a0f`，138 项安装文件前后核验 0 漂移；四路 2560×1260 并排对比视频及单路高清 MP4 归档至本地 `output/v4-4way-comparison-20261122/`）。
- **核心结果**（全组 100% 动作成功、0 定位暂停）：
  - **`hybrid_v4.1`（有云台，commit `0778a0f`，四道防碎片滤波 + `0.35 rad/s` 真实云台速率）**：观测 **550.75 m²** / 有效高程 **495.06 m²**（全场断层第一，较 `v4.0` 提升 **+33.4%**、较 `surface_v3` 高 **+9.2%**、较无云台版高 **+15.1%**）/ 后半程 **+259.8 m²**（全场第一，较 `v4.0` 暴涨 **3.35×**）/ **底盘原地自转 0 次（0.0 s）** / 云台耗时由 70.4 s 降至 **29.7 s** / **XY P95 5.94 cm**（云台组最佳）/ **全图高程 RMSE 0.15 cm** / **坡度 MAE 0.095°** / 27/27 动作、仅 1 次精准跨区切换。
  - `surface_v3_gimbal`：观测 **504.31 m²** / 有效高程 **463.31 m²** / RMSE **0.15 cm** / XY P95 **6.18 cm**（相比 11-18 旧 R5 同策略 `0.65 cm / 8.23 cm` 精度提升 4.3×）；但底盘原地转向 18 次（75.0 s）。
  - `coverage_v3_gimbal`：观测 **489.19 m²** / 有效 **447.31 m²** / RMSE **0.13 cm** / XY P95 **6.09 cm** / 22 次底盘转向（63.0 s）。
  - `hybrid_v4_fixed`（无云台固定俯角 `-28°`）：观测 **478.31 m²** / 有效 **448.69 m²** / 后半程 +235.9 m² / **XY P95 4.90 cm** / 坡度 MAE **0.082°** / 转向时间 13.8 s（比 `surface_v3` 降 81.6%）。
  - `hybrid_v4.0`（有云台初版，保留作消融基线）：观测 412.75 m² / 有效 384.62 m²（前半程 +317.8 m²，后半程因碎片侧视口袋与高估云台速度回落至 +77.5 m²），为 `v4.1` 迭代提供直接因果依据。

## 下一步计划

1. 推进坡面行驶换向（平缓区转向 / 缓弯，R1）与坡面弱几何重选 / 定位协方差校准（R4/R5/R7）。
2. 待实车硬件到齐后，按 [docs/hardware_handoff.md](hardware_handoff.md) 与 [docs/hardware_parameter_gaps.md](hardware_parameter_gaps.md) 开展树莓派、OAK-D-Lite EEPROM 标定、S3 雷达外参与 STM32 联调。
