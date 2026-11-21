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

## 下一步计划

1. 在 Docker Gazebo 真实 V3 地形上开展 `surface_view_v3` vs `hybrid_frontier_v4`（配合抬升云台）8 分钟实测 A/B 对比，验证高程覆盖、XY/高程精度与 CPU 负载。
2. 审视并按需收紧 Nav2 / `experiment_motion.py` 中仍沿用的旧车 `0.86×0.72 m` 轮廓与 `inflation_radius`（对比 R5 实际 `0.466×0.381 m`）。
3. 推进坡面行驶换向（平缓区转向 / 缓弯，R1）与坡面弱几何重选 / 定位协方差校准（R4/R5/R7）。
