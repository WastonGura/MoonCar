# Working Context

> 2026-11-05 及之前的详细历史记录已归档至 [docs/archive/working_context_20261105.md](archive/working_context_20261105.md)。本文件仅保留当前有效状态与待办事项。

## 当前有效基线（2026-11-21）

* **分支**：`dev/terrain-mapping-workbench`
* **默认配置**：`config/simulation_baseline.json`（`r5_support_v1` / `survey_v3_native` / `rgbd_window` / `active_revisit: false` / `independent_linear_v1` / `0.25 ms`）
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

## 下一步计划

回到小车本体优化（参照 [docs/repair_checklist_20261103.md](repair_checklist_20261103.md)）：
1. 修复工作台 `ready` 早于 Nav2 `ACTIVE` 的启动竞态。
2. 推进坡面行驶换向（平缓区转向 / 缓弯）接入或云台策略统一。
3. 定向验证弱几何定位重选与定位协方差校准。
