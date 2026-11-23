# R1 坡面安全换向（Slope-Safe Heading Adjustment）真实 Gazebo 物理验证报告

- **日期**：2026-11-23
- **代码版本**：`dev/terrain-mapping-workbench`（提交 `621cf67`，139 项安装文件通过 `runtime_baseline.py verify` 校验）
- **仿真配置**：R5 固定支撑（`r5_support_v1`）、`rgbd_window` 窗口定位、独立 Gazebo 物理引擎（`mooncar_ramp_v2.sdf`，月面重力 `1.62 m/s²`，未修改摩擦/悬架/包络/定位门，控制环零真值输入）
- **3D 录屏归档**：[r1-slope-safe-ramp.mp4](../output/r1-slope-safe-validation-20261123/r1-slope-safe-ramp.mp4)（1120×760，2,265 帧，4.14 MB）

## 1. 算法与架构摘要

严格落实 [docs/slope_heading_alternative_20261104.md](../docs/slope_heading_alternative_20261104.md) 并结合 Ishigami et al. (JFR 2007/2009)、Rabiee & Biswas (ICRA 2019) 与 Martínez et al. (IROS 2024) 坡面滑移转向运动学（完整引文与 BibTeX 见 [docs/references.md](../docs/references.md) §4）：

1. **核心规划模块**（[scripts/slope_heading.py](../scripts/slope_heading.py)）：
   - `assess_local_slope()`：综合实测地形质量图（`gx, gy, slope, roughness, fit_state`）与实测车体姿态倾角，默认平缓转向包络 `≤ 5.0°`、粗糙度 `≤ 0.025 m`。
   - `plan_heading_adjustment()`：按任务意图分离 `observe_heading`（优先二轴云台 + 三维俯仰补偿，零底盘滑移）与 `travel_heading`（平地保持原原地转向；坡面先搜索起始朝向兼容 `≤ 35°` 的实测平缓转向区，无平缓区时评估有界曲率前进缓弯 `R ∈ {1.1, 1.5, 2.0, 2.5} m` + 全程扫掠足迹与未知/障碍核验，否则显式安全拒发）。
2. **任务级坡面 Nav2 控制器与行为树**（不破坏平地默认 `FollowPath` 与 `navigate_to_pose_workbench.xml`）：
   - [terrain_navigation.launch.py](../ros2_ws/src/mooncar_navigation/launch/terrain_navigation.launch.py)：派生 `SlopeFollowPath`（`use_rotate_to_heading: false`、`allow_reversing: false`、`desired_linear_vel: 0.14 m/s`、`regulated_linear_scaling_min_radius: 0.90 m`、`max_angular_accel: 2.0`）。
   - [navigate_to_pose_slope_safe.xml](../ros2_ws/src/mooncar_navigation/config/behavior_trees/navigate_to_pose_slope_safe.xml)：移除 `Spin` 与 `BackUp` 恢复节点，仅保留清图与短等待，防止坡面过渡途中隐式触发原地旋转或盲目下坡倒车。
3. **单测与回归**：`test_slope_heading.py`（9/9）、`test_hybrid_frontier_v4.py`（9/9）、`mooncar_navigation/tests`（15/15，含 ROS 2 Launch 实测）、`mooncar_terrain/tests`（408/408）全部通过。

## 2. 真实 Gazebo 坡道试验结果（`mooncar_ramp_v2.sdf`，`slope_safe_turn`）

固定路线：`[0,0] → (1.0,0) → (2.5,0) → (4.0,0) → observe_heading(+90°) → observe_heading(0°) → travel_heading(180°) → 下坡返回 (4.0,0)`。

| 指标 | 历史基线：坡面纯原地 `Spin`（`half_spin` / R5 复测） | 本轮 R1：`slope_safe_turn`（真实 Gazebo） | 对比结论 |
|---|---:|---:|---|
| 任务完成度 | 坡中滑移明显；第 6 动作曾因弱特征暂停 | **7/7（100%）完成**（含下坡返回），`115.7 sim s` | 全流程闭环通过 |
| 定位暂停次数 | 1 次（坡面） | **0 次** | 连续稳定无断链 |
| 侧向观察底盘位移（`-9.27°` 坡面） | `35.3 / 30.0 cm`（半速纯转） | **`0.01 mm`**（`gimbal_observe`） | 零底盘滑移 |
| 坡面 `180°` 行驶换向策略 | 坡面强行原地 `Spin` | 自动识别 `8.827° > 5.0°`，经 `SlopeFollowPath` 驶入坡顶平缓区 `(7.05, 0.95)` 后复核实测 `2.534°` 再转向 | 彻底规避陡坡原地转 |
| 驶入平缓区途中隐式旋转 / 倒车指令 | N/A | `/mooncar/cmd_vel_safe`: **前进 2,137 条，倒车 0 条，纯原地转 0 条**（`v ∈ [0.058, 0.140] m/s`, `max|ω| = 0.156 rad/s`） | 100% 无隐式 `Spin` |
| 换向阶段真实中心净位移（真值对齐） | `32.2 ~ 35.3 cm`（8.8° 坡面） | **`6.29 cm`**（最大 `6.45 cm`，144° 掉头至 `-179.1°`，航向误差 `0.33°`） | **净位移下降 82.2%** |
| 全程独立真值定位误差（31,805 帧对齐） | 坡面有输出段 XY P95 `4.04 cm` | **XY 中位 `1.25 cm` / P95 `3.44 cm` / Max `3.55 cm`**；Yaw P95 `0.03°` | 精度优良且全程有界 |

## 3. 逐阶段实测明细（`Action 5: travel_heading → π`）

1. **坡面感知**：在坡中 `(3.783, 0.067)` 实测局部坡度 **`8.827°`**（16/16 有效表面格，实车真值俯仰 `-9.27°`），触发 `in_place_turn_safe = False`。
2. **`stage_transit`（61.9–87.6 sim s）**：经 `SlopeFollowPath` + `navigate_to_pose_slope_safe.xml` 沿起始朝向兼容路径（起步偏角 `7.71°`、最大折角 `26.57°`）连续上坡 `3.19 m` 至坡顶 `(6.859, 0.798)`，真值俯仰由 `-9.27°` 平滑过渡至 `-0.99°`。
3. **`verify_gentle_settled`（87.6–88.6 sim s）**：停稳 1 sim s（真值漂移 `0.30 cm`），复核实测地形坡度 **`2.534° ≤ 5.0°`**（17/17 有效格，距计划点 `0.236 m`），确认进入安全转向包络。
4. **`reorient_in_gentle_zone`（88.6–98.5 sim s）**：在平缓坡顶从 `36.8°` 平稳转向至 `-179.1°`（真值净位移 `6.29 cm`），随后 Action 6 顺利下坡返回 `(4.0, 0.0)`。

## 4. 边界说明

- 本轮实机仿真直接验证了 `observe_heading → gimbal_observe` 与 `travel_heading → gentle_zone_staging (SlopeFollowPath) → verify_gentle_settled → reorient_in_gentle_zone` 全链路；无邻近平缓区时的有界曲率缓弯（`smooth_arc_maneuver`）及不可行安全拒发（`refused_unsafe_slope_turn`）已由 `tests/test_slope_heading.py` 单测严格覆盖，后续扩展多坡道场景时可补充专项仿真对照。
