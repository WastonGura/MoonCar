# MoonCar（寻迹者探测车）仿真模型与核心参数清单

> 本文档汇总当前默认仿真配置（`r5_support_v1` + `rgbd_window` + `elevation_neighborhood_v1` + `hybrid_frontier_v4.1`）的**全部精确数值及源码出处**，便于论文实验设置（Experimental Setup）撰写、参数核查与真机标定对照。
>
> **诚实边界声明**：下表所有动力学、悬架、摩擦与传感器噪声均为**仿真研究参数**（`hardware_calibrated = false`），用于算法对比与物理一致性检验，不等价于已标定的实物硬件参数；实车部署前须逐项实测标定。

---

## 1. 仿真环境与物理引擎

| 参数项 | 数值 | 说明与源码位置 |
|---|---|---|
| 物理引擎 | Gazebo Sim + 定制 DART (`libmooncar-physics7-dartsim.so`) | `/opt/mooncar/turn/r5-v2`，见 [config/simulation_baseline.json](../config/simulation_baseline.json) |
| 物理仿真步长 | **`0.25 ms`（4000 Hz）** | R5 运行时覆写 `physics_step_s = 0.00025`（世界 SDF 默认 1 ms） |
| 重力加速度 | **`(0, 0, -1.62) m/s²`** | 月面重力，见 [mooncar_survey_v3.sdf](../ros2_ws/src/mooncar_gazebo/worlds/mooncar_survey_v3.sdf) / [mooncar_ramp_v2.sdf](../ros2_ws/src/mooncar_gazebo/worlds/mooncar_ramp_v2.sdf) |
| 主测绘场景 | `mooncar_survey_v3`（30 × 30 m 连续无缝月面） | PBR 渲染引擎 `ogre2`，含月壤、软沙、碎石、岩块 |
| 坡道验证场景 | `mooncar_ramp_v2`（16 × 16 m，中段约 `8.83°` 斜坡 + 平缓坡顶） | 见 [scripts/validation/ramp_fixture.py](../scripts/validation/ramp_fixture.py) |
| 常规月壤摩擦 (`regolith_passable`) | `mu = 0.80`, `mu2 = 0.72` | 见 [mooncar_survey_v3.sdf](../ros2_ws/src/mooncar_gazebo/worlds/mooncar_survey_v3.sdf) |
| 松软沙地摩擦 (`sand_soft`) | `mu = 0.52`, `mu2 = 0.48` | 同上 |
| 碎石砾石摩擦 (`rubble_gravel`) | `mu = 0.90`, `mu2 = 0.82` | 同上 |
| 岩块地面摩擦 (`rock_boulder`) | `mu = 1.00`, `mu2 = 0.92` | 同上 |

---

## 2. 车体几何、质量与独立悬架（`mooncar_r5.urdf.xacro`）

| 参数项 | 数值 | 说明与源码位置 |
|---|---|---|
| 整车总质量 | **`9.186 kg`** | 底盘 `4.10 kg` + 四轮 `4×0.40 kg` + 电池 `3.05 kg` + 雷达 `0.115 kg` + 相机 `0.061 kg` + IMU `0.080 kg` + 云台组件 `0.180 kg` |
| 整车物理外包络（长 × 宽） | **`0.466 m × 0.3811 m`** | 轮外沿长 `2×0.157 + 2×0.076`；轮外沿宽 `2×0.16555 + 0.05`（含轮帽视觉宽 `0.4011 m`） |
| 车体主箱尺寸（长 × 宽 × 高） | `0.320 × 0.250 × 0.080 m` | 见 [mooncar_r5.urdf.xacro](../ros2_ws/src/mooncar_description/urdf/mooncar_r5.urdf.xacro) |
| 离地间隙 / `base_link` 中心高 | `0.060 m` / `0.100 m` | `base_link_z_offset = 0.06 + 0.08/2` |
| 轴距 / 轮距 | **`0.314 m`** / **`0.3311 m`** | `wheel_x_offset = ±0.157 m`, `wheel_y_offset = ±0.16555 m` |
| 车轮直径 / 宽度 | **`0.152 m`**（半径 `0.076 m`）/ **`0.050 m`** | 四轮滑移差速驱动（Skid-Steer） |
| 轮胎各向异性摩擦 | 侧向 `mu1 = 0.20`，纵向 `mu2 = 0.85` | `fdir1 = (0, 0, 1)` 沿轮轴方向，见 [rover_macros.xacro](../ros2_ws/src/mooncar_description/urdf/common/rover_macros.xacro) |
| 独立悬架型式 (`independent_linear_v1`) | 4 轮独立垂向滑移副（Prismatic `Z` 轴） | 见 [r5_suspension.xacro](../ros2_ws/src/mooncar_description/urdf/common/r5_suspension.xacro) |
| 悬架行程 / 预压参考 / 阻尼比 | `±20 mm`（`[-0.02, 0.02] m`） / `-10 mm` / $\zeta = 0.7$ | 最大输出力 `50 N`，限速 `0.5 m/s` |
| 前轮刚度 / 阻尼 | `k = 292.45 N/m`, `c = 32.16 N·s/m` | 按前轮静载 `2.925 N`（月面重力 `1.62 m/s²`）推导 |
| 后轮刚度 / 阻尼 | `k = 322.01 N/m`, `c = 35.41 N·s/m` | 按后轮静载 `3.220 N`（电池重心位于 `x = -0.035 m`）推导 |
| 底盘驱动插件限幅 (`DiffDrive`) | 线速 `±0.35 m/s`，角速 `±0.80 rad/s`，线加 `±0.50 m/s²`，角加 `±1.00 rad/s²` | 见 [gazebo_drive.xacro](../ros2_ws/src/mooncar_description/urdf/common/gazebo_drive.xacro) |

---

## 3. 二轴观测云台与车载传感器

| 参数项 | 数值 | 说明与源码位置 |
|---|---|---|
| 云台光学中心高度（距地） | **`z = 0.285 m`**（`x = 0.183 m`） | 高于电池顶 `0.227 m` 与雷达扫描面 `0.2013 m`，消除全行程遮挡 |
| 云台偏航（Pan）行程 | **`±135°`**（`[-2.356, +2.356] rad`） | 绕 `Z` 轴旋转，见 [mooncar_r5.urdf.xacro](../ros2_ws/src/mooncar_description/urdf/mooncar_r5.urdf.xacro) |
| 云台俯仰（Tilt）行程 / 默认巡航角 | **`-35° ~ +15°`**（`[-0.611, +0.262] rad`） / **`-15°`**（`-0.2618 rad`） | 绕 `-Y` 轴旋转 |
| 云台关节控制器与软件限速 | Gazebo PID `P=20, I=0.1, D=0.5`（力矩限幅 `5 N·m`）；运行限速 **`0.35 rad/s`** | 约 `20°/s`，防止快速旋转引起视觉运动模糊；关节反馈 `100 Hz` |
| RGB-D 深度相机（对标 OAK-D Lite） | **`640 × 480`**, `HFOV = 73°`, 帧率 `10 Hz`, 有效裁剪 `0.35 ~ 8.0 m` | `rgbd_v1` 对齐渲染（理想针孔模型，无镜头畸变），见 [gazebo_sensors.xacro](../ros2_ws/src/mooncar_description/urdf/common/gazebo_sensors.xacro) |
| IMU（对标 BNO055） | **`100 Hz`**，角速度 $\sigma = 0.0002\text{ rad/s}$，线加速度 $\sigma = 0.017\text{ m/s}^2$ | 占位白噪声，未建模温漂与零偏游走 |
| 2D 激光雷达（对标 SLAMTEC S3） | `10 Hz`，`3200` 束/圈，`360°`，测距 `0.05 ~ 15.0 m`，$\sigma = 0.01\text{ m}$ | **`rgbd_window` 三维模式下不参与定位与障碍层**，仅保留用于历史 2D 模式对照 |

---

## 4. 定位、地形评估、探索规划与 Nav2 控制参数

| 子系统 | 关键参数与数值 | 源码位置 |
|---|---|---|
| **滑动窗口视觉定位（`rgbd_window`）** | 窗口 `6` 关键帧、每帧上限 `96` 观测、跟踪预算 `576` 特征、预热 `96` 路标、关键帧间隔 `0.5 s`；R5 样本外漂移校准 $\sigma_0 = 8.58\text{ mm}, \alpha_s = 5.22\text{ mm}/\sqrt{\text{m}}$（95% 界系数 `2.4477`） | [core.py](../scripts/offline_rgbd_window/core.py) · [uncertainty.py](../scripts/offline_rgbd_window/uncertainty.py) |
| **定位健康互锁门** | 连续 `3` 帧真实解算就绪，最大观测龄 `0.3 s`，状态墙钟超时 `0.5 s`（失效立即撤销运动许可） | [localization_quality.py](../scripts/localization_quality.py) |
| **地形高程与风险评估（`elevation_neighborhood_v1`）** | 高程/可通行分辨率 `0.20 m`（采样质量格 `0.25 m`）；缓坡起算 `5.0°` / 禁行 `18.0°`；粗糙度起算 `0.015 m` / 禁行 `0.040 m`；相邻平面台阶 `0.18 m`、落差 `0.12 m` | [terrain_node.py](../ros2_ws/src/mooncar_terrain/scripts/terrain_node.py) |
| **Nav2 代价地图（R5）** | 足迹 **`0.50 × 0.44 m`**（内切 `0.22 m` / 外接 `0.333 m`），膨胀半径 **`0.45 m`**，`cost_scaling_factor = 4.0`；全局滚动图 `20 × 20 m @ 0.10 m`，局部 `4 × 4 m @ 0.05 m` | [nav2_params_r5.yaml](../ros2_ws/src/mooncar_navigation/config/nav2_params_r5.yaml) · [nav2_profile.py](../ros2_ws/src/mooncar_navigation/launch/nav2_profile.py) |
| **平地路径跟踪与独立转向** | `FollowPath` (RPP)：`desired_linear_vel = 0.16 m/s`, `lookahead = 0.30~0.80 m`, `cost_scaling_dist = 0.45 m`, `use_rotate_to_heading = true`；独立 `Spin`：`max_rotational_vel = 0.30 rad/s`, `acc_lim = 0.40 rad/s²` | [nav2_params_r5.yaml](../ros2_ws/src/mooncar_navigation/config/nav2_params_r5.yaml) |
| **R1 坡面安全换向与控制器** | 坡面禁转阈值 **`5.0°`**（粗糙度 `≤ 0.025 m`），起步偏角 `≤ 35°`，前进缓弯半径 `R ∈ {1.1, 1.5, 2.0, 2.5} m`；`SlopeFollowPath`：`desired_linear_vel = 0.14 m/s`, `use_rotate_to_heading = false`, `allow_reversing = false`, `regulated_linear_scaling_min_radius = 0.90 m`，行为树移除 `Spin` / `BackUp` | [slope_heading.py](../scripts/slope_heading.py) · [terrain_navigation.launch.py](../ros2_ws/src/mooncar_navigation/launch/terrain_navigation.launch.py) · [navigate_to_pose_slope_safe.xml](../ros2_ws/src/mooncar_navigation/config/behavior_trees/navigate_to_pose_slope_safe.xml) |
| **自主探索规划（`hybrid_frontier_v4.1`）** | 局部/全局切换阈值 `1.5 m²`、全局单步航点 `6.0 m`、宏未知半径 `2.0 m` / 面积门槛 `≥ 3.5 m²`（`≥ 10` 格）、视点增益门槛 `≥ 1.5 m²`、`>90°` 回溯滞后权重 `4.5`、云台往返限速 `0.35 rad/s`、R5 净空 `0.45 m` | [frontier_planner.py](../scripts/frontier_planner.py) |
