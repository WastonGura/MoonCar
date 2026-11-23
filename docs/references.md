# 参考文献与算法溯源（MoonCar / 寻迹者探测车）

> 本文件保存本项目在自主探索、地形可通行性评估、导航代价地图与路径跟踪控制中所参考的**原始学术论文**（含已核实的作者、发表期刊/会议、卷期页码、DOI 链接与 BibTeX），以及每篇论文与 MoonCar 代码模块、关键参数的对应关系，便于科创结题报告、论文撰写与评审溯源。

---

## 1. 自主探索与下一最佳视点（NBV / Frontier / Hierarchical Exploration）

### [1] Yamauchi (1997) — 前沿探索基础框架
- **完整引文**：B. Yamauchi, "A Frontier-Based Approach for Autonomous Exploration," *Proceedings of the 1997 IEEE International Symposium on Computational Intelligence in Robotics and Automation (CIRA '97)*, Monterey, CA, USA, 1997, pp. 146–151.
- **DOI**：<https://doi.org/10.1109/CIRA.1997.613851>
- **核心思想**：将“已知可通行栅格”与“未知栅格”之间的边界定义为前沿（Frontier），引导移动机器人持续向未知区域边界推进，直到所有可达前沿消解。
- **本项目映射**：[scripts/frontier_planner.py](../scripts/frontier_planner.py) 中 `extract_frontier_clusters()` 同时提取传统占据未知前沿（`cost < 0` 邻接可通行区）与“激光雷达已清障但深度相机尚未覆盖”的双重前沿。

### [2] González-Baños & Latombe (2002) — 安全区 NBV 收益与代价权衡
- **完整引文**：H. H. González-Baños and J.-C. Latombe, "Navigation Strategies for Exploring Indoor Environments," *The International Journal of Robotics Research (IJRR)*, vol. 21, no. 10–11, pp. 829–848, Oct. 2002.
- **DOI**：<https://doi.org/10.1177/0278364902021010834>
- **核心思想**：仅在当前已观测安全区域（Safe Region）内采样候选视点，将预期可见新信息增益与到达该视点的行驶代价做统一效用权衡，兼顾探索效率与安全约束。
- **本项目映射**：[scripts/frontier_planner.py](../scripts/frontier_planner.py) 中 `choose_hybrid_view()` 与 [scripts/autonomous_survey.py](../scripts/autonomous_survey.py) 均严格限制底盘目标位于已知安全代价地图内，并以“预期新增/改善面积 ÷ 动作耗时”作为 NBV 效用函数。

### [3] Bircher et al. (2016) — 滚动时域下一最佳视点（RH-NBV）
- **完整引文**：A. Bircher, M. Kamel, K. Alexis, H. Oleynikova, and R. Siegwart, "Receding Horizon 'Next-Best-View' Planner for 3D Exploration," *2016 IEEE International Conference on Robotics and Automation (ICRA)*, Stockholm, Sweden, 2016, pp. 1462–1468.
- **DOI**：<https://doi.org/10.1109/ICRA.2016.7487281>
- **核心思想**：在有限局部范围内反复规划下一最佳视点，执行首段动作后依据最新观测在线重规划，适配增量建图与未知障碍环境。
- **本项目映射**：`hybrid_frontier_v4` 局部层保留滚动重规划范式，每步基于实时更新的 `/global_costmap/costmap` 与地形质量图重新评估候选位姿与云台偏航。

### [4] Selin et al. (2019) — AEP 局部 NBV + 全局前沿双层切换
- **完整引文**：M. Selin, M. Tiger, D. Duberg, F. Heintz, and P. Jensfelt, "Efficient Autonomous Exploration Planning of Large-Scale 3-D Environments," *IEEE Robotics and Automation Letters (RA-L)*, vol. 4, no. 2, pp. 1699–1706, Apr. 2019.
- **DOI**：<https://doi.org/10.1109/LRA.2019.2897343>
- **核心思想**：纯局部 NBV 易陷入局部极小并遗漏远处未探区域，纯全局前沿又忽略近处高密度信息增益；AEP 将局部 RH-NBV 与全局前沿缓存结合，在局部信息增益低于阈值时无缝切换至全局目标。
- **本项目映射**：[scripts/frontier_planner.py](../scripts/frontier_planner.py) 中 `LOCAL_NEW_GAIN_SWITCH_M2 = 1.5` 与双层切换逻辑：局部有足够新覆盖时优先近处高性价比位姿与云台扫描，局部枯竭时立即切入全局前沿航路点，打破旧版 `v2/v3` 的 6 m 视野墙。

### [5] Zhou et al. (2021) — FUEL 前沿聚类与开放式 TSP 全局排序
- **完整引文**：B. Zhou, Y. Zhang, X. Chen, and S. Shen, "FUEL: Fast UAV Exploration Using Incremental Frontier Structure and Hierarchical Planning," *IEEE Robotics and Automation Letters (RA-L)*, vol. 6, no. 2, pp. 779–786, Apr. 2021.
- **DOI**：<https://doi.org/10.1109/LRA.2021.3051563>
- **核心思想**：将零散前沿单元聚类为紧凑簇并为每簇生成观测视点，通过求解开放式旅行商问题（Open TSP）决定全局访问顺序，显著减少大场地内的来回折返。
- **本项目映射**：[scripts/frontier_planner.py](../scripts/frontier_planner.py) 中 `extract_frontier_clusters()`（空间聚类）与 `order_frontier_tour()`（贪心最近邻 + 2-opt 开放式 TSP 巡航排序，含当前航向连续性偏置），并以低开销纯 NumPy/Python 在 CPU 上实时求解。

### [6] Cao et al. (2021) — TARE 分层局部精细 / 全局粗粒度探索
- **完整引文**：C. Cao, H. Zhu, H. Choset, and J. Zhang, "TARE: A Hierarchical Framework for Efficiently Exploring Complex 3D Environments," *Robotics: Science and Systems (RSS)*, Jul. 2021.
- **DOI**：<https://doi.org/10.15607/RSS.2021.XVII.018>
- **核心思想**：在大尺度环境中将局部高分辨率规划与全局粗粒度路径规划解耦，同时保持全局拓扑连续性，兼顾计算实时性与全局覆盖完整性。
- **本项目映射**：[scripts/frontier_planner.py](../scripts/frontier_planner.py) 对局部 NBV 使用 `stride=1` 精细 Dijkstra，对 30×30 m 全局前沿寻路采用 `stride=2` 降采样 Dijkstra（覆盖至 45 m 路径代价），并以全局主干方向引导局部候选偏好、单步航点截断至 `GLOBAL_WAYPOINT_STEP_M = 6.0 m` 以匹配 Nav2 20×20 m 滚动全局代价地图。

---

## 2. 地形高程建图与可通行性评估

### [7] Fankhauser, Bloesch & Hutter (2018) — 面向移动机器人的局部高程建图
- **完整引文**：P. Fankhauser, M. Bloesch, and M. Hutter, "Probabilistic Terrain Mapping for Mobile Robots With Uncertain Localization," *IEEE Robotics and Automation Letters (RA-L)*, vol. 3, no. 4, pp. 3019–3026, Oct. 2018.
- **DOI**：<https://doi.org/10.1109/LRA.2018.2849506>
- **核心思想**：以机器人为中心的 2.5D 高程栅格地图，结合邻域几何信息评估地表坡度与粗糙度，支撑复杂非结构化地面的可通行性判断。
- **本项目映射**：[ros2_ws/src/mooncar_terrain/scripts/terrain_node.py](../ros2_ws/src/mooncar_terrain/scripts/terrain_node.py) 默认启用的 `elevation_neighborhood_v1` 策略，基于邻域高程平面拟合与局部残差计算连续可通行代价图，避免单格噪点引发误阻断。

---

## 3. ROS 2 导航框架、代价地图膨胀与路径跟踪控制

### [8] Macenski et al. (2020) — Nav2 导航系统架构与代价地图设计
- **完整引文**：S. Macenski, F. Martín, R. White, and J. Ginés Clavero, "The Marathon 2: A Navigation System," *2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Las Vegas, NV, USA, 2020, pp. 2718–2725.
- **DOI**：<https://doi.org/10.1109/IROS45743.2020.9341207>（arXiv: <https://doi.org/10.48550/arXiv.2003.00368>）
- **核心思想**：基于 ROS 2 托管生命周期节点（Managed Lifecycle Nodes）与行为树（Behavior Tree）的可配置自主导航栈；代价地图膨胀半径须严格大于机器人外接圆半径（`inflation_radius > r_circumscribed`），以形成完整的内切致命区、外接警示区与平滑代价衰减梯度。
- **本项目映射**：
  - [scripts/experiment_motion.py](../scripts/experiment_motion.py)：异步轮询四个核心 Nav2 节点的 `/get_state` 生命周期状态，确保全部进入 `ACTIVE` 后才放行指令。
  - [ros2_ws/src/mooncar_navigation/config/nav2_params_r5.yaml](../ros2_ws/src/mooncar_navigation/config/nav2_params_r5.yaml) 与 [nav2_profile.py](../ros2_ws/src/mooncar_navigation/launch/nav2_profile.py)：按 R5 实车包络（`0.466 × 0.3811 m`）留裕度设定 `0.50 × 0.44 m` 足迹（与 `laser_filter_r5.yaml` 一致，内切半径 `0.22 m`、外接半径 `0.333 m`），设定 `inflation_radius = 0.45 m`（`> 0.333 m`）与 `cost_scaling_factor = 4.0`。

### [9] Macenski et al. (2023) — Regulated Pure Pursuit 路径跟踪控制器
- **完整引文**：S. Macenski, S. Singh, F. Martín, and J. Ginés, "Regulated Pure Pursuit for Robot Path Tracking," *Autonomous Robots*, vol. 47, pp. 685–694, 2023.
- **DOI**：<https://doi.org/10.1007/s10514-023-10097-6>（arXiv: <https://doi.org/10.48550/arXiv.2305.20026>）
- **核心思想**：在经典 Pure Pursuit 基础上引入曲率限速、障碍代价接近限速（`cost_scaling_dist`）与碰撞前瞻检查，改善狭窄及近障环境下的路径跟踪安全性。
- **本项目映射**：[ros2_ws/src/mooncar_navigation/config/nav2_params.yaml](../ros2_ws/src/mooncar_navigation/config/nav2_params.yaml) 与 [nav2_params_r5.yaml](../ros2_ws/src/mooncar_navigation/config/nav2_params_r5.yaml) 的 `controller_server.FollowPath`（`RegulatedPurePursuitController`），并将 R5 下的 `cost_scaling_dist` 对齐为 `0.45 m`；同时派生坡面专用的 `SlopeFollowPath`（关闭 `use_rotate_to_heading` 与倒车，启用曲率限速 `regulated_linear_scaling_min_radius = 0.90 m`）。

---

## 4. 坡面滑移力学与四轮滑移转向控制（Slope Locomotion & Skid-Steer Kinematics）

### [10] Ishigami et al. (2007) — 松软月壤行星探测车转向滑移土力学模型
- **完整引文**：G. Ishigami, A. Miwa, K. Nagatani, and K. Yoshida, "Terramechanics-Based Model for Steering Maneuver of Planetary Exploration Rovers on Loose Soil," *Journal of Field Robotics (JFR)*, vol. 24, no. 3, pp. 233–250, Mar. 2007.
- **DOI**：<https://doi.org/10.1002/rob.20187>
- **核心思想**：建立轮-壤三维接触应力与滑转/侧滑角（slip ratio / slip angle）动力学模型，定量揭示行星探测车在低重力与非平整松软坡面上转向时横向滑移与中心漂移的物理机理。
- **本项目映射**：解释并指导 R5 实车在 8.8° 坡面执行纯原地 `Spin` 产生约 30–35 cm 下坡偏移的机理分析，驱动 [scripts/slope_heading.py](../scripts/slope_heading.py) 将“观察视向调整”优先交由二轴云台完成，避免不必要的坡面底盘纯转。

### [11] Ishigami, Nagatani & Yoshida (2009) — 行星探测车坡面行驶与滑移补偿转向控制
- **完整引文**：G. Ishigami, K. Nagatani, and K. Yoshida, "Slope Traversal Controls for Planetary Exploration Rover on Sandy Terrain," *Journal of Field Robotics (JFR)*, vol. 26, no. 3, pp. 264–286, Mar. 2009.
- **DOI**：<https://doi.org/10.1002/rob.20277>
- **核心思想**：对比分析坡面横向行驶与转向中的下坡滑移抑制策略，指出在低牵引坡面上应优先采用大半径平顺曲率转移或选择低坡度过渡区换向，避免高曲率/原地转向诱发的失稳滑移。
- **本项目映射**：[scripts/slope_heading.py](../scripts/slope_heading.py) 中的 `find_gentle_turn_zone()`（优先沿起始朝向兼容路径驶入实测平缓区再转向）与 `plan_smooth_arc_maneuver()`（无平缓区时采用有界曲率前进缓弯 + 全程扫掠足迹核验）。

### [12] Rabiee & Biswas (2019) — 基于摩擦力的四轮滑移转向运动学模型
- **完整引文**：S. Rabiee and J. Biswas, "A Friction-Based Kinematic Model for Skid-Steer Wheeled Mobile Robots," *2019 International Conference on Robotics and Automation (ICRA)*, Montreal, QC, Canada, 2019, pp. 8563–8569.
- **DOI**：<https://doi.org/10.1109/ICRA.2019.8794216>
- **核心思想**：基于车轮-地面接触摩擦约束推导四轮滑移转向移动机器人（SSWMR）的瞬时旋转中心（ICR）分布规律，说明内侧车轮反转原地转向需要克服显著更大的侧向摩擦阻力矩。
- **本项目映射**：为 MoonCar 四轮差速底盘设置 `DEFAULT_ARC_RADII_M = (1.1, 1.5, 2.0, 2.5) m`（远大于半轮距 `~0.19 m`），确保坡面缓弯时四轮均保持正向滚动牵引而不发生内轮反转打滑。

### [13] Martínez et al. (2024) — 坡面四轮滑移转向车辆的 ICR 运动学
- **完整引文**：J. L. Martínez, J. Morales, M. Sánchez, and A. García-Cerezo, "ICR-based Kinematics for Wheeled Skid-Steer Vehicles on Firm Slopes," *2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Abu Dhabi, UAE, 2024, pp. 1322–1328.
- **DOI**：<https://doi.org/10.1109/IROS58592.2024.10801776>
- **核心思想**：将 ICR 滑移转向运动学推广至倾斜坡面，证明重力沿坡分量会使原地转向的瞬时旋转中心明显向下坡方向偏移，导致无外部位姿约束的纯旋转必然伴随下坡侧移。
- **本项目映射**：支撑 [ros2_ws/src/mooncar_navigation/config/behavior_trees/navigate_to_pose_slope_safe.xml](../ros2_ws/src/mooncar_navigation/config/behavior_trees/navigate_to_pose_slope_safe.xml)（移除坡面导航行为树中的 `Spin` 与 `BackUp` 恢复动作）与 [terrain_navigation.launch.py](../ros2_ws/src/mooncar_navigation/launch/terrain_navigation.launch.py) 中的 `SlopeFollowPath`（禁用隐式 `use_rotate_to_heading`），同时保留平地 `FollowPath` 完整转向能力。

---

## 5. 滑动窗口视觉里程计与定位不确定度一致性校准（VO Uncertainty & NEES Consistency）

### [14] Bailey et al. (2006) — SLAM 协方差一致性与 NEES 检验
- **完整引文**：T. Bailey, J. Nieto, J. Guivant, M. Stevens, and E. Nebot, "Consistency of the EKF-SLAM Algorithm," *2006 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Beijing, China, 2006, pp. 3562–3568.
- **DOI**：<https://doi.org/10.1109/IROS.2006.281644>
- **核心思想**：系统定义并运用归一化估计误差平方（Normalized Estimation Error Squared, NEES）检验定位估计器协方差矩阵的一致性，揭示忽视线性化与历史相关性时协方差会显著乐观（overconfident）。
- **本项目映射**：[scripts/offline_rgbd_window/uncertainty.py](../scripts/offline_rgbd_window/uncertainty.py) 中 `evaluate_nees_coverage()` 与 `OdometryDriftUncertainty`，针对 `rgbd_window` 原始条件协方差 NEES 严重偏高（16.3–56.3，95% 覆盖率仅 4.6%–34.3%）的问题，构建并样本外验证基于累计估计里程的随机游走漂移校准模型。

### [15] Leutenegger et al. (2015) — 基于关键帧与舒尔补边缘化的滑动窗口视觉惯性里程计（OKVIS）
- **完整引文**：S. Leutenegger, S. Lynen, M. Bosse, R. Siegwart, and P. Furgale, "Keyframe-Based Visual–Inertial Odometry Using Nonlinear Optimization," *The International Journal of Robotics Research (IJRR)*, vol. 34, no. 3, pp. 314–334, Mar. 2015.
- **DOI**：<https://doi.org/10.1177/0278364914554813>
- **核心思想**：采用固定窗口关键帧优化并通过舒尔补（Schur complement）边缘化旧帧与路标形成先验信息矩阵，实现有界计算复杂度下的连续位姿估计；同时指出无全局回环时窗口边缘化仅给出局部条件精度，不可替代全局漂移界。
- **本项目映射**：[scripts/offline_rgbd_window/core.py](../scripts/offline_rgbd_window/core.py) 中 `WindowEstimator._marginalize_oldest()` 与 [scripts/rgbd_window_odometry.py](../scripts/rgbd_window_odometry.py) 显式区分 `conditional_position_std_m`（局部条件标准差）与 `calibrated_xy_bound_95_m`（含累计漂移的 95% 全局误差界）。

### [16] Zhang & Scaramuzza (2018) — 视觉（惯性）里程计轨迹误差定量评测规范
- **完整引文**：Z. Zhang and D. Scaramuzza, "A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry," *2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Madrid, Spain, 2018, pp. 7244–7251.
- **DOI**：<https://doi.org/10.1109/IROS.2018.8593941>
- **核心思想**：规范视觉里程计绝对轨迹误差（ATE）与相对位姿误差（RPE）随行驶路程增长的定量分解方法，强调在独立测试集上做跨场景泛化评估。
- **本项目映射**：[reports/R4_R5_R7_UNCERTAINTY_CALIBRATION_20261123.md](../reports/R4_R5_R7_UNCERTAINTY_CALIBRATION_20261123.md) 在 6 次真实 Gazebo 全程实测（278 m 累计里程）中分离 4 组训练集与 2 组样本外验证集（`hybrid_v4.1` 58.74 m 平场 + `r1_ramp` 8.83° 坡道），实测样本外 95% 覆盖率分别达 **95.9%**（NEES 2.13）与 **100.0%**（NEES 1.41）。

---

## 6. BibTeX 汇总（可直接用于论文 / 科创结题报告）

```bibtex
@inproceedings{yamauchi1997frontier,
  author    = {Yamauchi, Brian},
  title     = {A Frontier-Based Approach for Autonomous Exploration},
  booktitle = {Proceedings 1997 IEEE International Symposium on Computational Intelligence in Robotics and Automation (CIRA'97)},
  year      = {1997},
  pages     = {146--151},
  doi       = {10.1109/CIRA.1997.613851}
}

@article{gonzalezbanos2002navigation,
  author  = {Gonz{\'a}lez-Ba{\~n}os, H{\'e}ctor H. and Latombe, Jean-Claude},
  title   = {Navigation Strategies for Exploring Indoor Environments},
  journal = {The International Journal of Robotics Research},
  year    = {2002},
  volume  = {21},
  number  = {10--11},
  pages   = {829--848},
  doi     = {10.1177/0278364902021010834}
}

@inproceedings{bircher2016receding,
  author    = {Bircher, Andreas and Kamel, Mina and Alexis, Kostas and Oleynikova, Helen and Siegwart, Roland},
  title     = {Receding Horizon "Next-Best-View" Planner for 3D Exploration},
  booktitle = {2016 IEEE International Conference on Robotics and Automation (ICRA)},
  year      = {2016},
  pages     = {1462--1468},
  doi       = {10.1109/ICRA.2016.7487281}
}

@article{fankhauser2018probabilistic,
  author  = {Fankhauser, P{\'e}ter and Bloesch, Michael and Hutter, Marco},
  title   = {Probabilistic Terrain Mapping for Mobile Robots With Uncertain Localization},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2018},
  volume  = {3},
  number  = {4},
  pages   = {3019--3026},
  doi     = {10.1109/LRA.2018.2849506}
}

@article{selin2019efficient,
  author  = {Selin, Magnus and Tiger, Mattias and Duberg, Daniel and Heintz, Fredrik and Jensfelt, Patric},
  title   = {Efficient Autonomous Exploration Planning of Large-Scale 3-D Environments},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2019},
  volume  = {4},
  number  = {2},
  pages   = {1699--1706},
  doi     = {10.1109/LRA.2019.2897343}
}

@inproceedings{macenski2020marathon2,
  author    = {Macenski, Steve and Mart{\'i}n, Francisco and White, Ruffin and Gin{\'e}s Clavero, Jonatan},
  title     = {The Marathon 2: A Navigation System},
  booktitle = {2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2020},
  pages     = {2718--2725},
  doi       = {10.1109/IROS45743.2020.9341207}
}

@article{zhou2021fuel,
  author  = {Zhou, Boyu and Zhang, Yichen and Chen, Xinyi and Shen, Shaojie},
  title   = {{FUEL}: Fast {UAV} Exploration Using Incremental Frontier Structure and Hierarchical Planning},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2021},
  volume  = {6},
  number  = {2},
  pages   = {779--786},
  doi     = {10.1109/LRA.2021.3051563}
}

@inproceedings{cao2021tare,
  author    = {Cao, Chao and Zhu, Hongbiao and Choset, Howie and Zhang, Ji},
  title     = {{TARE}: A Hierarchical Framework for Efficiently Exploring Complex 3D Environments},
  booktitle = {Robotics: Science and Systems (RSS)},
  year      = {2021},
  doi       = {10.15607/RSS.2021.XVII.018}
}

@article{macenski2023regulated,
  author  = {Macenski, Steve and Singh, Shrijit and Mart{\'i}n, Francisco and Gin{\'e}s, Jonatan},
  title   = {Regulated Pure Pursuit for Robot Path Tracking},
  journal = {Autonomous Robots},
  year    = {2023},
  volume  = {47},
  pages   = {685--694},
  doi     = {10.1007/s10514-023-10097-6}
}

@article{ishigami2007terramechanics,
  author  = {Ishigami, Genya and Miwa, Akiko and Nagatani, Keiji and Yoshida, Kazuya},
  title   = {Terramechanics-Based Model for Steering Maneuver of Planetary Exploration Rovers on Loose Soil},
  journal = {Journal of Field Robotics},
  year    = {2007},
  volume  = {24},
  number  = {3},
  pages   = {233--250},
  doi     = {10.1002/rob.20187}
}

@article{ishigami2009slope,
  author  = {Ishigami, Genya and Nagatani, Keiji and Yoshida, Kazuya},
  title   = {Slope Traversal Controls for Planetary Exploration Rover on Sandy Terrain},
  journal = {Journal of Field Robotics},
  year    = {2009},
  volume  = {26},
  number  = {3},
  pages   = {264--286},
  doi     = {10.1002/rob.20277}
}

@inproceedings{rabiee2019friction,
  author    = {Rabiee, Sadegh and Biswas, Joydeep},
  title     = {A Friction-Based Kinematic Model for Skid-Steer Wheeled Mobile Robots},
  booktitle = {2019 International Conference on Robotics and Automation (ICRA)},
  year      = {2019},
  pages     = {8563--8569},
  doi       = {10.1109/ICRA.2019.8794216}
}

@inproceedings{martinez2024icr,
  author    = {Mart{\'i}nez, Jorge L. and Morales, Jes{\'u}s and S{\'a}nchez, Manuel and Garc{\'i}a-Cerezo, Alfonso},
  title     = {{ICR}-based Kinematics for Wheeled Skid-Steer Vehicles on Firm Slopes},
  booktitle = {2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2024},
  pages     = {1322--1328},
  doi       = {10.1109/IROS58592.2024.10801776}
}

@inproceedings{bailey2006consistency,
  author    = {Bailey, Tim and Nieto, Juan and Guivant, Jose and Stevens, Michael and Nebot, Eduardo},
  title     = {Consistency of the {EKF-SLAM} Algorithm},
  booktitle = {2006 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2006},
  pages     = {3562--3568},
  doi       = {10.1109/IROS.2006.281644}
}

@article{leutenegger2015keyframe,
  author  = {Leutenegger, Stefan and Lynen, Simon and Bosse, Michael and Siegwart, Roland and Furgale, Paul},
  title   = {Keyframe-Based Visual--Inertial Odometry Using Nonlinear Optimization},
  journal = {The International Journal of Robotics Research},
  year    = {2015},
  volume  = {34},
  number  = {3},
  pages   = {314--334},
  doi     = {10.1177/0278364914554813}
}

@inproceedings{zhang2018tutorial,
  author    = {Zhang, Zichao and Scaramuzza, Davide},
  title     = {A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry},
  booktitle = {2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2018},
  pages     = {7244--7251},
  doi       = {10.1109/IROS.2018.8593941}
}
```
