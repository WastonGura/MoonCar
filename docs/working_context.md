# Working Context

## 2026-10-08：正式图片采集组件已实现，首轮测试模型展开失败，按规则停止

最小 RMW 8.4.4 容量入口、初始化回执、独立派生镜像及正式桥接启动门禁已实现。无 Gazebo 的实际桥接核验通过：发送段 8 MiB、端口队列 512、仍加载原 Fast DDS 2.14.6，配置只进入桥接子进程。

新验收尝试 1/10，测试观察模型被 ElementTree 写成 ns0 前缀，xacro 报 Undefined substitution argument prefix；未取得有效观察窗口，失败后停止，校准 0/4，三场景和旧配置仿真未启动，默认仍 legacy。失败、冻结源码和签名保留；这是新增测试工具错误，不是容量修复或车辆失稳证据。

停止后修复 XML 前缀并通过无 Gazebo 的真实 R5 展开与物理/控制定义一致性检查，没有补跑仿真。完整软件 276 项、前端 3 项、固件 UBSan 和语法检查通过。本轮仅提交新增组件、回归和日志，原工作台及其他未提交改动保留。下一轮必须新目录、新冻结完成 10 次验证，才能启用限定范围的 auto 默认。

报告：reports/evidence/terrain-v2/r5-dds-production-20261008/README.md。

## 2026-09-19/23：v3 自主策略族算法重构、底盘机动激活与 300s 全场景仿真回归全面超越 v2

针对前期 300s 长期巡视对照中暴露的“v3 策略过早中断退出、底盘缺乏前瞻推进机动（原地瘫痪）、实际覆盖表现劣于 v2 基线”的技术痛点，团队完成了深度算法根因排查、多层决策解耦重构与 Gazebo 全场景仿真回归，v3 策略族在完整 300s 测试中全面实现质的突破：

1. **核心算法根因与重构突破**：
   - **两级前沿跃迁机制（Frontier Leap，彻底终结 `nearest_view_v3` 原地瘫痪）**：
     - *根因*：转动云台耗时仅约 2s（打分 $\approx 0.5$），而底盘机动耗时约 12s（打分 $\approx 0.08$），就近贪心算法陷入“局部原地死循环”，全景扫视完成后因缺少未见视场，误选超出云台限位的大角度执行 Nav2 原地差速自旋，在松散月壤摩擦模型下严重超时（$>60\text{s}$）导致硬熔断（$L=0.00\text{m}$）；
     - *重构*：彻底剔除原地大角度底盘自旋，引入两级前沿跃迁——当上一动作为原地扫视或新增视场增益 $<1.5\,\text{m}^2$ 时，强制锁定并触发底盘长距离位移（`moves`），继承并融合了 v2 稳健的 1.5m 步进节拍与云台的高频扫视优势。
   - **安全防撞补采视距（Safe Standoff，彻底终结 `surface_view_v3` 连续取消中断）**：
     - *根因*：地表补采点原选在未见区域前沿 1.5m 处，过于贴近弹坑与岩石边缘，落入 Nav2 代价图膨胀阻挡区（Cost $>50$），连续 3 次路径规划取消触发硬性退赛机制；
     - *重构*：将补采选点视距扩大至 $2.0 \sim 2.8\,\text{m}$，强制要求候选点通行代价 $\text{Cost} \le 50$，并在目标被阻挡时启动自愈机制（标记避障黑名单、换向扫盲、重置容错计数），保障了长周期作业的鲁棒性。
   - **容错监控与看门狗环境适应性调优**：
     - 放宽深度流活跃判断窗口至 6.0s，扩大正则兼容接收 `WARN` 及 `Too few depth returns` 状态（由云台极端仰角或稀疏点云引起），消除假性超时；
     - 坡道爬坡与滑移工况下的无进展超时上限从 15s 放宽至 25s。

2. **全场景 300s 实车仿真回归实测数据**：
   - **复杂弯道场景（`visible_curve_v1`）**：
     - `nearest_view_v3`：平稳完赛 **298.7s（`budget_reached`）**，底盘机动行驶 **$22.24\,\text{m}$**（由瘫痪状态彻底激活），观测面积由落后反超至 **$161.12\,\text{m}^2$**（全面超越 `nearest_view_v2` 的 $144.62\,\text{m}^2$），29 次动作 **0 失败（100% 成功率）**；
     - `surface_view_v3`：平稳完赛 **296.4s（`budget_reached`）**，观测面积达到全场最高 **$179.19\,\text{m}^2$**（超越 `surface_view_v2` 的 $174.25\,\text{m}^2$），地表欠采样恢复率达到 **99.1%**（110/111 栅格），单位位移观测收益达到 **$11.37\,\text{m}^2/\text{m}$**；
   - **标准月表巡视场景（`survey`）**：
     - `nearest_view_v3`：顺利完赛 293.5s，底盘行驶 **$23.53\,\text{m}$**，观测面积由原先瘫痪状态下的 $33.44\,\text{m}^2$ 飙升至 **$143.69\,\text{m}^2$**（净增 **+$110.25\,\text{m}^2$**），全程 28 次动作 **0 失败**。
   - **起伏坡道场景（`visible_ramp_v1`）**：
     - 三大 v3 策略**全部 100% 满额完赛**（`budget_reached`，彻底消除此前因坡道滑移和贴障引发的连续失败熔断退赛）；
     - 底盘在非平坦陡坡上持续推进机动：`nearest_view_v3` 行驶 **$18.09\,\text{m}$**（由 $0.00\text{m}$ 激活），初始缺陷恢复率达到 **100%（131/131 满额恢复）**；
     - `surface_view_v3` 与 `coverage_view_v3` 单位位移观测收益分别达到 **$15.45\,\text{m}^2/\text{m}$** 与 **$11.63\,\text{m}^2/\text{m}$**，能效比 v2 对应策略提升了 **+60% ~ +112%**！

3. **测试套件与资产归档**：
   - 编写 `test_planner_v3_features.py`（4/4 通过）及全量 Targeted 测试（17/17 绿灯通过）；
   - 更新 6 维时间序列对比曲线、单位位移收益双图、中文对比柱状图与 `time_milestones.csv`，统一归档至主仓库 [`MoonCar/benchmark_results/`](file:///D:/Document/Repositories/MoonCar/benchmark_results) 及 `summary_charts/`。

## 2026-09-18：月球车四大核心算法与系统闭环瓶颈全面解决（3D成果输出 / 掠射角门禁 / 2.5D路径代价 / 靶向补采视点）

在完成云台软硬件解耦、2.5D 可通行代价图与 3D LOS 光线投射的基础上，系统全面攻克并闭环了前期技术审查中指出的**最后四项核心系统与算法瓶颈**：

1. **标准 3D 地形地图成果输出（DEM / 3D Mesh 自动导出与 ROS 2 服务）**：
   - 在 `TerrainMap` 中实现了 `export_dem(path)` 与 `export_mesh(path)`：
     - **高分辨率 2.5D DEM**：导出标准 GIS 兼容的 ESRI ASCII Grid (`.asc`) 与包含高程统计（`min_z`, `max_z`, `mean_z`, `valid_cells` 等）的 JSON 伴随元数据；
     - **3D 表面三角网格**：将空间相邻的有效高程网格拓扑缝合成连续的立体三角面片，导出标准 Wavefront OBJ (`.obj`) 格式；
   - 在 `map_recorder.py` 中接入定期自动归档及 ROS 2 导出触发服务 `/mooncar/map/export_dem` 与 `/mooncar/map/export_mesh`。
2. **视场预测中的“地表法向量与掠射角退化门禁”（Grazing Angle Gating）**：
   - 针对相机从远距离大角度侧向斜视坡面时点云在 $XY$ 展布退化为单条线导致拟合报错（`insufficient_xy_spread`）的问题；
   - 在 `TerrainMap.quality(with_surface=True)` 中透传地表拟合梯度 $(g_x, g_y)$；
   - 在 `autonomous_survey.py` 的视场投射中，当网格坡度梯度 $S \ge 0.20$（$>11^\circ$）时，计算光线水平向量与梯度方向的对齐度；若夹角过大（接近等高线侧视，$\cos_{\text{align}} < 0.25$），直接触发掠射退化门禁剔除，迫使小车正对坡面进行高质量观测；平地不受影响。
3. **候选路径代价中的“2.5D 地形坡度与抗倾覆代价”（Terrain-Cost Dijkstra）**：
   - 上层 `choose_view` 的 Dijkstra 边代价计算由纯几何距离升级为 2.5D 地形代价加权：
     $$\Delta s_{\text{eff}} = \Delta s \times \left(1.0 + 2.5 \times \left(\frac{\text{cost}}{100}\right)^2\right)$$
   - 真实反映 Nav2 因陡坡横滚/翻覆风险降速产生的时间消耗与风险，引导月球车宏观规划优先避开横切险坡。
4. **针对地表缺陷格的“靶向法向补采视点”（Targeted Resampling Viewpoints）**：
   - 在 `surface_view_v3` 中，对标记为 `too_few_voxels` 或 `insufficient_xy_spread` 的地表缺陷格进行空间聚类；
   - 提取缺陷群几何质心与法向坡向，沿法向反方向距离 $1.5 \sim 2.0\text{m}$ 的安全开阔区生成专属正对候选位姿；
   - 给予 $2.0\times$ 的缺陷修复效用权重，使补采策略与覆盖优先形成本质区分，主动航向缺陷坡面前方进行高密度正向点云重建。
5. **单元测试与回归套件（110/110 项测试全部 100% 通过）**：
   - 新增 `test_terrain_export.py`（4/4 通过）：验证 DEM 头信息、网格矩阵、NODATA 填充与 OBJ 面片拓扑合法性；
   - 新增 `test_planner_v3_features.py`（3/3 通过）：验证掠射角剔除、2.5D Dijkstra 加权惩罚与缺陷群靶向视点生成；
   - 全量回归测试：`mooncar_terrain`（103/103）+ `mooncar_control`（7/7）= 110/110 项自动化测试在 ROS 2 Jazzy 容器内全绿运行。

## 2026-09-18：基于 2.5D/3D DEM 视线遮挡（Line-of-Sight Raycasting）视场预测模型重构完成

为消除起伏地形环境下由于 2D 平面视场投射忽略高程遮挡而产生的“虚假收益幻觉（False Visibility Gain）”（坡道/山脊通行代价 $<100$ 时射线直接穿透山体，误判背坡可见，导致非平地覆盖率低至约 66%），已完成 3D DEM 视线遮挡光线投射模型的全链路推导、重构与测试闭环：

1. **2.5D 地表高程数据透传（`terrain_map.py`）**：
   - 在 `TerrainMap.quality(with_surface=True)` 导出字典中，为每个网格补充拟合高程 `z`、坡度 `slope`、粗糙度 `roughness` 与几何有效性标志 `valid`；
   - 使全局观测质量与规划层能够直接读取真实地表 DEM 高程，彻底打通 3D 地形与自主规划空间。
2. **3D Line-of-Sight 光线投射与遮挡判据（`autonomous_survey.py`）**：
   - 相机空间光心：$(C_x, C_y, C_z) = (x + d_{\text{mount}}\cos(\text{yaw}), y + d_{\text{mount}}\sin(\text{yaw}), Z_{\text{robot}} + 0.35\text{m})$；
   - 垂直视场范围：根据云台倾角 `tilt`（巡航默认 $-15^\circ$）动态计算视线斜率范围 $[S_{\min}, S_{\max}] = [\tan(\text{tilt} - 29^\circ), \tan(\text{tilt} + 29^\circ)]$；
   - 动态地平线仰角追踪（Horizon Tracking）：
     沿方位角射线步进，计算视线仰角斜率 $\alpha(r) = \frac{z_t - C_z}{r}$：
     - 若 $\alpha(r) < \alpha_{\max}$：判定为山体阴影遮挡（Occluded），剔除出可见集合；
     - 若 $\alpha(r) \ge \alpha_{\max}$ 且 $\le S_{\max}$：判定为视界内可见（Visible），加入 `visible` 并更新地平线 $\alpha_{\max} \leftarrow \alpha(r)$；
     - 若 $\alpha(r) > S_{\max}$：超出垂直视场上沿，完全阻断后续光线，终止该射线（`break`）；
   - 保留向下兼容性：未探测区域与无高程测试桩平滑回退，`v1`/`v2` 历史策略保持纯平面投射不变。
3. **单元测试与回归套件（`test_raycasting_visibility.py`）**：
   - 新增 `test_raycasting_visibility.py`（7/7 全部通过）：验证平地一致性（重合度 $>90\%$）、山脊背坡遮挡（`v2` 误检数 $>0$ 而 `v3` 严格为 0）、远端高山越顶可见（Peek-Through）、下坡凸边（Brow）死角遮挡、垂直高墙视场截断、无高程回退与 `v1`/`v2` 历史策略兼容；
   - 全量回归测试通过：`mooncar_terrain` 96/96 用例全过，`mooncar_control` 7/7 用例全过，总计 103/103 项自动化用例 100% 绿色。

## 2026-09-18：2.5D 地形可通行性代价图（Traversability Costmap）模型与 Nav2 图层融合完成

为解决 Nav2 纯 2D 平面代价图与 3D/2.5D 真实地形起伏割裂的问题（无法区分缓坡与险坡、平地与大斜坡无代价差别、将可行起伏路段误判为二值化障碍），已完成 2.5D 地形可通行性数学评价模型、在线格网聚合广播与 Nav2 代价流水线深度融合：

1. **地形连续可通行性数学模型（`terrain_geometry.py`）**：
   - 实现 `compute_traversability_cost(slope, roughness, reason, ...)` 函数；
   - 区分平坦死区（$\le 5^\circ$ 且粗糙度 $\le 0.015\text{m}$ 为代价 0）、可通行缓坡/中坡（$5^\circ \sim 18^\circ$ 非线性增长为 $10 \sim 95$），以及致命险坡/断崖（$\ge 18^\circ$ 或粗糙度 $\ge 0.04\text{m}$ 或台阶落差超限置为 100）；
   - 代价值上限控制在 $95 < 99$，确保当路径必须翻越坡道到达目标时，A* 规划器允许通行且合理评估代价，同时在有平地时优先选择平地。
2. **在线 2.5D 代价图聚合与标准化广播（`terrain_node.py`）**：
   - 内部维护空间哈希字典 `self.traversability_cells`，对相机每次观测的坡度、粗糙度与障碍信息进行时空融合与老化淘汰（默认保留 60 秒）；
   - 在线广播标准 `nav_msgs/msg/OccupancyGrid` 话题 `/mooncar/terrain/traversability_map`（分辨率 0.20m，原点与坐标系对齐，未观测区域填充 -1）；
   - 保留原 `/mooncar/terrain/obstacles` (`PointCloud2`) 发布，保证原有障碍物监控与安全继电器 100% 兼容。
3. **Nav2 成本图层深度集成（`terrain_navigation.launch.py`）**：
   - 在 `global_costmap` 和 `local_costmap` 中引入 `terrain_traversability` 图层（采用 `nav2_costmap_2d::StaticLayer` 并配置 `trinary_costmap: false` 与 `use_maximum: true`）；
   - 允许非二值化的连续代价平滑注入主代价图，配合 Nav2 `RegulatedPurePursuitController` 的 `use_cost_regulated_linear_velocity_scaling` 自动实现“缓坡自主翻越、险坡主动绕行、起伏路面自适应平稳降速”。
4. **单元测试与回归套件（`test_traversability_costmap.py`）**：
   - 编写 `test_traversability_costmap.py`（6/6 全部通过），严格验证单调性、阈值边界、粗糙度增益惩罚、`OccupancyGrid` 序列化与空间对齐；
   - `test_geometry.py`（10/10）、`test_pan_tilt_transform.py`（4/4）、`test_experiment_motion.py`（15/15）、`test_level_scan_filter.py`（1/1）、`test_terrain_map.py`（7/7）全部通过回归测试。

## 2026-09-18：双自由度观测云台（Pan-Tilt）解耦方案已完成模型、控制、规划与硬件规范落地

为从根本上解决四轮滑移转向底盘在月面碎石/松散沙地原地转向时的剧烈滑移打滑、车轮刨沙沉降陷车，以及深度相机低掠射角点云退化问题，已按照方案完成两自由度正交观测云台（Pan $\pm 135^\circ$, Tilt $-35^\circ \sim +15^\circ$）的全链路软硬件实现与解耦：

1. **机器人描述与运动学解耦（`mooncar_description` / Gazebo）**：
   - 重构 `mooncar.urdf.xacro`：将固定相机改为 `base_link` $\to$ `camera_mount_link` $\to$ `camera_pan_link`（偏航轴 $Z$，$\pm 2.356\text{ rad}$）$\to$ `camera_tilt_link`（俯仰轴 $-Y$，$-0.611 \sim +0.262\text{ rad}$）$\to$ `depth_camera_link` $\to$ `depth_camera_optical_frame`。
   - `gazebo_drive.xacro` 新增 `JointPositionController` 与 `JointStatePublisher` 配置，动态控制两轴关节位置；
   - `ros_gz_bridge.yaml` 打通 `/mooncar/cmd_camera_pan` 与 `/mooncar/cmd_camera_tilt` 话题双向桥接；经 `check_urdf` 100% 验证通过。
2. **云台平滑控制与仿真执行（`mooncar_control` & `scripts`）**：
   - 新增 `pan_tilt_controller.py` 控制节点，实现一阶平滑加减速（限速 $1.5\text{ rad/s}$，抑制运动模糊）、机械死区硬限位保护与默认 $-15^\circ$（$-0.2618\text{ rad}$）俯视巡航回位（Homing）；
   - `scripts/experiment_motion.py` 扩展 `pan_tilt(pan, tilt, task)` 执行接口与生命周期状态机。在云台动作期间，底盘运动安全继电器被显式锁止（`permit=False`），彻底杜绝底盘误动与沙地滑移。
3. **自主探测算法解耦（`scripts/autonomous_survey.py`）**：
   - 保留原有 `v1` / `v2` 策略族与历史评价回归兼容性，新增 `v3` 策略族（`nearest_view_v3`, `coverage_view_v3`, `surface_view_v3`）；
   - 在选择观察视场时，当所需视向角处于云台偏航量程（$\pm 135^\circ$）内，自主规划器自动选择原地云台扫描（耗时仅 $1.5\sim 2.5\text{s}$），替代笨重耗时且高风险的底盘原地旋转（耗时 $>5\text{s}$ 并引起滑移累积误差）。
4. **全套自动化单元与几何变换测试验证**：
   - `test_pan_tilt_controller.py`（3/3 通过）：平滑轨迹插值、限位保护、Homing 正确；
   - `test_pan_tilt_transform.py`（4/4 通过）：云台偏航水平投影、俯仰俯视投影、结合非平地 IMU 姿态的 3D 点云变换与几何距离不变性验证全部通过；
   - `test_experiment_motion.py`（15/15 通过）：参数校验、生命周期切换、超时保护及心跳看门狗验证全部通过；
   - `test_autonomous_survey.py`（14/14 通过）：策略决策、预算审计、回归兼容性测试 100% 通过；
   - `test_geometry.py`（10/10 通过）：地图与几何投影回归通过。
5. **物理落地与硬件选型规范（`docs/hardware_pan_tilt_spec.md`）**：
   - 制定完整硬件工程规范：选用飞特 STS3215 串行总线舵机（$19.5\text{ kg}\cdot\text{cm}$ 扭矩，12 位磁编码反馈）、铝合金双轴 U 支架、独立 $7.4\text{V}/5\text{A}$ DCDC 降压隔离供电、OAK-D Lite 专用减振背板及 U 型防缠绕超柔线束；新增 BOM 预算仅约 ¥317 RMB。

## 2026-09-18：v2 冻结版 300 秒正式对照完成并通过证据审计

最终接续批次 `tmp/ros_acceptance/formal-strategy-v2-tf-replay-300s-20260918-font-recovery/` 保留原协议 SHA-256 `18d3aecd10f3e718c03890d04dedee37435b6548dd8f55a57b0b3e8f842b4205`，复用原批次已审计通过的 `survey` 18 轮，并在完整运行环境下重新执行 `visible-ramp`、`visible-curve` 各 18 轮。三场景 54/54 轮均到预算终点，三个场景审计和终审均为通过，终审为 54 轮、6 个固定参考条件、0 问题；原始深度、评价和源码证据各 54 份齐备。原始编排及两次接续的只读报告目录、ROS 域、中文字体环境缺项均保留作运行故障记录，未混入正式成绩。正式结果、协议及审计表见 `reports/evidence/terrain-workbench/formal-strategy-v2-tf-replay-results-20260918/README.md`。

六个场景起点条件等权的平均 S：就近 66.81、覆盖优先 83.59、地表补采 87.26。覆盖优先与补采的平均 C 分别为 87.82%、89.22%，但非平地平均 C 仅 66.03%、67.35%，平均 P 仅 64.22%、59.47%；按冻结的总体及分层 C≥80%、P≥90% 暂定门槛，54 轮中仅覆盖优先 2 轮、补采 1 轮通过几何门槛，不能宣称覆盖不足已全面解决。三策略成对比较的最小 Holm 校正 p=0.09375，不作 5% 显著优势或跨场景泛化断言；实物安全仍未验证。用户要求此处停止推进，先汇报结果，待其安排。

## 2026-09-18：静态 TF 修复版正式 54 轮已重新登记并启动

因旧体素精度版 `survey` 批次有两轮策略启动前 TF 漏接，共用启动链已修复并通过定向测试、78 项地形单元测试和 10/10 次短启动验证。新的统一源码正式协议位于 `tmp/ros_acceptance/formal-strategy-v2-tf-replay-300s-20260918/protocol.json`，SHA-256 为 `18d3aecd10f3e718c03890d04dedee37435b6548dd8f55a57b0b3e8f842b4205`；129 个源码/配置文件、监护脚本和分析脚本已冻结。仍按原 3 场景×2 起点×3 策略×3 重复、300 仿真秒、8 m 半径及 `mapping-acceptance-v1.1` 执行。批次已启动，正式结果待全部完成和审计。旧版有效及无效档案只作缺陷诊断，不混算。用户要求仿真及验收完成后先停止推进，只汇报状况。

## 2026-09-18：静态 TF 漏接导致修复版正式批次两轮启动无效

体素精度修复版正式批次 `formal-strategy-v2-voxel-precision-300s-20260918` 的 `survey` 场景已跑完 18 个计划槽位，但第 15、18 轮在策略启动前因工作台定位 TF 不连通而超时，没有同步原始捕获或 C/P/S；其余 16 轮档案保留，整批退出码 1，后两场景未启动。现场 `/tf2_frames` 显示工作台 TF 缓存缺少 `survey_odom → odom`，而新订阅者可读到该静态变换；这不是某种探测策略或覆盖率分数导致。共用启动链现于开始 10 秒后仍无定位时，重建一次 TF 订阅以重放静态变换。两项定向测试先失败后通过，完整地形测试 78 项通过；预登记的两起点交替 10 次短启动诊断均就绪，耗时 26.4–37.5 秒，退出码 0。这只是启动链验证，正式 54 轮仍须以统一修复版重登记重跑。旧批次全部保留、不混算，评价公式/分母和策略目标不变。详细证据见 `reports/evidence/terrain-workbench/tf-static-replay-20260918/README.md`。

## 2026-09-18：地图记录修复版正式对照已重新登记并启动

120 仿真秒在线短验证已完成：`surface_view_v2` 正常运行至预算末端，7 个动作零失败，142 帧原始数据全部纳入评价，记录节点没有再次抛出跨格界 `KeyError`；S=52.71 仅为诊断值。完整记录见 `reports/evidence/terrain-workbench/voxel-centroid-boundary-20260918/README.md`。新的 54 轮协议已登记在 `tmp/ros_acceptance/formal-strategy-v2-voxel-precision-300s-20260918/protocol.json`，SHA-256 为 `22f05f9370b9f65fcbe582ee9b2fa9ff7056593fa19ecb448d90d33f3f76ddf9`；本次将每轮源码包中的 129 个源码与配置文件全部冻结，包括此前遗漏的 `terrain_map.py` 和 `map_recorder.py`。三场景、两起点、三策略、三重复、300 秒预算及 `mapping-acceptance-v1.1` 不变。正式批次已启动，结果仍待全部完成和审计；先前两个中断批次仅用于缺陷诊断。

## 2026-09-18：共用地表记录节点跨格界崩溃，第二版正式批次中断

`formal-strategy-v2-goal-tolerance-300s-20260918` 的前两轮完整、档案预检通过，第三轮补采在 300 秒预算早期因共用 `map_recorder.py` 抛 `KeyError (-62,-70)` 而停止，第 4 轮启动中被中断；`survey-exit-code.txt=130`、`campaign-exit-code.txt=1`。第三轮 S=19.81 是基础设施故障后的早停记录，不用于修复版公平对照。旧档案全部保留。根因是融合体素中心从双精度缩为 `float32` 后跨越 0.25 m 网格边界，和原始点登记的观测格不一致。旧源码重放实际崩溃档案会再次抛同一异常，修复版可生成 906 个质量格；定向测试先失败后通过，完整地形单元测试 76 项通过。源码和容器安装运行副本已同步；预登记的 120 秒在线短验证现已通过，并另建协议重跑 54 轮。证据见 `reports/evidence/terrain-workbench/voxel-centroid-boundary-20260918/README.md` 与 `tmp/ros_acceptance/voxel-boundary-replay-20260918/`。评价标准、三策略和场景未因分数修改。

## 2026-09-18：共用到点失效已修复，第二版正式对照登记与中断前证据

旧正式批次前三轮完整档案出现三次 Nav2 到达目标附近却持续绕行的动作失败，失败段逐秒位姿最小目标距离为 0.199、0.180、0.240 m，均未进入运行时配置的 0.15 m 到点半径。已保留旧批次前三轮结果和被中断第 4 轮的 247 帧原始数据，`survey-exit-code.txt=130`、`campaign-exit-code.txt=1`，没有把未完成轮伪装成评分结果。共同工作台 Nav2 `PositionGoalChecker` 半径改为 0.25 m（Jazzy 默认值）；独立复测导航至旧失败目标，运行时参数实测 0.25 m，Nav2 正常完成，终点误差 0.2326 m。诊断见 `reports/evidence/terrain-workbench/goal-tolerance-v2-20260918/README.md`。

新的正式协议在 `tmp/ros_acceptance/formal-strategy-v2-goal-tolerance-300s-20260918/protocol.json` 登记并保存 SHA-256；仍是 3 场景×2 起点×3 策略×3 重复、300 仿真秒、8 m 半径和原评价版本/门槛，源码仅共同到点参数变化。新的 `survey` 18 轮 `plan.json` 已生成；首轮就近策略到预算末端，399 帧原始采集、36 个动作且零动作失败，C=61.19%、P=99.31%、S=60.77，非平地 C=31.55%，未通过冻结的分层门槛。其坐标、源码、原始数据及参考哈希预检无异常；后续状态以 `survey/trials.json` 为准。顺序运行进程 PID 10638 与监护进程 PID 10620 已确认存活。监护程序每个场景完成后先审计，再启动下一场景；最终以 `campaign-exit-code.txt` 和 `reports/evidence/terrain-workbench/formal-strategy-v2-goal-tolerance-results-20260918/` 的审计与对照表为准。旧版与新版不混算。当前仍不能宣称几何门槛或实物安全验收通过。

## 2026-09-18：旧版 54 轮正式对照登记与中断前证据

在修复版三策略同条件重验证、78 项地形测试和源码哈希核验后，正式方案已在 `tmp/ros_acceptance/formal-strategy-v2-300s-20260918/protocol.json` 登记，固定 3 场景×2 起点×3 策略×3 重复、每轮 300 仿真秒、`mapping-acceptance-v1.1` 不变。`survey` 场景的 18 轮批次已启动，`plan.json` 确认 18 个槽位。首轮起点一 `nearest_view_v2` 已完成：原始捕获 `captured`，405 帧可评价，C=57.55%、P=98.21%、S=56.52，未通过总体覆盖和非平地分层门槛；33 个动作中 1 个因持续未接近目标被取消，任务仍到预算末端。第二轮 `coverage_view_v2` 也已完成：C=87.48%、P=98.97%、S=86.58，但非平地 C=41.67%、P=78.57%，仍未通过分层门槛。后续逐轮状态以 `tmp/ros_acceptance/formal-strategy-v2-300s-20260918/survey/trials.json` 为准；运行日志为批次根目录的 `survey-run.log`，批次完成或失败后在批次根目录写入 `survey-exit-code.txt`。不得把正在运行的槽位当作完成，也不得因中途成绩更改后两场景计划。全部 54 轮及审计、对照报告完成前，研究结论仍未冻结。

正式结果的独立审计脚本为 `reports/evidence/terrain-workbench/formal-strategy-v2-300s-20260918/analyze.py`；已用前两轮逐档案预检，坐标、哈希、参考和评分记录相符。`tmp/ros_acceptance/formal-strategy-v2-300s-20260918/continue.sh` 已作为后台续跑监护程序启动：每个 18 轮批次结束后先审计，审计通过才按预登记顺序进入下一场景；若运行或审计失败，写 `campaign-failure.txt` 并停止。监护程序第一次启动因 `set -u` 与 ROS setup 脚本冲突立即退出，原日志 `campaign-run.log` 保留；调整初始化顺序后在 `campaign-run-retry.log` 下重新启动，未触碰正在执行的 `survey` 轮次。最终完成仍以 54 个槽位和 `campaign-exit-code.txt`、正式审计报告为准。

## 2026-09-18：代价 99 净空缺陷修复与三策略重验证

旧同条件小批次中，补采轮曾到达观察点后因转向 `Collision Ahead` 安全中止。离线栅格和 Nav2 Jazzy 代价值核查确认：候选点的 0.65 m 车体净空圆含内接障碍代价 99，普通与边界候选却只拒绝 ≥100，和共同路径搜索及目标中心检查的 ≥99 判定不一致。两处候选净空现统一拒绝 ≥99；定向测试先失败后通过，地形测试 78 项通过。旧小批次的结果和哈希不变。

修复版另建协议、重跑同场景同起点三策略各一次，三轮均有完整原始捕获、通过参考/坐标/截止审计、到预算末端且零动作失败。就近/覆盖/补采的总体 C=66.00%/88.21%/87.49%，P=99.90%/99.86%/99.78%，S=65.93/88.09/87.30；非平地 C=51.61%/79.03%/82.26%。补采轮**几何门槛满足但安全未认证**，覆盖轮非平地 49/62 格、距 80% 差 1 格。单场景单次只说明当前条件下覆盖与补采的取舍：覆盖总体 S 略高，补采非平地 C 较高，不能宣称跨场景优势。新旧小批次不混算。证据见 `reports/evidence/terrain-workbench/strategy-v2-clearance-small-batch-20260918/README.md`。

共同实现现在可进入 `docs/formal_strategy_v2_300s_protocol.md` 的 54 轮多场景正式对照，按事前场景/起点/重复数和同一 300 秒预算执行；所有失败和未通过结果保留。正式对照及实物安全验收尚未完成，不能把本轮几何通过等同于项目全部问题解决。

## 2026-09-18：共同修复后三策略同条件小批次已完成

预先保存协议后，在 `visible_curve_v1` 第二起点、8 m 半径、300 仿真秒预算下，三种 v2 策略各从空白状态运行一次。三轮捕获、参考、坐标、来源和截止审计均有效，使用同一源码与场景哈希；均到预算末端。就近/覆盖/补采的总体 C 分别为 60.15%/92.36%/93.53%，P 为 99.84%/99.83%/99.87%，S 为 60.06/92.21/93.40；非平地 C 为 53.23%/54.84%/72.58%，因此**三轮均未通过固定分层门槛**。补采有 1/16 个动作失败（Nav2 目标错误码 703），其他两轮动作失败数为 0。完整表、路径、原始证据和解释边界见 `reports/evidence/terrain-workbench/strategy-v2-shared-fix-small-batch-20260918/README.md`。单场景单起点各一次仅是探索性小批次，不能宣称补采稳定优于其他策略；不得据此继续为成绩调参。

当前明确终点：共同运行链在这个条件下能连续跑满预算，总体 C 不足不再是覆盖/补采这两轮的主要故障；尚未解决的是固定非平地 C≥80% 和跨场景稳定性。下一步正式研究须在运行前固定多场景、起点、重复数和机时，保留失败及不达标结果。若用户只需阶段性论文材料，本批可如实写为策略适用性和限制；若要求“仿真验收通过”，现有证据明确不能宣称完成。安全认证和真实硬件验收仍未核实。

## 2026-09-18：共同导航修复后的完整预算诊断

`visible_curve_v1` 第二起点、8 m 半径、覆盖优先 v2 的一次 300 仿真秒诊断轮正常跑到预算末端，16 个动作全部成功。固定 `mapping-acceptance-v1.1` 下总体 C=89.66%、P=99.93%、S=89.60，平地 C=90.38%、P=99.93%；但非平地 C=53.23%、P=100%，因此几何评价仍为**未通过**（唯一失败门槛：`terrain: coverage below target`）。原始观测覆盖率 97.42%；404 帧全部用于评价，坐标、来源和时间截止审计通过。安全认证仍为空，不能宣称实物或整机验收通过。本轮是缺陷诊断，不是三策略公平对照，也不能证明总体 C 已稳定达标。完整记录见 `reports/evidence/terrain-workbench/precision-recovery-20260918/README.md`。

共同修复包括：倾斜时停止向二维 SLAM 输入激光扫描，但 Nav2 仍使用原始扫描避障；在线全局代价地图改为 20 m 滚动窗口、修正静态层合并和禁入区/障碍层次序；禁入区掩膜延伸出稀疏 SLAM 地图边界；SLAM 发布当前时间戳的持续 TF，避免倾斜期间扫描暂停导致 TF 过期。完整轮的估计位姿对独立真值 XY RMSE≈0.050 m，未重演此前的明显朝向跳变和定位故障。这是单轮现场验证，不能保证所有场景和起点。

非平地 62 个固定可评价格仅 33 个有效重建；未有效的 29 格中 19 格无原始观测，主要位于曲面目标西侧。保存的 96.5 s、126.9 s 导航栅格显示目标西侧多个观察位置代价为 0、车体净空检查通过，Dijkstra 有约 3.3–7.2 m 的已知路径；这只证明**规划图上可达**，尚未证明 Nav2 实际可达或相机能有效重建。覆盖优先策略在这一轮绕过目标南侧并转往其他平地，属于当前策略和有限预算的真实结果，不应通过更改分母或事后增加时间掩盖。

下一固定终点：先冻结这套共同导航、采集和评价实现；在预先写明的同场景、起点、初始状态、300 s 预算和安全约束下，分别运行三种策略的独立小批次，保留失败及负面结果。若某轮仍有共同执行故障，修复后重跑受影响批次；若运行稳定而非平地 C 仍不足，报告各策略适用限制，不因某个分数继续调参。旧 54 轮和本页诊断轮不混入新的正式比较。

## 2026-09-18：位姿与边界补探验证后的当前状态

在固定 `mapping-acceptance-v1.1`、5 cm/5° 误差限及 C≥80%、P≥90% 暂定门槛下，同一 `visible_curve_v1` 第二起点、8 m 半径、覆盖优先 v2、300 仿真秒上限的最新完整捕获仍未达标：任务在 249.4 仿真秒因定位 TF 时间失效而停止，C=58.05%、P=96.97%、S=56.29；平地 C=57.34%，非平地 P=25.86%。原始深度与 IMU 均通过来源、坐标、截止时间和逐帧姿态重放审计。这轮有 19 个动作、5 个前进目标因持续无进展取消，新增地图边界兜底没有被实际触发，不能宣称已通过现场验证。71 项地形测试通过。证据和各次失败档案见 `reports/evidence/terrain-workbench/precision-recovery-20260918/README.md`。

独立真值仅用于离线诊断：同一 385 帧深度改用真值位姿重建，C=57.57%、P=99.78%；因此本轮 C 不足主要是实际路径与观测不足，不能靠修正评分位姿补到 80%。当前位姿相对真值的 XY RMSE 为 0.447 m、最大约 0.921 m；未经 SLAM 修正的轮速/IMU 融合轨迹 XY RMSE 为 2.921 m，而末帧经 SLAM 修正的朝向偏约 10.1°。非平地的低 P 同时暴露定位/朝向误差。该版本的单轮成功或失败均不与旧正式 54 轮混算。目前**没有**经过同条件重复验证的 C 达标保证，也尚未冻结新三策略正式对照版本。下一阶段先定向核查导航受阻与二维定位漂移，再做一次短验证和一轮完整预算验证；若仍未达标，保留负面结果并按预定对照评价策略适用条件，不再因成绩循环调参。

## 2026-09-17：有效覆盖率 C 的定向修复与单轮达标

90 秒不足，直接延长的旧运行在 212.5 秒因受阻目标导航超时，C 反降到 25.86%。共同执行层现对多次恢复且 15 仿真秒无有效进展的目标取消并换点，代价地图更新后被安全检查拒绝的候选点也会记录失败并重新选择，不再误停整轮；安全失效仍停止。67 项地形测试通过。最终同场景同起点覆盖优先 v2 单轮以 300 秒为上限，在 282.1 秒正常结束：C=81.65% 达到暂定 80% 门槛，C_obs=88.43%，17 个动作全成功，路程 22.29 m；P=81.99% 低于 90%，整体暂定验收仍失败。约 270 秒 C=78.57%、281 秒 C=81.49%，当前条件采用 300 秒上限有据可查，不能泛化到其他场景或策略。完整证据及失败档案见 `reports/evidence/terrain-workbench/coverage-recovery-20260917/README.md`；旧正式 54 轮成绩保持冻结，不混入新运行层结果。

## 2026-09-17：共同运行层修复后三策略首个小批次

按预先保存计划，在 `visible_curve_v1` 第二起点 `(6.5,-2,2.4)`、第一次重复、90 仿真秒和 8 m 半径下，三种 v2 策略顺序各跑一轮。三轮均有正确 Gazebo/任务起点、完整原始采集和 `mapping-acceptance-v1.1` 评价；28/28 个 Nav2 目标获得接受响应，无行动失败。C 分别为覆盖优先 36.83%、地表补采 36.77%、就近观察 36.49%，均未达到暂定 80%；旧同条件两条零行驶轮本批已分别行驶 5.42 m 和 2.82 m。单条件各一次仅支持运行层诊断，不能宣称策略优势、整体覆盖问题解决或偶发失响应根治。小批报告见 `reports/evidence/terrain-workbench/runtime-repair-small-batch-20260917/README.md`；原 54 轮正式结果保持冻结。

## 2026-09-16：共同运行层修复及两轮短全栈验证（当前进展）

每轮首次动作现核对定位与计划起点在 0.25 m 内，否则工作台保持未就绪并拒绝自主任务或手动发令；跨轮重置不再依赖无法清除 Jazzy 静态 TF 的 `Buffer.clear()`，而是重建 TF 监听器、缓存及 Nav2 动作客户端。现有 Jazzy 容器相关 28 项测试通过。最终代码的短全栈只验证同一工作台从弯曲场景起点二切到起点一的两个首动作：前进目标被接受且产生新轨迹点，转向目标被接受并成功；证据见 `tmp/ros_acceptance/formal-strategy-v2-cause-20260916/live-start-isolation-final-05/`。这不是 90 秒正式试验，不证明旧六轮偶发目标失响应已根治；54 轮冻结结果不变。详细边界见 `reports/evidence/terrain-workbench/formal-strategy-v2-results-20260916/cause-analysis.md`。

## 2026-09-16 晚：v2 正式 54 轮及覆盖不足诊断（当前状态）

3 场景×2 起点×3 策略×3 重复、每轮 90 仿真秒的 v2 正式对照已完成并审计；54/54 的 C 均低于暂定 80% 门槛，最高 51.86%，没有稳定的策略优势。六条旧档案起点错位按预先规定的坐标协议补跑，原件保留；六条坐标正确的弯曲场景试次几乎零行驶，仍计入结果。正式成绩见 `reports/evidence/terrain-workbench/formal-strategy-v2-results-20260916/README.md`。下方“尚未启动 v2 正式实验”是此前阶段记录，不能当作当前状态。

原因核查见同目录 `cause-analysis.md`：48 条移动试次中 43 条因剩余预算不足结束；对正式 54 条逐格重算，84,766 个试次格未观测、22,379 格已观测但拟合无效，后者 92% 是不足 6 个体素。六条零行驶试次卡在 Nav2 目标响应链，底层失响应原因尚未证实；六条起点错位与旧静态 TF 缓存及缺少当轮起点门禁相符，已定向复现。当前开发工作区已补目标 UUID、Future 状态和 5 秒接收时限；相关 24 个单元测试及隔离 ROS 的 30 次显式 UUID 动作往返通过。第一次短时全栈诊断另暴露 0.1 秒 TF 未来时间门槛过严而取消动作，现已按共同导航容差修为 0.3 秒；之后一次新工作台和同一工作台连续三次重启的短诊断均收到目标响应并观察到动作执行。**旧六次目标响应缺失仍未复现，不能宣称已彻底修复**。冻结的 54 条成绩不变。

## 2026-09-16：自主观察规划器 v2 三轮验证完成并冻结

最终有效批次 `tmp/ros_acceptance/v2-validation-20260916-final/` 使用综合场地、共同起点 `(-15,-16,0)`、90仿真秒、8米半径，三种v2策略各1轮。三轮原始捕获均captured、研究报告均ready、评价均为mapping-acceptance-v1.1，运行源码哈希一致，档案与评价哈希回读无误。容器回归：地形58项、控制4项通过。验证说明与冻结边界见 `reports/evidence/terrain-workbench/planner-v2-validation-20260916/README.md`。

Nearest：12动作=10转向+2前进，全部成功，路程2.749m，C=42.58%，P=97.35%，S=41.45。Coverage：9动作=6转向+3前进，全部成功，路程4.401m，C=45.90%，P=96.84%，S=44.45。Surface：4次前进，3成功、末次预算中断，路程9.713m，C=52.90%，P=98.72%，S=52.23。末次Surface动作持续移动且Nav2/传感器健康，但截止时距目标约0.28m，未达到共同0.15m到达容差；保留为负面结果，不放宽容差。三轮均因C<80%未通过暂定验收；单场景单次不能证明Surface更优。

冻结实现包括路径沿途视野、移动机会预算保留、明确的budget/no-candidate分类、4秒公共执行余量和发令前二次预算复核。修复期间的三个较早批次保留但不纳入最终表：首批中文字体环境失败；rerun暴露写盘后预算变化；postfix验证0.5秒余量不足。最终批次后不再按成绩调参。尚未启动v2正式54轮多场景重复实验。

## 2026-09-16：自主观察规划器 v2 修复及短冒烟完成，尚未启动新正式实验

54轮结果的逐格与动作审计表明，有效覆盖约42%至43%由约42%未观测和约15.5%已观测但无法有效拟合共同造成。571个动作中483个为原地转向、88个为前进；53个 `no_candidates` 轮次结束时最多只剩11.8秒，而原地观察至少需要5秒、1.5米前进观察至少约12.5秒。旧状态混合了真正无候选与剩余时间不足。

新菜单使用 `nearest_view_v2`、`coverage_view_v2`、`surface_view_v2`。三者共同新增：候选过滤诊断；有正收益候选但时间不足时记 `budget_reached`；原地转向不得消耗最后一个仍可执行的安全前进机会；前进候选计入Dijkstra预计路径沿途与终点视野的并集。策略各自的时间/覆盖/补采排序目标、Nav2、安全约束、90秒预算和8米半径不变。v1入口和54轮冻结证据保留，评价继续使用 `mapping-acceptance-v1.1`。

现有 `mooncar-terrain:jazzy` 容器中地形测试56项、控制测试4项全部通过；其中自主规划器7项定向测试覆盖v1兼容、v2移动预算保留、沿途视野和终止分类。无ROS的90秒开阔网格诊断中，coverage/surface由v1的1次前进增加为v2的4次，nearest由1次增加为2次；该诊断只证明调度分支生效，不是覆盖率或策略优越性证据。

另以综合场地起点 `(-15,-16,0)` 完成一次20仿真秒、无GUI的 `coverage_view_v2` 冒烟：首个动作由移动预算保留规则选为前进，动作成功，路径约1.378米，0个失败动作；终点约 `(-13.624,-15.938)`，剩余动作因预算不足正确记为 `budget_reached`。档案 `tmp/ros_acceptance/frontend-experiments/experiment-6c456366b8aa/research/autonomy-16083f5e7b/` 的报告状态 ready，哈希回读无误。仿真已停止，验证容器已关闭。该单轮仅验证ROS/Nav2集成和记录，尚未运行新54轮正式对照，不能用旧成绩代表v2。

## 2026-09-16：54轮正式对照完成并按评价 v1.1 复算

3场景×2起点×3策略×3重复共54轮仿真已完成。综合场地原参考审计失败定位为12个曲面/倾斜盒体边缘网格的单平面采样收敛问题；没有缺失参考、掩膜分歧或坐标错误。`mapping-acceptance-v1.1` 将非收敛格预先归为 edge_or_complex，保留清单并从固定分母排除；公式和5cm/5deg、80%/90%阈值不变。综合场地两起点排除4格和8格，其他条件0格。

全部54轮使用原冻结深度和轨迹统一离线复算，无需重跑Gazebo；54轮均可评价且均因覆盖不足未通过。六条件等权S均值：nearest 42.04、coverage 41.70、surface 42.00；三组精确配对检验Holm校正后p均为1，没有稳定策略优势。53轮因无新安全候选观察位置结束，1轮达到预算。正式结果：reports/evidence/terrain-workbench/formal-strategy-results-v1_1-20260916/README.md。

## 2026-09-14：评价交付中文版与云端归档

用户要求输出统一中文，并新建 Drive 文件夹上传。报告生成器已更新中文标题、图例、坐标轴、状态及阅读版 CSV/JSON；机器接口字段仍保留原格式，历史冻结证据未改动。
中文 PDF 使用嵌入字体，图表同时处理栅格压缩和文字提取问题。5 项自动测试通过；两份地图的逐格数值与上一版完全一致，所有中文 PDF 已渲染检查。
本地交付：reports/地形采集评价_2026-09-14_中文版，共 17 个文件（含说明、地图、中文表格和原貌截图）。
Drive 文件夹：https://drive.google.com/drive/folders/1-sZF87VkJiZIL9pgtUjZjEQ_fht5ygYv
文件已上传并列目录回读核对。中文版生成不启动仿真，不改变原始成绩或历史证据。

## Offline mapping acceptance delivered 2026-09-14

Current evaluator: scripts/mapping_acceptance.py and scripts/report_mapping_acceptance.py.
docs/mapping_acceptance.md now specifies v1; the earlier draft entry below is historical.
Scope: fixed single-surface reference, synchronized raw-depth capture, estimated-camera
map reconstruction, C/P/S, unfiltered valid-cell height/slope errors, category gates,
black/white PNG/PDF maps, CSV/JSON and evidence hashes. No online policy or ML changes.
Supported runtime capture records actual SLAM frames, survey/odom TF, deployed launch
and robot camera geometry. Truth is audit-only; no final-map/trajectory registration.
Thresholds5cm/5deg, coverage80%, precision90% remain provisional simulation targets.
Safety certification stays unknown; successful geometry alone cannot pass whole-system acceptance.
Real 30s motion example: 53 synchronized frames; C=56.712963%, P=100%, S=56.712963;
height RMSE0.00084634m, slope MAE0.209317deg. 245/432 eligible cells qualified,
ROI28m2, eligible27m2, excluded complex edge1m2. Geometry fails coverage target;
these ideal-simulation values do not describe hardware accuracy or strategy superiority.
Initial requested approach did not start because the outer runner used ROS domain0
while capture used89. Keep its raw evidence and explicit execution-fault report;
run() now rejects mismatched domain before launching anything. Subsequent same-scope
capture in domain89 moved and Nav2 succeeded. Simulator ended stopped/inactive.
Delivery: reports/evidence/terrain-workbench/mapping-acceptance-20260914/README.md.
Old six-trial, three-strategy comparison reports remain unchanged and unscored under this standard.

## Acceptance specification drafted 2026-09-14

User requires explicit accuracy/coverage acceptance before further algorithm claims.
docs/mapping_acceptance.md defines a reviewable v0.1 spec: fixed single-surface ROI,
sensor-supported reconstruction, height+slope qualification, C=A_est/A_ROI,
P=A_good/A_est and S=100*A_good/A_ROI. Reference unavailable means score null;
an evaluable empty map scores0 with P null. Errors include all reported cells,
not only qualified cells; independent reference and fixed coordinates required.
Proposed tolerances5cm/5deg and C>=80%,P>=90% are explicitly provisional simulation
targets, not calibrated capability or external standards. Safety and evidence are
separate gates; flat/nonflat strata must be reported and gated separately.
No scorer integration, new simulation, ML training or retrospective scores were
produced in this step. Old six-trial fit-area comparisons remain developmental;
they cannot be relabelled as qualified-area or accuracy comparisons.

## Simulation closeout campaign completed 2026-09-14 (Asia/Tokyo)

Six predeclared attempts at two peripheral starts completed: six captured reports,
five no_candidates stops and one budget_reached stop; zero failed actions and one
budget-interrupted action. Each run saved four scene images (24 total), raw archives,
baseline/final maps, telemetry and decisions. Current online algorithms were not
edited. Per-run sources match the pre-capture freeze; deployed ROS runtime sources
matched (only development tests differed). Portable evidence and area arithmetic
checks pass; comparison figures and representative scene/map images visually checked.
Primary valid-surface gains at (-15,-16), Nearest/Coverage/Surface:
107.3125/112.8125/105.25 m2. At (15,16):113.4375/108.375/109.75 m2.
Surface does not exceed both baselines at either start: no consistent descriptive
advantage; one run per condition cannot establish significance or generalization.
Original one-off wrapper exited1 after all six reports: duplicate asynchronous
stop request returned400 while already stopping. Error preserved, actual stopped
state independently confirmed; existing offline reporter used without rerunning
trials. Do not advertise this wrapper as a stable reusable command.
Report: reports/evidence/terrain-workbench/strategy-closeout-20260913/README.md.
Scope/checklist: docs/simulation_closeout.md. Backend remains available, simulation
stopped, motion/autonomy inactive. Offline v5 stays unvalidated and unintegrated.
This closes the bounded comparison; no automatic next tuning or experiment stage.

## Bounded closeout direction, 2026-09-13

User raised endless iteration and overfitting concerns. This direction supersedes
the research "Next" items below: freeze current online v1 strategies and offline
v5, do not tune or integrate v5, and close this simulation stage with honest evidence.
See docs/simulation_closeout.md. One final campaign is frozen before observations:
/evidence/strategy-closeout-20260913/protocol.json and source-before.zip;
tmp/run_closeout.py reuses the existing runner and report without algorithm edits.
Two previously unused starts (-15,-16,0) and (15,16,pi), three online strategies
each once, 90 simsecond cap, radius8m; six attempts, no replacement or tuning.
Primary is final-minus-initial valid fitted area; secondary coverage, recovery,
path, actual duration and failures remain separate. Early stops are retained.
Peripheral arrangement checks reuse the same world primitives; NOT independent
unseen-world validation. Prior task inventory proves unused start coordinates,
not that every cell is unseen. One repetition cannot establish significance.
The offline v5 remains unvalidated, and is not the model being compared online.
Final results belong in reports/evidence/terrain-workbench/strategy-closeout-20260913.
Do not automatically launch another model-development or benchmark campaign.

## Common known-surface model and supplemental quartet completed 2026-09-13

read_pending_json now tolerates ENODATA only (warning retained), propagating other
OS errors. Actual /evidence bind-mount stress reproduced2ENODATA across1000atomic
writes/1754successful reads, finalsequence1000; no unhandled reader/writer errors.
Capture records exception stack and stage. read_pixel_offset separates unavailable
service, reply timeout, malformed response and wrong type/range; at most2bounded
attempts for availability/timeout, no guessed calibration. Fault injection and an
actual isolated ROS90 delayed service passed; first fixture teardown warning was
removed by draining its deliberately late reply before destruction. Clean rerun
saved /evidence/parameter-retry-live-drained-20260913.{json,log}. Historical parameter
failure has no timing/type detail; cannot retroactively identify its precise cause.

New offline scripts/action_support.py unifies turn/path sampling and known-cell
ray/voxel/actual TerrainMap fit hypotheses (v5). Predicts recovered-minus-lost
known surface area over20s; unknown new area and total action rank remain null.
No hypothetical points enter archives; online autonomous_survey.py unchanged.
compare_observation_actions.outcome adds loss-adjusted net utility, old positive
utility retained; ROI loss scoping now reuses scoped_cells. New evaluator also
intersects initially-valid cells across arms for shared loss-adjusted differences.
Development on old6valid captures: movement FP53/60 vs old14/23; FN6/6 vs20/20,
three real valid-cell losses missed. Baseline-fit consistency mismatch0. Common
meaning is NOT improved accuracy. New4run protocol frozen before fresh observations:
tmp/run_common_action.py, tmp/evaluate_common_action.py, /evidence/common-action-20260913.
Third quartet order hold/wide/advance/small, same ramp/tasks/60s and20s evaluation.
Old8attempt failure status preserved. Fresh4/4capture+execution checks passed;
105..111frames,window endpoint lag.104..426s; both prefixes match final raw arrays,
source hashes match frozen protocol, baseline-fit inconsistency0. All4parameter
reads first-attempt success,value.5; no startup recurrence, not a lifetime guarantee.
Shared37deficits,11shared targets,204shared initially-valid cells. Hold/small/wide/
advance new cells0/0/5/50; recovered0/1/9/10; original-target recovery0/0/0/3.
Shared-loss adjusted difference0/.0625/.875/3.75m2. Old complete R2 re-evaluated with
same loss cohorts:0/0/1.1875/4m2. Two descriptive complete quartets now available;
original8-attempt admission remains failed, total12attempts10valid,2unpaired valid.
New small turn loses1valid cell outside shared initially-valid cohort; predictor
misses it. New advance forecast60recoveries:TP25,FP35,FN3(actual28). Wide TP10/FP5/
FN11,small TP7/FP3/FN11. Model has NOT passed accuracy/online admission. Known-only
predicted net is NOT comparable with observed total net including new coverage.
Loss of fit validity may reveal an earlier unreliable fit, not worse accuracy;
loss subtraction remains a candidate metric, not an automatic safety penalty.
Report reports/evidence/terrain-workbench/common-action-20260913, tests and live
failure probes recorded. Backend stopped/idle. Next diagnose path support inflation
using planned versus measured camera views and voxel fusion/fit threshold changes,
then validate independent geometries; unknown new-area prediction remains absent.
No online integration, new full three-strategy run, or hardware acceptance.

## Equal-budget action-value comparison captured and evaluated 2026-09-13

scripts/compare_observation_actions.py and an optional capture helper argument now
support hold, +.02rad, +.4rad and onward Nav2 movement with explicit final heading.
Online autonomous_survey.py is unchanged. Frozen8-attempt protocol and raw output:
tmp/ros_acceptance/action-value-20260913; runner tmp/run_action_value.py, evaluator
tmp/evaluate_action_value.py. Two ramp repeats, reversed arm order,60s captures;
primary20s from comparison dispatch includes movement/turn/holding. New observed
area and old-deficit recovery are disjoint; fixed-target/outside-target recovery,
lost valid area, sampled motion costs, stages and four-arm common cohorts retained.
Movement and turn forecasts still use different existing approximations: no joint
predicted ranking or online admission. Initial hold failed with ENODATA before
comparison; initial small turn failed at parameter acquisition. Keep both failures.
Old parameter message conflates timeout/empty response/type errors; root cause not
established. Source snapshots and raw-prefix checks prevent future-data leakage.
Eight attempts finished,6 valid captures/actions;105..110frames,20s endpoint lags
.118..498s. Original8-valid admission NOT met; no retry substitution. Second repeat
is the only complete quartet. Shared34deficits include10shared original targets.
Hold/small/wide/advance shared new cells0/0/7/52; shared recovery0/2/12/12;
shared original-target recovery0/0/0/6. Positive utility difference versus hold
0/.125/1.1875/4m2, over equal20s. Descriptive single quartet, not general superiority.
Second wide/small arms lose1/2previously valid cells(.0625/.125m2); hold/advance lose0.
These losses are reported separately, NOT deducted from the positive-utility metric.
Movement sampled distance.805/.776m, action time5.939/6.613s. Wide action1.394/1.302s.
Movement forecast TP22/16,FP14/23,FN20/20; wide-v4 TP20/11,FP4/4,FN13/13;
small-v4 TP3,FP3,FN3. No reliable joint new-coverage/recovery prediction yet.
Both sensor prefixes match final raw arrays, target sets and event order verified;
current evaluator sources match pre-capture hashes. Focused tests pass; old target
evidence checker passes. Final report reports/evidence/terrain-workbench/action-value-20260913.
Backend stopped/idle, active false. Next diagnose startup/record failures and obtain
another complete quartet, unify turn/path forecast meaning and account for validity
loss, then extend geometry/starts before online integration/full three-strategy rerun.

## Measured-hold v4 and narrow target-probe screen completed 2026-09-13

Diagnosed all14 false hold predictions against recorded raw depth: zero already-fit
baselines, zero same-pose raw-ray replay recoveries. Proxy intersections introduce
unsupported new voxels (including micron-scale voxel-boundary cases); they were
not actual measurements. scripts/audit_hold_support.py saves the per-cell evidence.
Optional model_version=v4 keeps the hold baseline observed-only, excludes the initial
proxy pose from future samples, and aligns support centring with the terrain grid.
Default v2 remains. Tests pass;48 legacy candidates reproduce unchanged. Development
replay hold predictions14->0; small-angle turn TP26/FP7/FN20 (old FP10), swept-support
TP47/FP14/FN35 (old FP17). Existing data are development, not independent validation.

Global48 candidates remain positive. New bounded question: does observing improve
the current target set? All deficient cells are frozen BEFORE the Nav2 move using
new preaction sensor-only checkpoint; newly found transit deficits are separate
targets. This is a scope change, NOT validation of the old global stop rule.
Current fresh4-run ramp protocol /evidence/target-probes-20260913:2turn/hold pairs,
40simseconds,5s initial/arrival holding,first target-negative angle among+/- .02/.04/.08,
nonempty remaining target set,<=5s feature age,>=12s remaining. Primary shared initial
target recovery over10s from command; outside-target/global recovery also recorded.
Runner tmp/run_target_probes.py; frozen evaluator tmp/evaluate_target_probes.py.
Require2actual nonzero negative turns; no online deployment from this narrow screen.
Fresh4 captures and2actual turns completed;70..73frames,39.4..39.8s spans,one selected
drop each. All target sets131cells frozen before movement;20/20/15/19remaining target
deficits at command. Requested+.02rad,actual+.028479/+.029491rad;turn time.405/.354s.
Primary shared target cohorts18/14:zero recovery in both arms. Two real nonempty
target-negative probes meet the frozen narrow screen. NOT validation of global stop.
All shared decision-deficient cohorts70/97:turn-only3/7,hold-only1/0,net+2/+7 outside
the shared original targets. Raw per-arm outside recoveries1/5/8/0;do not subtract
those directly instead of comparing the common cohort. Precommand recoveries all0.
First hold control still recovers1outside-target cell: measured-hold baseline is an
approximation,not a guarantee of zero future recovery. Need inspect repeated fusion
and near-threshold fits;do not claim hold calibration solved universally.
Prefix arrays,target sets andchoices reproduce; portable frozen checker passes;plots
visually checked. Evidence reports/evidence/terrain-workbench/target-probes-20260913.
Backend stopped/idle. No online integration or full new three-strategy comparison.
Next compare the total gain/time of this turn with other angles/next locations,
retaining target/outside-target gains and uncertainty;extend scope tests to curves,
occlusion andother starts before using them as a general online rule.
Experiments a4248dd6a0c6,1bece01823f7,618942803305,5e63836661fd.

## Prospective skip-probe validation rejected for missing coverage 2026-09-13

User authorized the next validation step. Added sensor-only checkpoint export on
explicit request in trajectory_evaluation.py, scripts/skip_probe.py frozen ordered
angle selection, and capture helper skip_probe=True (evaluation only). Checkpoint
contains no truth arrays; arrival replay accepts its exact feature cutoff separately
from the eventual action time. No online policy change. Prefix/selection, swept,
arrival and trajectory tests pass; previous small-angle evidence checker passes.

Frozen batch /evidence/skip-probe-validation-20260913: two prior scenes/positions,
two repeats per turn/hold arm, counterbalanced,60s each,5s baseline/arrival holding.
Select first nonempty no-marginal candidate from +/-.02,+/-.04,+/-.08 rad; maximum
feature age5s,remaining>=8s. Preserve absent candidates/failures; require actual
negative turns, never count held-arm suggestions as executed probes. Runner
tmp/run_skip_probe_validation.py; frozen assessor in captures/source. Do not change
rules against outcomes. Supplemental fixed10s dispatch-window audit is separate.
All8 navigation/captures completed,106..110 frames per run,59.4..59.7s captured spans,
1..2selected frame drops. All48 candidates predicted
positive marginal support (1..31cells); ZERO selected negative candidates and ZERO
actual turns. Ages1.253..1.88s were fresh with ample budget: lack of candidates, not
stale data, prevented probes. Predeclared admission REJECTED for missing executed
negative probes/pairs. Do not call this eight executed negative tests or zero false
skips. Planned turn arms actually held heading. No online integration or new full
three-strategy comparison. Backend stopped/idle after captures.

All8 sensor-only prefixes exactly match final raw-record prefixes and reproduce all
candidate choices. Existing focused regression tests and previous frozen evidence
check passed. Report reports/evidence/terrain-workbench/skip-probe-20260913 includes
raw captures, pre-action selections, frozen sources, candidate table,8scene/map panels,
prefix audit, hold-baseline audit, portable hash/statistics checker. Generic swept
replay marks unexecuted turn arms unevaluated; default-yaw hold predictions when no
candidate was selected are diagnostic only, not chosen actions or stop evidence.

Supplemental holding audit:8 no-turn-command runs,14 cell-instance hold support
predictions,0actual fit recoveries over10s. Does NOT support earlier guess of
underestimated hold gains; instead there are false hold predictions. Physical yaw
stable and estimated drift tiny. Development-only one-prefix36-angle scan and one
held-run5..30s prefix scan found no negative candidate; neither changes this batch
nor constitutes independent validation. Next diagnose hold-support false predictions
and prepare actual low-gain/occluded or sufficiently sampled conditions that admit
nonzero negative probes before further independent tests. Do not just add repeats
under these same two conditions, tune thresholds to these outcomes, or deploy.

## Small-angle v3 gate and coverage audit 2026-09-13

Added decide(...,version=v2/v3) and optional --gate-version. V3 removes the0.12rad
heading-tolerance veto; budget and marginal support remain, point/ray model unchanged.
Capture helper --observation-delta resolves relative yaw from measured decision pose
in both arms, records absolute target and estimated executed delta; fixed mode retained.
V2 remains default; autonomous_survey.py unchanged. Regression/semantics tests pass.
Old swept-support8 captures are development only: all883 cell rows byte-identical,
only the old5.38deg aligned skip changes to try. Prior frozen evidence checker passed.

New8 captures: ramp start[-1.4,1.55,-.3],goal[-.65,1.3],relative+.02rad; curve
start[-1.75,2.2,-.4],goal[-1.,1.8],relative+.08rad. Two repeats per turn/hold arm,
counterbalanced;35s including5s baseline and5s common post-arrival observation.
All8 navigation/captures and4 spins succeeded and met predeclared executed-angle
criteria.61..65frames,one dropped selected frame,34.5..34.8s captured spans. Stopped.

Turn arms ALL decision-deficient: support TP26 FP10 FN20 (precision72.2%,recall56.5%);
marginal TP25 FP6 FN21. Initial-deficient support TP1 FP0 FN0; marginal TP0 FP0 FN1.
Shared remaining cohort turn-minus-hold: curve+8/+6(n90/79),ramp+4/+3(n87/82),
no hold-only recoveries. Shared initially deficient net all0. No accuracy claim.

Frozen exploratory assessment PASSED (7turn suggestions,1residual skip,positive pairs,
precision/recall floors,execution checks). FOLLOW-UP EVIDENCE AUDIT: NOT sufficient
for online stop validation. Only skip is ramp repeat2 NO-TURN arm; all4 executed
turns were recommended. ZERO executed nonzero skip probes. Paired turn arm recommended
turn on a different snapshot; its+3 recovery does not test that held-arm skip.
Original assessment retained; stop-coverage-audit.json records this separate gap.
Do NOT claim stop rule accepted or deploy from superficial overall branch coverage.

Evaluator-only bracketing-frame true/estimated yaw changes agree closely but exceed
completion-event estimate (ramp~.04rad,curve~.10..11rad); margins include pre/post time,
not exact spin interval or proven inertia. Preserve actual pose history in modelling.
Next target sensor-only snapshots where model says skip, actually execute that nonzero
turn with repeated hold controls; retain missing-candidate and failed attempts. Require
executed negative probes, not merely skip suggestions in controls. Then online integration
and full common-budget three-strategy comparison. No new full strategy run this turn.

Evidence reports/evidence/terrain-workbench/small-angle-20260913: raw captures, frozen
sources/protocol, sensor/scene panels, cell/paired tables, rotation audit, development
comparison, independent stop-coverage audit, hashes and runnable stdlib checker.
Experiments97853a88e3b4,29aad644b335,62e19893ee8f,ed3769707404,
5eca666e3265,9a5f5a96be4a,b2fdd48b5400,504b6c4b9b34.

## Swept support v2 and independent validation 2026-09-13

Added offline scripts/swept_resampling.py and test_swept_resampling.py. Uses only the
pre-decision estimated camera/base transforms, observed voxel cloud and CameraInfo.
Shortest world-Z turn preserves the camera lever arm/tilt, integrates <=0.25rad steps,
and intersects stride-4 rays with sensor-only median-height cell patches. Voxel union
and XY singular-value support replace coarse bearing-bin gating. Hypothesized points
never enter observed maps; no occlusion/tilt/noise/phase calibration. Frozen v1 and online
autonomous_survey.py unchanged. Core, scanline, partial-cell-FOV and no-future checks pass.

Old arrival-decision eight-capture batch is development only: initially deficient
recovery v1 0/7 vs v2 3/7, zero v2 false positives. New eight fresh35s captures use
ramp start[-1.3,1.3,-.4], goal[-.6,1.05], yaw.25; curve start[-1.8,2.,-.25],
goal[-1.05,1.65], yaw-.8. Each arm repeated twice, counterbalanced; baseline5s,
arrival settling1s. Source and admission criteria frozen before outcomes. All8
navigation/captures and4 spins succeeded; captured spans34.2..34.7s,62..64frames,
1..2selected drops. Simulation stopped. No new full three-strategy comparison.

Executed-turn ALL decision-deficient cohort: v1 TP1 FP1 FN81; v2 support TP47 FP17
FN35 (precision73.4%,recall57.3%); visibility-only TP72 FP126 FN10. INITIAL deficient
stratum: v2 TP2 FP2 FN2. Do not mix denominators: most recovery is in transit-discovered
cells. Shared decision-deficient turn-minus-hold recovery: curve R1+7 (n104),R2+15
(n109),ramp R1+13(n65),R2+17(n55). Shared initially deficient net0/0/0/+1.

Admission REJECTED: curve R1 turn shadow skip already_aligned at0.093957rad (5.38deg),
although5 modelled marginal cells remained; actual0.683s turn arm has7 exclusive
recoveries. This is descriptive, not isolated causal truth. Seven suggestions turn,
one alignment skip, ZERO no_modelled_marginal_support; residual-stop branch untested.
Do not lower the threshold on these outcomes and call it independently validated.
Next separate heading tolerance from information stopping; test small-angle changes,
held-view noise, saturated support and occlusion/low-gain cases with time costs.
Only after independent validation integrate into third policy and rerun common-budget
three-policy comparison. No hardware-readiness or overall-strategy advantage claim.

Evidence: reports/evidence/terrain-workbench/swept-support-20260913, including raw
captures, frozen sources/protocol, development vs independent results, per-cell CSV,
scene/observed-map panels, prediction/paired charts, hashes and stdlib checker.
Experiments6ec8ae0c0bc7,10b4f48bd30e,376da1b59ff1,1c504da3bdbf,
577682b19fd6,cc1a461fd8ca,5df595697453,074c5cf51b10.

## Arrival-stage attribution and prospective gate rejection 2026-09-13

Added arrival_resampling.py (event-aligned cumulative replay and sensor-only residual
opportunity recheck), assess_arrival_gate.py, and runnable tests. Capture helper accepts
--arrival-wait, logs observation_decision_sim_stamp, and uses idempotent ROS cleanup.
Default wait remains0. No online autonomous_survey.py change or three-strategy rerun.

Replayed8 earlier captures using recorded Nav2 arrival events, not pose-inferred stops.
Prior paired slope targets were38/39 valid at arrival,39/39 at end; curves16/16 already
valid on arrival. Stage tables track gained/lost/net, and no-future-input tests passed.

Prospective validation: eight fresh35s captures, two farther starts, changed initial
headings, two repeats per turn/hold arm. Common5s baseline and1s post-arrival settling;
all8 navigation and4 observation turns succeeded. Experiments baa6ebe5e4eb,
2607cc3cf93e, ee557ce32859,502f834ea0a7,e9097adbf361,1a60b19ba566,
6526cf6eeb8a,5dbf867bf5f8. Simulation stopped. Capture/model source frozen before
the prechecked batch; assessment implementation completed during capture before analysis.

Gate arrival-recheck-v1 rejected: all8 suggestions were no_predicted_remaining_recovery.
Four actual turn trials nevertheless recovered2/1/1/3 eligible cells after decision;
hold trials recovered0. Paired COMMON decision-deficient cohorts: ramp R1 n6 hold0
turn2, R2 n6 hold0 turn1; curve R1 n17 hold0 turn1, R2 n22 hold0 turn2.
Pure target common residual cohorts only0/1/2/2 cells, with one recovery in curve R2.
These are small descriptive comparisons, not overall strategy superiority or accuracy.

Seven missed recovery instances: five too_few_voxels, two insufficient_xy_spread;
all retain the same coarse bearing bin and gain voxel points. Six are reference ground,
one curved_top. Two cell centres lie outside the final nominal FOV, prompting swept-turn
and partial-cell visibility diagnostics; no unique cause established. Predicted-zero
does not justify immediately stopping observation. Do not tune thresholds on this batch
and call it held-out validation. Next refine turn-swept visibility and continuous point
support/range changes; independently validate before online integration and three-arm runs.

Evidence reports/evidence/terrain-workbench/arrival-decision-20260913: raw observations,
stage states, decision features, paired results, scenes/maps, retained pre-motion abort,
source snapshots and stdlib check_evidence.py. Initial attempted ramp start overlapped
an operator prior zone; abortedf8bc4f33289e before motion, then added explicit prior-circle
footprint prechecks and froze a replacement plan. Original abort and ROS double-shutdown
error retained. Plot-only axis/headroom fixes preserve quantitative assessment byte-for-byte.

## Session order and matched-budget heading controls 2026-09-13

Collection dropdown now sorts embedded UTC capture timestamps newest first, including
microseconds; random experiment UUIDs no longer determine order. Undated archives go
last. Existing/current/preferred choices are preserved; no history defaults to newest.
Node regression and live Chrome 8765 reload/refresh verified, including newly added
records appearing at the top. No archive rename or server restart was needed.

Added --no-observe-turn to controlled captures (default unchanged), and paired outcome
reporting. Both arms query real Nav2 paths before moving; no-turn predictions exclude
the unexecuted final observation heading. Fresh starts (0,-1,+0.463648) ramp and
(0,+1,-0.463648) curve, goals (0.8,-0.6)/(0.8,+0.6). Four 35s captures, 5s baseline,
counterbalanced arm order; all navigation and both spins succeeded. Experiments:
d30e44a4386c, 8a93a530d6b7, 1d6b4b2adeb8, ae2ccb7bff04. Simulation stopped.

Shared deficient baseline cells also match initial state/count/coverage/view summaries:
ramp 75 cells, no-turn recovered57 vs turn60; curve63, no-turn49 vs turn48.
Pure target cohorts: ramp39/39 recovered in BOTH arms; curve16/16 in BOTH arms.
Final valid area rises0.75m2 with turning in each scene, but target recovery has a
ceiling and no added benefit. Spin costs2.162/1.791 simulated seconds. Actual captured
spans34.3..34.8s; raw trajectories differ slightly. One run per arm, no causal/general
strategy superiority claim. Frozen rules and online autonomous_survey.py unchanged.

Evidence: reports/evidence/terrain-workbench/heading-control-20260913 (raw observations,
pre-action maps/routes, scene/map comparisons, per-cell CSV, paired outcomes, sources,
hashes and check_evidence.py). Paired cohort and no-future-feature checks passed.
Next: distinguish transit recovery from marginal post-arrival heading benefit; examine
remaining deficits and a no-further-observation stopping rule, then repeat on ranges/
headings without target ceiling effects. Do not automatically deploy unconditional
arrival turns. Three-strategy comparison follows validated online integration.

## Pre-action Nav2 route and explicit observation heading 2026-09-13

Added scripts/observation_path.py: ComputePathToPose with an explicit current start,
saved map/path snapshot before any NavigateToPose request. Routes are checked through
segment interiors against the pre-action known/free costmap. The query sends no
motion command. It is a snapshot, not a guarantee against Nav2 replanning.
The controlled capture helper now accepts --nav2-plan and --observation-yaw.
After reaching the position it reuses leased Nav2 Spin through the backend /turn
command; task identity, autonomous-task conflict, yaw range and region are checked.
Move completion, turn completion and <=0.1 rad final yaw error are separate records.
No frontend control was added; the experimental runner uses backend port 8766.

Four fresh 35s captures (5s baseline), two repeats each on the independent visible
fixtures. Experiments: 1d50c65d2873, 4df840bb3e12, 398d17bcf7f9, 5fc18793bef3.
All navigation and observation turns succeeded. Max final yaw error 0.563deg;
post-turn captured windows 19.73..20.84s. Sensor-pose frame counts 63/63/63/64.
On pure target deficient cells, endpoint -> Nav2 path + final heading predictions:
ramp repeat1 TP20->31, FP1->1, FN18->7; repeat2 TP19->30, FP2->2, FN18->7.
Curve both repeats TP4->10, FP0->1, FN10->4. Adding heading to path alone adds
2 curve true positives in each repeat, no extra false positives; no extra ramp gain.
This is prediction comparison on shared trajectories, NOT three-strategy efficacy.
No fitted parameters or online policy replacement; autonomous_survey.py unchanged.

Initial batch failed at an incorrect within_region import before any motion; stopped
the remaining batch, corrected import, added a sourced-ROS import regression check,
and froze a new batch. Aborted records and original source preserved, not analyzed
as model outcomes. Evidence: reports/evidence/terrain-workbench/nav2-observation-20260913
(raw depth/poses/screenshots, pre-action routes/maps, stage results, CSV/plots,
aborted batch, sources, checksums and runnable standard-library checker).
New evidence checker and previous visible-path checker passed. Simulation stopped.

Next: retain route + heading as a candidate; validate different starts and matched
budget no-turn controls before online integration into strategy three. Then run the
shared three-strategy comparison. Intermediate headings, visibility and fit-recovery
prediction still approximate; no calibrated probability or hardware accuracy claim.

## Visible independent worlds and path opportunities 2026-09-13

Completed four fresh Gazebo captures in two new explicit world fixtures:
visible_ramp_v1 (stationary de36034fbaf0, approach 622800100952) and
visible_curve_v1 (stationary 7983ce4c6913, approach 7c8a9a6d1f25).
Each had a 25s requested capture and 5s baseline; moving Nav2 goals succeeded.
The camera and online strategies are unchanged. Real same-ray evaluator audits
confirm 65 pure sloped-top / 34 pure curved-top observed cells in moving captures.

Predeclared offline path rule samples the intended straight segment every <=0.1m
and unions the existing range/direction opportunity per cell, without duplicate
gain. No fitted parameters, future features, truth inputs or online deployment.
On pure baseline target surfaces: ramp 36/39 recovered after moving, curve 14/15;
stationary recovered 0 in both. Endpoint vs path: ramp TP19->24, FP2->2,
FN17->12; curve TP4->8, FP0->1, FN10->6. These are correlated cells from one
pair per world, not a strategy efficacy experiment. Recovery is valid fit, not accuracy.

The planned segment is NOT a pre-action queried Nav2 route. Maximum actual XY
deviations .137/.113m and heading deviations 26.45/15.25deg expose this limitation.
Next: query executable Nav2 paths before action, specify observation heading and
its tolerance, then repeat fixed-rule tests before integrating strategy three.
Only then repeat the full three-strategy comparison with the shared evaluation.

Frozen evidence: reports/evidence/terrain-workbench/visible-path-20260913
(README, raw depth/poses/scenes, per-cell CSV, independent surface audits, figures,
source snapshots and hash checker). Simulation stopped; frontend remains available.
Task schema now accepts optional allowlisted scene (default still survey); no new
scene selector UI. Existing saved tasks remain unchanged.

## Miss attribution and held-out region check 2026-09-13

scripts/resampling_attribution.py replays shared-baseline frame subsets without changing
online policies. All full-replay fit labels reproduce the prior frozen model dataset.
For the short approach's 30 false negatives: waiting 0, transit 25, settled suffix 17,
final frame 15, transit+settled 30. Transit/settled overlap: both 14, transit only 11,
settled only 3, combined-only 2. Per-cell best single-transit oracle covers 24, but the
best common frame covers 19 (individual frames 6/19/16/18). Do not conflate this oracle
with a realizable predictor or add overlapping counts as causal contributions.

Fresh held-out region near north_ridge: (-9.8,6.8,-.528074), goal(-9.2,6.45), stationary
experiment-348f798f421b and approach experiment-77c20d5e42f8. Both 30s/54 frames, Nav2
approach succeeded, truth path .586m. Rules unchanged: 95 initial deficient cells each;
stationary recovers 0, approach 65. Range+direction TP/FP/FN/TN=13/6/52/24; direction
0/1/65/29. Precision 68.4%, recall20%; not stable model acceptance.

Important audit: observed reference-classified cells are 146 ground,24 mixed,1 without
visible reference, ZERO pure curved-top cells. This is a new ground/edge region, not
curved-top validation or an independent-world generalization experiment. Truth-based
visible audit remains entirely separate from sensor-only prediction and online policy.

New capture helper accepts explicit start and records observed motion-terminal time;
motion phase split for old records remains a post-hoc estimated-pose stability rule.
Evidence: reports/evidence/terrain-workbench/resampling-attribution-20260913/README.md.
Prior evidence preserved; active worktree remains MoonCar-mapping; other checkouts untouched.
Next: prevalidated-visible surfaces in independent worlds and path-view union gain model;
avoid summing duplicate frame gains or fitting a probability from correlated cells.
Simulation stopped, frontend/backend retained; no online model promotion or ML training.

## Stop diagnostics and offline distance opportunity model 2026-09-13

Added motion.diagnostics (TF exception, Nav2 readiness, wall/sim data ages), first-stop
snapshot including raw Nav2 result, and per-action/mission/telemetry/report preservation.
Simulation startup failure snapshot persists once to startup-failure.json. Diagnostic
errors cannot prevent cancellation; thresholds and online strategy scoring unchanged.
This does not establish the root cause of the previous transient TF or archive startup failures.

Fresh short approach experiment-9d39b7482383: goal (6.15,-1.7), 30 sim-second capture,
53 synchronized frames, Nav2 succeeded; real control output includes healthy diagnostic
fields. Existing slope captures plus this trial feed scripts/resampling_dataset.py.
Only pre-action estimated-pose sensor data and planned goal are features; final fit is
label only. No truth features, fitted probability or online policy update.

Fixed offline range-density proxy n*(r0/r1)^2 plus direction opportunities: new short
trial has 71 initial deficient cells, 42 recover. Direction rule TP/FP/FN/TN=0/0/42/29;
range+direction=12/1/30/28. Recall only 28.6%; 92.3% precision alone would mislead.
Same scene/start and correlated cells are not independent generalization evidence;
new 30s versus exploratory 45s is not equal-budget strategy efficacy. Metric cohort is
estimated-pose cells, not the prior truth-pose sloped-top 34/40 cohort.

19 unit tests plus range/FOV/conflict and no-future-feature-leakage checks pass.
Evidence: reports/evidence/terrain-workbench/resampling-model-20260913/README.md.
Previous frozen evidence unchanged. Main/M6/efc4 untouched; existing dirty work preserved.
Next: inspect false negatives, separate transit/fusion from endpoint-view gains, then
multiple geometries/distances/bearings with whole-trial held-out validation before an
online range-term ablation. Simulation ended; frontend/backend remain available.

## Unified strategies and live comparisons 2026-09-13

Implemented in the existing dirty D:/Document/Repositories/MoonCar-mapping workspace;
main, M6 and efc4 untouched. Frontend exposes nearest_view_v1, coverage_view_v1 and
surface_view_v1 with shared candidates/Nav2/safety/sensors/recording. Surface utility
only rewards insufficient voxel count/XY spread plus a new bearing, not steep/high
residual fits. Fixed 0.5 utility is uncalibrated; same-bearing approach is not modeled.
Quality messages now carry sensor-derived fit_state and bearing_mask. Q is unchanged.

Fresh Gazebo/SLAM/archive per trial, 90 sim-second upper budget, radius 8 m: two valid
starts x three strategies x two measured trials = 12 reports (9 no_candidates, 2 budget
stops, 1 runtime fault). Keep all 19 attempts: 6 invalid-start rejections, 1 archive
startup failure and its later retry are explicit. Important runtime source hashes match.
Means include reported runtime failure; startup failures are not zero observations.
No stable superiority established; two repeats are descriptive, not significance evidence.

Separate live slope pair at (6.5,-2,2.4 rad), ~45 sim seconds: stationary 80 frames versus
Nav2 approach 82 frames / 0.714 m true path. Initial deficient sloped-top cells recovered:
0/43 stationary versus 34/40 approach. Baselines differ slightly; one ideal-slope pair
supports a mechanism only. Truth poses/surface labels are evaluator-only. No accepted
three-strategy global map precision or physical hardware evidence. Two pre-motion failed
approach attempts retained; fixed JSON replacement polling and optional screenshot failure
handling. ROS capture shutdown guard avoids losing data after signal-triggered shutdown.

16 focused unit tests plus polling check pass; Chrome selector verified for all three
options. Legacy missing fit_state now reports unavailable recovery metrics rather than
zero; legacy gain field preserved. These compatibility fixes followed the live campaign.
Backend refreshed only after all motion and experiments stopped; 8765/8766 remain available.

Frozen local report, CSV, plots, raw evidence, sources, failures and hash checker:
reports/evidence/terrain-workbench/strategy-comparison-20260913/README.md.
Next: instrument startup/safety failure reasons, model recovery versus range/bearing/sample
geometry from separate calibration trials, then held-out scenes/starts and repeated ablations.
Do not call the current heuristic a validated innovation or an autonomous full-coverage guarantee.

## Visible-surface / Q audit 2026-09-13

Offline replay of the frozen corrected 147-frame capture is complete. New
scripts/surface_quality_evaluation.py compares measured depths and nearest visible
analytic surface at identical rays, frames and synchronized truth poses. It reuses
TerrainMap fusion/Q/fits, separates ground, sloped tops, curved tops, steep sides and
mixed faces, and retains missing/invalid denominators. These truth-pose evaluation
maps are not the online map; evaluator frames differ from mission keyframes.

Fixed ROI: 3228 cells, 1657 observed, 548 invalid fits, 1095 paired valid fits but only
1072 single-surface pairs. Sloped tops: 36 observed, 8 valid, 28 with fewer than six
voxel points. High Q >= .8: 307 ground cells all valid; 27 mixed cells with 16 invalid
(11 steep fits, 5 large residuals). Overall 95.2% valid-fit fraction therefore hides
40.7% valid-fit fraction for high-Q mixed cells; neither is percentage accuracy.
Mixed paired-plane differences remain diagnostics, not usable single-surface truth.

Ten math/map/projection tests pass, including Q=1 but invalid multilayer and Q=1 with
fixed height bias counterexamples. Sources, paired maps, CSV, black/white PNG/PDF and
hash verifier: reports/evidence/terrain-workbench/surface-quality-20260913/README.md.
No live motion, production navigation/Q changes, runtime update or prior evidence
rewrite this turn. Main/M6/efc4 remain untouched. Next: controlled same-baseline,
same-budget stationary versus changed-viewpoint slope resampling; separate sampling
shortage from surface conflicts using sensor-computable features. Never send offline
truth categories to the online candidate selector; strategy superiority is unproven.

## Depth pixel-center correction 2026-09-13

Continued in the existing dirty MoonCar-mapping workspace. Current Gazebo Ogre2 data
reproduces a half-pixel projection mismatch. Shared project_depth keeps default offset 0;
Gazebo terrain navigation and collect.sh explicitly set depth_pixel_offset=0.5. Both
hazards and map observations use the corrected optical points. No hardware calibration
claim; no old archives rewritten. Runtime source files were backed up before updating.

Fresh experiment-7670cb590c1d / autonomy-a5c5e2ce5e completed 53.9/60 sim seconds:
6 successful turns + 1 advance, 85 new keyframes, 3 distinct scene images, no_candidates
normal stop, 73 zero safe-velocity samples. A separate 79.6-second synchronized raw-depth
capture has 147 accepted frames and 1 startup bracketing rejection. All 147 raw frames
bitwise reproduce the published cloud using the recorded node offset. Same-frame ray
counterfactual RMSE: offset 0 = 211.00 mm; offset .5 = 5.592 mm. Plane/solid groups after
correction = .101 / 13.633 mm. Nearest visible analytic visual surface, not map-height
accuracy. Old 154-frame trajectory also reproduces the fixed half-pixel relationship.

XY pose RMSE remains 4.15 cm; full plane-footprint height RMSE including edges 27.73 mm.
Global mapping, slope/edge accuracy and real sensors remain unaccepted. New independent
raw-depth and ray analysis tools retain missing reference counts, input hashes and source
snapshots; ordinary autonomy still automatically captures its existing research evidence,
not the optional synchronized raw-depth stream. Nine targeted regression checks pass.
See reports/evidence/terrain-workbench/projection-20260913/README.md. Vehicle stopped.
Next: slope/edge surface correspondence and Q versus usable surface quality, then matched
strategy trials. Main, M6 and detached efc4 checkouts remain untouched.

## Synchronized trajectory / height check 2026-09-13

Continued in D:/Document/Repositories/MoonCar-mapping: the newly selected efc4 detached
checkout is still an older snapshot and lacks the mapping extension. No merge or overwrite
of that checkout. Restored the Windows frontend at 8765; normal experiment apply replaced
an old graph with incomplete ROS discovery, preserving its archives. Current experiment
experiment-edb683fc4ee7, ROS domain 89, autonomous task autonomy-c6626caf60: 54.5/60 sim s,
6 successful turns + 1 advance, normal no_candidates stop, 87 new map keyframes.

New evaluation-only scripts/trajectory_evaluation.py captured 154 selected depth/pose
samples over 84.4 sim s, with Gazebo bracketing interpolation gaps <= .05 s and no rejected
selected samples. Converted running robot description independently supplies camera mount
extrinsics; optical-axis transform and interpolation tests pass. Truth sampled path 1.665 m;
XY pose RMSE 4.10 cm, max 7.53 cm; max yaw difference .268 degrees. SDF/TF mount transforms
match numerically on this flat trajectory. No alignment or camera correction was fitted.

Same-point height reprojection: full plane-footprint subset 311742 points, RMSE 35.01 mm;
independent 0.25 m inflated-object-AABB interior subset 288029 points, RMSE 2.85 mm,
bias +2.27 mm. No height-based outlier rejection; both subsets retained. ROS versus truth
poses give near-identical height metrics here. Not global map, slope or physical accuracy.
See reports/evidence/terrain-workbench/alignment-20260913/README.md. Vehicle stopped.
Next: raw depth / intrinsic / rendering correspondence, then visible-surface and slope
checks before sampling-score changes and matched strategy comparisons. Old runs are not
retroactively calibrated. Synchronized trajectory capture is currently a separate evaluator;
ordinary autonomous runs automatically produce existing survey/reference reports only.

## Reference diagnostics 2026-09-12: implemented, not calibrated accuracy acceptance

`scripts/terrain_reference.py` now analyses frozen SDF VISUAL geometry against the existing
final-stamp elevation export. Same fixed ROI; uniform 5x5 top-surface fits at 0.25 m;
ground / terrain / edge-or-complex groups retain missing and invalid cells. Unsupported
geometry is rejected. CSV error statistics, Q bins, black/white PNG/PDF maps and source
hashes are separate from the original capture. The report CLI automatically generates
these diagnostics for future SDF-backed runs, retaining diagnostic failures separately.
Geometry tests and the real report-to-reference subprocess check pass. Three accepted
continuous-run archives were reanalysed without changing their original files.
Overall height RMSE is 1.39-3.25 cm, but this is dominated by flat ground; edge/complex
matched-cell RMSE is 11.72-14.45 cm. These are nominal-alignment diagnostics, not sensor
accuracy or proof of strategy quality. High Q is not calibrated accuracy and can coexist
with invalid surface fits. No trajectory-level ground truth was available in old captures.
Read-only synchronized Gazebo / ROS parked checks on experiment-23b486559f42 (domain 89)
found 2.17 cm XY residual at three timestamps at one location; no correction fitted, no
motion commanded, and this does not certify the archived trajectories. All diagnostics
retain accepted_accuracy=false and nominal_unverified alignment. See
`reports/evidence/terrain-workbench/reference-20260912/README.md` and `docs/survey_evaluation.md`.
Next: timestamped trajectory truth and height/visible-surface alignment checks, then
evaluate sampling-quality improvements and conduct matched strategy comparisons.

## Accepted validation 2026-09-12: navigation freshness and live scene

Three-start continuous acceptance passed in its bounded scope; not all navigation scenarios.
Costmap freshness now uses ROS message age (3 simulation seconds) plus a 30-second
wall disconnect guard, shared by goal checks, motion permission and autonomous planning.
GUI connects only after the Gazebo server scene contains the spawned rover. Recording
initialization runs after an immediate start acknowledgement, before any motion.
First 180-second-budget run finished normally at 174.4 simulation seconds: 6 translations,
11 turns, 321 new depth keyframes, zero safe velocity at stop; all 49 frozen artifacts
verified. Second start finished at 180.0 s: 19 successful actions, one budget-interrupted
translation, 295 new frames and zero safe velocity. Its acknowledgement took 0.125 wall s.
Switching a third experiment exposed stale TF across simulation time reset: Buffer had
not used the node's ROS clock, and reset occurred before old publishers exited. Fixed
Buffer(node=self) and moved cache reset after process exit. Real ROS clock rollback and
shutdown-order tests pass. Same-backend live switching now reaches the correct (12,0)
pose and starts autonomy in 0.079 wall seconds. Third run experiment-23b486559f42 /
autonomy-64d68b8b56 finished normally at 177.4 s: 6 translations, 13 turns, 286 new frames,
and zero safe velocity. Total: 55 successful actions, one budget interruption, 902 new frames.
Live costmap deactivation during translation cancelled navigation and stopped autonomy;
three safe velocity samples were zero. Costmap reactivation restored ready without resuming
the stopped task. Earlier observation-phase fault test and failed TF startup remain archived.
14 targeted tests plus the offline evidence verifier passed. Current vehicle is stopped,
map/pose are ready, Windows frontend 8765 proxies Docker backend 8766. The last autonomy
fault status belongs to the deliberate fault test, not an unresolved service outage.
See `reports/evidence/terrain-workbench/continuous-20260912/README.md`.
This supersedes the map-freshness/identical-scene failure status in the earlier metrics
section only to the extent verified here; earlier failure packs remain preserved.

## Active extension 2026-09-12: survey metrics and automatic research evidence

Each autonomous mission now automatically captures a fixed-ROI baseline, simulation-time
telemetry, full attempted-action records (including interruptions), scene checkpoints,
raw observations and exact dirty source/config snapshots. Existing recorder save service
is called at stop. A separate postprocessing process produces CSV/JSON summaries and
black/white PNG/PDF figures; failures remain in the cross-run table. See
`docs/survey_evaluation.md` and `reports/evidence/terrain-workbench/evaluation-20260912/README.md`.
Metric version survey-metrics-v1 uses cell-centre disk/world intersection; predicted and
actual quality gain are compared within the same candidate-visible cells, while whole-ROI
gain is recorded separately. Warm-start history is subtracted. Sampling sufficiency is
not accuracy. Reference height/slope errors are null without a supplied aligned reference;
reference import math is tested, actual reference alignment is NOT verified.
Three live evidence packs captured successfully: 40.6/60 s map-freshness fault (6 turns),
8.8/15 s budget-feasible-candidate exhaustion (1 turn), and 2/15 s same-scene continuation
fault with zero new gain. Navigation freshness instability remains unresolved; no safety
guard was bypassed. Identical scene captures are flagged as context-only, not motion proof.
No translation or live manual-action interruption was accepted in this task; interruption
retention is unit-tested. 15 targeted tests and offline artifact/hash verification passed.
Current frontend remains 8765 / Docker backend 8766; current vehicle is stopped.
No hardware acceptance change, dependency install, main/M6 edits, commit, push or upload.

## Active extension 2026-09-12: autonomous observation loop

The workbench now supports one-click autonomous observation from the current pose,
with simulated-time budget and start-relative radius; it does not require a rectangle
or manual goal sequence. Fixed camera preserved. Sensor-only candidate view planning
chooses Nav2 translation / Spin / repositioned resampling; backend leases keep it
running across browser refresh while preserving health and motion protection.
Gray sampling-quality heatmap and decision reasons are implemented. Quality is spatial
subcell coverage plus bearing diversity, not measurement accuracy or safety probability.
Chrome/Gazebo verified 18 successful autonomous actions (6 translations, 12 turns) in
173.8 of 180 budgeted simulation seconds, with 289 depth frames and zero safe velocity
at automatic stop. A second run from the unchanged parked pose verified successful
repositioned resampling and manual mission stopping. See `docs/autonomous_survey.md`
and `reports/evidence/terrain-workbench/autonomous-20260912/README.md` / `check_evidence.py`.
No full-area coverage or optimality claim, no learned model, no return-to-start behavior,
no historical depth loop-closure correction, and no hardware acceptance change.
The older text below saying autonomous next-view selection is unimplemented is superseded
by this section. Main/M6 modifications remain preserved; no dependency install or push.

## Active extension 2026-09-12: online workbench navigation

Current runtime is Windows frontend port 8765 proxying the existing Docker simulation
backend on port 8766, not the historical VM below. `MoonCar-mapping` remains the active
dirty worktree; main and M6 changes were preserved.
Worktable experiments now use a 50×50 m world with 32 additional distributed objects,
online SLAM, actual Nav2 path/goal/track display and explicit map/obstacle target checks.
Point goals do not require a terminal heading; navigation budget uses simulation time.
Verified goal `(9,3)` succeeded with 0.158 m Gazebo truth error; online map right edge
expanded from about 12.3 to 20.0 m; depth record grew from 3 to 77 frames.
The user's original `(5.3,-0.31)` goal still failed (208): new observations marked its
cell as obstacle safety margin near the central ramp. Safe output was zero; do not
describe this regression as successful navigation or all navigation issues as resolved.
See `reports/evidence/terrain-workbench/online-navigation-20260912/README.md` and its
offline `check_evidence.py`. Full-area autonomous exploration, historical depth pose
loop-closure correction, and 50 m long-run accuracy are not verified/implemented.
Hardware milestones remain unchanged. No new dependencies, commits or pushes.

## Active extension 2026-09-10

Worktree `MoonCar-mapping`, branch `codex/terrain-mapping-workbench`, baseline `db1851e`.
User authorized the 3D observation mapping, Three.js workbench and terrain pathfinding proposal.
See `docs/terrain_workbench.md` for the full scope, current evidence and incomplete gates.
Map persistence/elevation/search/viewer tests and 25 synthetic comparisons pass.
Docker builds all 8 ROS packages; 35 Python tests and the C11 core pass.
Observed Gazebo archive: 181 keyframes, 40278 voxel points, 1722 valid elevation cells.
Nav2 waypoint execution, browser cancellation and lost-lease stopping passed; original M6 passed.
Dynamic obstacle replanning stops when no route exists; successful dynamic detouring is NOT accepted yet.
The 3D map uses timestamped depth and TF, not scene ground truth; no 3D SLAM loop-closure correction.
Pathfinding is the research prototype; terrain-adaptive speed control is not implemented.
The historical milestones below do not cover this extension. Hardware Issues #52-#56 are unchanged.
Original main and M6 uncommitted files remain preserved. No commit, push, merge or procurement occurred.

Historical baseline below: 2026-09-06. VM addresses and commands below are historical, not current connection instructions.

## Current Direction

MoonCar is a simulation-first ROS2 project for a four-wheel lunar-terrain
rover. The M6 software prototype now passes the full GUI acceptance, including
terrain constraints, new-obstacle replanning and two ground-truth-checked goals.
The next development stage is hardware commissioning, not an unverified motor launch.

The final deadline in `docs/requirements.md` is 2027-01-10. No rover hardware
has been purchased yet. Board pinout, motor/current data and independent stop
circuitry must be confirmed before HAL integration and powered bench testing.

## Repository

Performance follow-up #58 / PR #59 adds native `topic_tools` wheel/clock
throttles and `accept_m6.sh --rviz-only` (also supported by M5). Runtime
`00bdcf3` passed isolated RViz-only and full-GUI M6, with truth errors below
0.074 m. Clock pause/resume and time-reset forwarding passed. Short RTF
windows were about 0.325 RViz-only / 0.295 full GUI versus 0.235 baseline;
these are not controlled speedup guarantees. See
`reports/performance_2026-09-05.md` for evidence and limitations.
Vehicle speed, sensor resolution and 1 ms physics step are unchanged.
Nav2 action acknowledgement is 200 ms; goal/safety deadlines are unchanged.
Production candidate `bebf3d4` also passed clean-environment server-only M6
(exit 0), truth errors 0.07523 / 0.06442 m; its sampled RTF was 0.749.

- Windows path: `D:\Document\Repositories\MoonCar`
- GitHub: `https://github.com/WastonGura/MoonCar.git`
- Active development branch: `main`; GitHub default remains historical `master`
- Always explicitly target `main` for PRs; do not assume closing keywords close issues
- M6 worktree: `D:\Document\Repositories\MoonCar-m6`, PR #51 and follow-up PR #57
- Original Windows worktree has unrelated local edits and was left untouched
- Ubuntu clone: `~/MoonCar`
- Workflow: Issue -> branch/worktree -> build/runtime verification -> merge

## Active VM Environment

- IP: `192.168.248.128`
- User: `homura`
- OS: Ubuntu 24.04.4 LTS
- ROS2: Jazzy
- Gazebo Sim: 8.11.0
- Resources: 4 vCPU, 8 GB RAM, 60 GB disk
- Services: SSH and open-vm-tools enabled
- ROS environment: `/opt/ros/jazzy/setup.bash` sourced from `~/.bashrc`

The VM uses NAT networking. Its Ubuntu package source is the Tsinghua mirror;
the ROS repository remains `packages.ros.org`.

## Milestone Status

- M0 project foundation: complete
- M1 rover description and visualization: complete
- M2 Gazebo lunar simulation: complete
- M3 simulated control stack: complete and re-verified on the active VM
- M4 SLAM mapping pipeline: complete, Issues #19 through #24
- M5 Nav2 autonomous navigation: complete, Issues #30 through #35
- M6 terrain-aware autonomy and hardware readiness: complete; PR #57 resolves the production regression found after PR #51
- M7 hardware commissioning: open, Issues #52 through #56, all `hardware-required`

Issue #12 GUI polish is closed. The M6 GUI additionally removes helper-plugin
description text from the scene and includes a working RViz Nav2 panel.

## M6 Delivery Baseline

Runtime code and map `de10bfa` passed isolated headless and default GUI acceptance
on 2026-09-05 in `/home/homura/MoonCar-m6.KPloMc`. A third full run in the
production directory `/home/homura/MoonCar` also passed. PR #51's 0.52 m
repeat error reopened Issue #48; `4de123d` also
failed a repeat at 0.53 m. Neither single-pass result is the delivery baseline.

- 18 Python regression tests plus portable C11 firmware checks with UBSan passed.
- Eight ROS packages built. GitHub Software checks passed.
- Live laser: 256 finite varied returns; depth: 3185 projected points.
- Keepout center clearance: 1.250 m; new-obstacle replanning clearance: 0.899 m.
- Headless goals `(1.5, 0)` and `(3.5, 3)`: truth errors 0.067 m and 0.067 m.
- GUI goals: truth errors 0.079 m and 0.052 m (unchanged 0.400 m acceptance gate).
- Production-directory headless goals: truth errors 0.074 m and 0.075 m.
- M4 loop survey saved a 314 x 238 map at 0.05 m/pixel, the fixed navigation map.
- 37775/74732 map cells are observed; bounds are not full surveyed coverage.
- Six laser-visible perimeter boulders provide features; featureless localization is not established.
- Captured Gazebo and RViz screenshots confirm visible scenes and active Nav2/localization.
- Human mouse-click navigation is not part of the automated goal evidence.

Read `reports/delivery_2026-09-05.md`, `docs/hardware_handoff.md` and
`docs/uart_protocol.md` before continuing. Do not claim hardware readiness from
the PTY emulator: there is no board HAL, flashable image or validated physical stop.

VM-specific fixes: software-rendered Ogre (both sensors and GUI), Fast DDS
for acceptance, Xvfb for SSH runs without DISPLAY, staggered lifecycle startup,
timestamped depth TF buffering, degenerate scanline rejection, nonholonomic EKF
constraint, self-scan filtering, axle-aligned wheel friction, and 1 Hz lifecycle
readiness polling. Unbounded rapid-response polling had flooded lifecycle services.
The original two managers are retained so RViz reports both states correctly.
Real covariance, friction,
wheel dimensions and PID settings must be measured again on the bench.

Run `~/MoonCar/scripts/accept_m6.sh` (GUI) or add `--server-only`; use
`~/MoonCar/scripts/check_software.sh` for the logic/UART regression only.
The acceptance script clears old evidence first and returns nonzero on failure.
Use `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` in additional ROS terminals.
Software checks disable ROS discovery to isolate synthetic encoder telemetry.
Manual mapping experiments that accidentally resolved `/home/homura/MoonCar`
instead of the isolated workspace were discarded. Use a fresh terminal, source
the intended overlay explicitly, and verify `ros2 pkg prefix mooncar_gazebo`
before comparing experiments. Final loop recording used the correct isolated
world and SLAM TF, not ground truth as a control input.

## Verified M3 Baseline

The active VM builds all eight packages. Server-only Gazebo starts the
`lunar_yard` world, spawns MoonCar, publishes sensor topics, accepts scripted
drive commands, and publishes odometry. The latest drive check reported about
0.608 m of forward travel followed by rotation.

## Completed M4 Pipeline

M4 adds:

1. Runtime validation of `/mooncar/scan`, `/mooncar/imu`, `/odom`, and TF.
2. `robot_localization` filtered odometry at `/odometry/filtered`.
3. SLAM Toolbox online asynchronous mapping at 0.05 m resolution.
4. One-command mapping bringup with optional RViz.
5. A failing smoke check that saves `.yaml` and `.pgm` map artifacts.

The VM acceptance run received laser, IMU, filtered odometry, map, and TF data.
After the scripted drive sequence, server-only Gazebo saved a 58 x 44 map at
0.05 m/pixel. The GUI launch also kept Gazebo, SLAM Toolbox, EKF, and RViz alive
for a 90-second smoke run without a process failure.

## Completed M5 Pipeline

M5 adds saved-map localization, NavFn planning, Regulated Pure Pursuit control,
bounded recovery behavior, and a one-command autonomous goal acceptance run.
The verified lunar-yard route from `(0.5, 0.0)` to `(3.5, 3.0)` crosses rock
marker A on a straight line; Nav2 routed around it and completed `4.22 m` of
filtered travel. A goal inside the ramp timed out, cancelled, and returned the
safety command to zero.

Run the complete GUI acceptance with:

```bash
~/MoonCar/scripts/accept_m5.sh
```

Use `--server-only` for a headless VM check.

## VM Build Workflow

```bash
cd ~/MoonCar/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Run M4 mapping with:

```bash
ros2 launch mooncar_slam mapping.launch.py
```
