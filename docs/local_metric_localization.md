# 局部激光定位模式

`direct` 是使用水平激光的局部定位后端。它用实际几何匹配取得 XY，用同步 IMU 取得朝向；轮速只提供匹配初值。算法不读取 Gazebo 真值、世界几何或 SLAM 修正，也不把未测得的横向速度设成零。[定位修复报告](../reports/LOCALIZATION_REPAIR_20261020.md)记录此前显式模式的验收及失败；[默认切换记录](../reports/LOCALIZATION_DEFAULT_20261020.md)记录 2026-10-20 的入口更改。之后的无标柱与坡面扩展状态见 [RGBD 说明](rgbd_metric_localization.md)及[扩展修复报告](../reports/LOCALIZATION_EXPANSION_REPAIR_20261021.md)，不能把旧 direct 验收套用到新后端。

## 使用

在已构建并加载当前 ROS 工作空间的环境中，地形在线勘测只需：

```text
ros2 launch mooncar_navigation terrain_navigation.launch.py online_mapping:=true laser_odometry_mode:=direct
```

工作台的**仿真服务进程所在容器内**可使用：

```bash
MOONCAR_LASER_ODOMETRY=true MOONCAR_LASER_ODOMETRY_MODE=direct python3 /source/scripts/map_workbench.py --simulation --sessions /evidence/local-metric-sessions --host 0.0.0.0 --port 8766
```

以上命令要求先构建并加载当前 `install/setup.bash`，并显式选择 direct。服务在启动时固定定位选择，并将同一选择传给每个新实验，确保定位、运动联锁和记录器一致。实际默认值以当前服务配置及新实验档案为准。已经运行的服务保持原设置，需先结束实验、重启工作台服务，再新建实验切换后端。`start_workbench.ps1` 会复用既有服务，不会中断当前实验或动态切换旧地图。

启动时保持静止，等待 `/mooncar/laser_odometry/status` 的 `healthy=true`。需要复核旧方案时可显式回退：

| 方案 | 在线 ROS 启动参数 | 工作台服务环境 |
| --- | --- | --- |
| direct 局部激光 | `laser_odometry:=true laser_odometry_mode:=direct` | `MOONCAR_LASER_ODOMETRY=true MOONCAR_LASER_ODOMETRY_MODE=direct` |
| 原轮速／IMU＋EKF＋SLAM | `laser_odometry:=false` | `MOONCAR_LASER_ODOMETRY=false` |
| 旧激光到 EKF 研究候选 | `laser_odometry:=true laser_odometry_mode:=ekf` | `MOONCAR_LASER_ODOMETRY=true MOONCAR_LASER_ODOMETRY_MODE=ekf` |

工作台环境指容器内服务进程的环境；宿主变量不会自动传入已运行的 Docker 服务，应通过 `docker exec -e` 或容器内设置。旧方案不套用 direct 的精度验收。

无参地形启动仍为离线 AMCL，激光默认跟随 `online_mapping`，不会要求不存在的激光质量发布者。独立 M4 `mooncar_slam/mapping.launch.py` 和底层独立 ROS 节点保留原研究入口；独立记录 direct 在线定位数据时，须自行设置 `require_laser_localization:=true`。R5 共享工作台也继承在线新默认，但本次切换不构成 R5 定位验收。

## 定位与地图

初始静止扫描建立原点实测参考，后续维护至多 20 个已注册的实测局部参考，保留原点并支持跨区接力。匹配使用有物理尺度的邻域估计法向，避免参考点反复堆积后把噪声团当作线段。普通成功匹配保持原输出；只有原全点残差失败时，才进行一次受限的 3σ 内点重拟合。该回退要求至少六点且超过当前全部有效回波的一半，并重新检查二维可观测性和 10cm 修正上限。参考维护的核心回归不等于实际全场验收。

每个输出必须包含当前新扫描的成功匹配。估计窗口最多保留 0.4 秒、八条原始位置；根据测量选择常位置或常速度模型，实际拒绝会清空窗口。等间隔等权的小运动可能产生约 0.2 秒滞后，不能把它当作所有情况下的硬上限。协方差下限是保守的软件处理，不是精度认证。

这个后端唯一发布 `odom→base_footprint` 和 `/odometry/filtered`，该模式不启动 EKF。位姿只用真实成功扫描的时间戳，缺失或拒绝时不刷新 TF。每次成功立即发布对应质量状态，另保留 0.1 墙秒心跳；心跳不会刷新位置测量时间。

新建的 SLAM mapper 仅将这些位姿用于占据图，显式关闭二次网格匹配和回环校正。这样图内位姿和 `map→survey_odom` 保持一致，而不是单独隐藏一个仍然存在的地图修正。只有完成连续三次真实匹配后，合格扫描才进入 `/mooncar/scan_localized`。原默认 SLAM 配置文件及原模式保持。

## 失效与边界

有效测量最长 0.3 仿真秒、状态最长 0.5 墙秒，时钟及 TF 保护不变。真正失效时，独立速度中继清空旧命令并停发运动，工作台取消原动作，采集停止并丢弃故障积压帧。三次新匹配恢复后仍需新动作，不自动续跑；生产者更换或时钟倒退必须开启新实验。

有界参考库允许已注册局部视图接力；当前视野若与所有保留参考缺少共同特征、严重遮挡或几何约束不足，仍会拒绝定位。有限参考维护不保证全场覆盖，也不提供全局回环。鲁棒回退不能排除一致动态物体、共同 IMU 朝向偏差和重复几何误配。二维水平过滤在车体明显倾斜时会停止提供合格扫描。本轮有限平地仿真不能代表坡上行驶、长期任务、R5 模型或实车标定验收。
