# 在线勘测默认定位切换（2026-10-20）

工作台仿真服务及地形在线勘测现在默认使用已验证的 `direct` 局部激光定位。新启动的工作台服务无需设置激光环境变量；地形在线入口只需 `online_mapping:=true`。已运行的服务仍保留启动时的选择，需先结束实验、重启后端服务，再新建实验。Windows 启动脚本会复用既有服务，不能靠刷新页面或重新运行脚本切换旧实验。使用和回退见[模式说明](../docs/local_metric_localization.md)。

## 更改范围

仅三份生产文件相对上一轮验收源发生改变：

- `scripts/simulation_experiment.py`：缺省选择 `true/direct`；服务构造时保存选择，运动联锁与每次子启动显式参数使用同一份选择，避免环境后来改变造成配置分叉。
- `containers/experiment.launch.py`：缺省 `true/direct`，继续接受显式参数和容器服务环境覆盖。
- `terrain_navigation.launch.py`：激光开关默认跟随 `online_mapping`，模式默认 `direct`；无参数离线 AMCL 保持原路径。

在线显式 `laser_odometry:=false` 可回退原轮速／IMU＋EKF＋SLAM；旧激光到 EKF 研究候选须显式指定 `laser_odometry:=true laser_odometry_mode:=ekf`。工作台对应的环境变量须在容器服务进程中设置。独立 M4 mapping 入口和底层独立节点保留原配置。R5 共享工作台继承新默认，但本次没有验收 R5 定位精度。

匹配算法、测量窗口、质量联锁策略、原物理模型及精度门槛均未改动。冻结记录包含 368 份生产源；此前的验收报告、失败记录及证据保持原样。

## 验证

在两个定位环境变量均未设置的条件下，从冷启动执行一组完整 Gazebo 流程：前进、请求左转 120°（实际 122.263°）、静止、世界暂停、通信断开积压、再次前进及同会话补传。全部功能和定位门槛通过。运行时直接查询证实激光节点 `direct_mode=true`，速度中继、地形处理和记录器均要求定位质量；进程记录只有一个激光定位节点，没有 EKF。

| 测量 | 本次结果 | 原门槛 |
| --- | ---: | ---: |
| 转向末 XY 误差 | 0.460 cm | ≤ 3 cm |
| 转后静止 XY P95 | 0.853 cm | ≤ 3 cm |
| 静止位置波动 | 0.800 cm | ≤ 1 cm |
| 全阶段 XY P95／最大值 | 0.908／1.698 cm | 如实记录 |
| 帧／原始点 | 32／67,682 | 字段和回放精确 |
| 最终传输队列 | 0 | 全部 32 帧确认 |

八个 ROS 包构建通过；已有工作台服务测试 9 项、新增实际 ROS LaunchContext／隔离构造测试 9 项通过。新增测试覆盖在线默认、离线原配置、显式关闭、旧 EKF 候选、环境覆盖、质量参数逐层传递，以及服务环境变化后联锁与子启动仍然一致。本轮新增物理仿真 1 次、硬件运行 0 次；此工况用于默认入口集成回归，不作为新的独立算法验证。

证据位于 [localization-default-20261020](evidence/localization-default-20261020)，包含[预运行冻结](evidence/localization-default-20261020/production-freeze.json)、[实际默认选择](evidence/localization-default-20261020/default-left120/default-selection.json)、[定位评价](evidence/localization-default-20261020/default-left120/localization-evaluation.json)、[功能评价](evidence/localization-default-20261020/default-left120/validation.json)、[最终核验](evidence/localization-default-20261020/verification.json)和[环境清理](evidence/localization-default-20261020/runtime-cleanup.json)。上一轮算法及断测验证见[定位修复报告](LOCALIZATION_REPAIR_20261020.md)。

固定初始参考仍要求原特征持续可见。本次只提升默认入口，没有增加跨区域参考维护、全局回环、坡上行驶或实车能力；离开适用范围时仍按原联锁停止。未提交、推送或操作硬件。
