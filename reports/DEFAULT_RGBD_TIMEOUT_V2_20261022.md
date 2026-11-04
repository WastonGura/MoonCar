# 默认 RGBD 观测超时修复与 V2 运动旁路验证

日期：2026-10-22，Asia/Tokyo。

**在本轮封存的资源配置下，默认定位观测超时未再出现，固定平移和转向路线完成；新滑窗方案的动态旁路取得了真实误差样本，但连续可用率仍不足以替换默认。** 没有替换 RGBD 默认算法、放宽时间或几何门限，也没有修改回访。

## 直接原因与最小修复

上一轮首个超时曝光为25.1s：进入 worker 时真实接收墙龄14.7ms，排队1.48ms；解码66.6ms、配准505.7ms，总计算572.3ms，结束锁等待约5微秒。结果的仿真年龄仅0.152s，仍在0.3s要求内；实际拒绝条件是从接收起超过0.5墙秒。因此不通过更新旧时间戳、放宽时限或调整健康检查顺序掩盖该问题。

只对该段已有原始数据复现了252个实际处理调用，249个原生发布位置/C复现误差在浮点舍入范围、所选参考ID一致；25.1曝光的同20参考状态离线配准约73ms。离线负载不同，不能据此隔离出在线耗时的完整因果，但足以支持先减少并发计算争用，没有改写 BF 匹配器或继续扩大算法排查。

最终修改只增加显式启动配置及验证工具：

- `rgbd_odometry_node.py` 直接运行且显式设置 `MOONCAR_RGBD_CPUSET=0,1` 时，在导入 ROS/OpenCV 前绑定CPU。普通模块导入及环境未设置时不执行该操作；默认定位原有函数、类及算法的 AST 均保持一致。
- `scripts/rgbd_shadow_runtime.py` 按实际实验身份、PID、启动时间和命令绑定进程：验证默认定位线程位于CPU0–1，将该实验 Gazebo 线程限制到CPU2–5；旁路从启动起使用CPU2–5和nice19。每次工作台轮询复核全部线程，允许合法的非空CPU子集，越界保留实际线程信息并停止。
- 总额度仍为6CPU、6GiB内存、1GiB共享内存。其他 ROS/证据进程保留原配置，仍共享CPU额度与缓存；这不是独占核，也不保证任意负载下的时限。结论仅适用于此次封存配置。

原默认定位质量门0.3仿真秒/0.5墙秒、噪声、几何、参考选择、地图安全门和全部回访源码保持。安装包装脚本时保留可执行权限，后台启动清单核对实际源码及安装件。

原wrapper29、worker13、计时3及新增启动3项共48项通过；资源工具最终8项通过，共 **56项针对性检查通过**。真实无世界 ROS/OpenCV 子进程完成特征匹配、LK及两次RGBD拟合后，17个线程均保持指定CPU范围。

## 启动失败单列，随后完成固定运动验证

本次实际创建过两个世界，不能写成只有一次启动：

1. `experiment-8690d6d84dfa`：首版在进程运行后设置CPU，随后严格掩码监视中止，首个云台动作被取消，车体平移/转向均未开始。旧监视没有记录失配线程掩码，不能事后判断是超出范围还是合法子集；此失败未删除、未记为成功验证。它促成了导入原生库前设置亲和性和监视诊断修正。
2. `experiment-bd3ef8a69286`：最终配置下的完整运动验证。原生无标柱 `survey_v2_native`，起点`[0,0,0]`、legacy车型、原默认RGBD。依次完成6个固定有界云台动作、等待、导航到`[1,0]`、`yaw=π/4`转向请求及结束等待。整段保留默认失健或故障计数增加即停止的护栏。没有再开第三个世界。

最终实验驱动结果为 `route_complete`，全部10项动作成功。独立真值按真实动作时间窗确认：

| 动作 | 实际覆盖 |
|---|---:|
| 导航至[1,0]动作，34.691–40.114s | 净XY位移0.79794m，路径0.80329m |
| yaw=π/4转向动作，41.318–43.597s | 实际净偏航55.59°，伴随XY位移4.13cm |
| 全评价窗 | 车体路径0.87024m；有效移动6.908s、转向6.792s |

转向请求是45°，真值净变化约55.59°；这里证明实际转动覆盖，**不能写成45°跟踪精度通过**。本轮未继续修改控制器或调查转角差异。

## 默认定位：本轮未再超时

就绪屏障为12.595s，终止状态53.218s。

| 默认定位指标 | 结果 |
|---|---:|
| 就绪后工作台状态采样 | 353次，全部健康 |
| 工作台故障计数 | 始终0 |
| 就绪后原生质量状态 | 1161条，无不健康状态 |
| 原生计算前 / 计算后观测超时累计计数 | 0 / 0 |
| 采集全档处理完成 / 发布计数 | 522 / 522 |
| 就绪后完整计算计时记录 | 407个不同曝光 |
| 解码+配准墙时P95 / 最大 | 0.11983 / 0.16714s |
| 配准墙时P95 / 最大 | 0.10160 / 0.15416s |
| 计算后曝光年龄P95 / 最大 | 0.136 / 0.200仿真秒 |
| 线程越界监视 | 无越界记录 |

原生计数包括关闭前采集尾段；计时表仅统计就绪后有完整字段的曝光，未虚构缺失计时。API与原生记录描述的是实际采集到的状态，不推断未观察回调。默认超时问题在此次完整运动验证中得到改善验证，不外推为所有资源配置或长期任务均已验收。

## 新滑窗旁路：有动态精度样本，连续性仍不通过

主评价窗为首个真实配对1.462s至终止状态53.218s，共51.756仿真秒；其中包括就绪前初始化及driver正常结束后的停车观察。世界、采集器关闭尾段不混入主窗。

| 指标 | 本轮 |
|---|---:|
| 真实RGBD配对 / 实际处理 | 518 / 358 |
| 替换 / 拒绝 | 158 / 2 |
| 100ms计划输出 / 无输出 | 207/518（39.96%）/ 311 |
| 移动条件计划输出 | 21/70，XY误差P95 3.947mm，最大4.191mm |
| 转向条件计划输出 | 21/67，XY误差P95 4.095mm，最大4.191mm |
| 纯转向条件计划输出 | 3/8，XY误差P95 4.088mm |
| 全部成功输出XY误差P95 / 最大 | 4.803 / 4.813mm |
| 条件模型XY σ P95 / 最大 | 4.969 / 5.121mm |
| 端到端墙时P95 / 最大 | 0.63083 / 1.09421s |
| 成功输出交付年龄P95 / 最大 | 0.398 / 0.593仿真秒 |
| 最长无新结果交付 | 2.051仿真秒 |
| 按实际交付积分的新鲜时间占比 | 23.04% |
| 最长不新鲜区间 | 2.166仿真秒 |

所有207个结果均在主窗内交付并有真值评分。移动与转向分类可重叠，各自只有21项成功输出；误差是条件样本结果，不能代表全部运动时间或证明新方案已经在线可用。

311项无输出包括211项计划区间缺少可用曝光、86项被替换、11项共享/空间支撑不足、2项精确IMU括取等待超时、1项关闭时缺少曝光；失败没有被删除。本轮只处理默认定位超时，没有扩展修复旁路调度或跟踪。

墙时统计来自220条有起止记录的计划尝试，包含失败，分母与207个成功输出不同。150/518个计划满足交付年龄≤0.3仿真秒，146/518同时满足端到端≤0.5墙秒。11个同时间精确基线交集样本中，新/旧XY误差P95为2.764/2.443mm，仅为小型共同样本附表，不宣称新方法胜出。C仅为条件模型量，未加C0或拟合轨迹旋转、尺度。

## 收尾与证据

最终实验driver、旁路、observer、bag均退出码0，未强制终止；世界`phase=stopped`、所属仿真子进程0。完整驱动含清理约108.154墙秒。27个受保护默认核心/质量/旁路/导航/回访/场景/传感器源文件首末一致；包装脚本仅增加显式启动钩子，原算法AST一致。工作台服务保留。

- [默认定位健康与计时](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/default-rgbd-timeout-v2-20261022/default-health-summary.json)、[真值动作覆盖](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/default-rgbd-timeout-v2-20261022/physical-actions.json)。
- [旁路完整评价](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/default-rgbd-timeout-v2-20261022/evaluation/summary.json)、[逐计划结果](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/default-rgbd-timeout-v2-20261022/evaluation/plans.jsonl)。
- [完成的实验](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/default-rgbd-timeout-v2-20261022/result.json)、[实际线程分配](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/default-rgbd-timeout-v2-20261022/runtime-affinity.json)、[首版启动失败](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/default-rgbd-timeout-v2-20261022/failed-startup/result.json)。
- [启动钩子与48项检查](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/default-rgbd-timeout-v2-20261022/tests/startup-affinity/receipt-01.json)、[最终8项资源检查](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/default-rgbd-timeout-v2-20261022/tests/runtime-tests/focused-tests-02.json)。
- [源文件和副本哈希](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/default-rgbd-timeout-v2-20261022/integrity.json)、[大文件原位清单](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/default-rgbd-timeout-v2-20261022/raw-files.json)。bag及冻结上下文不重复复制。

本次默认定位超时修复与动态旁路验证到此收束。新滑窗方案继续保持旁路，默认算法与回访不变，不追加其它修复或验证轮次。
