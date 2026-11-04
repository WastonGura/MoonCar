# 多关键帧原型最小在线适配与首轮 V2 旁路

日期：2026-10-22，Asia/Tokyo。

**独立旁路已接通实时数据并完成一轮采集；本轮未通过在线实时性验证，预定平移和转向路线未执行，动态定位精度仍未验证。** 没有替换默认定位、修改回访或追加第二轮。

## 本次交付

- [online.py](D:/Documents/Projects/Repositories/MoonCar-mapping/scripts/offline_rgbd_window/online.py)：实时测量到原滑窗的纯适配、固定计划、单槽待处理和独立 JSONL。
- [rgbd_window_shadow.py](D:/Documents/Projects/Repositories/MoonCar-mapping/scripts/rgbd_window_shadow.py)：独立 ROS 订阅节点，读取同曝光 RGB/深度、CameraInfo、IMU 及曝光时刻相机 TF；只记录结果，不发布定位、控制或 TF，不读取真值、地图或旧定位位置。
- [evaluate_shadow.py](D:/Documents/Projects/Repositories/MoonCar-mapping/scripts/offline_rgbd_window/evaluate_shadow.py)：关闭采集后评价全部固定计划、误差、失败、丢帧和延迟。

原六帧/96观测/至少24共享点及 0.1°、3mm 模型保留。旁路只记录条件协方差 C，不把离线评价中的 C0 输入估计。IMU 使用 sensor QoS；RGB/深度与当前仿真可靠传输匹配。已有核心、前端、默认定位器、生产启动、导航、回访、场景及传感器等 25 个源文件首末哈希一致。

新增适配检查 10/10、统计检查 14/14 通过；另在无 Gazebo 的隔离 ROS domain 中实际启动节点并发 SIGINT，确认线程终结、`run_end` 完整且退出码为 0。没有重跑既有历史录像或原 35 项来扩大本轮范围。

## 唯一实际实验与提前停止

实验：`experiment-86e0924c756d`，原生无标柱 `survey_v2_native`，起点 `[0,0,0]`，legacy 车型，保持现有 RGBD 默认定位和 6CPU/6GiB/1GiB 共享内存配置。原 Docker 守护进程与工作台原本未运行，本轮正常恢复已有环境，没有重建镜像或修改生产配置。

本轮通过原生产入口做有界启动观察，取得真实连通地图后计划停止该入口，再执行固定 `等待5s → 前进至[1,0] → 转向45° → 等待5s`。数据窗上限为 180 墙秒/90 仿真秒，初始化观察限 55 墙秒/25 仿真秒，原定位失去健康或故障计数增加即停止。

实际启动观察取得了连通状态，但原自主入口从初始化返回后立即开始下一项探索转向。验证器检查到这项已派发动作，取消并结束本轮，**没有启动固定路线 driver**。这是本轮验证流程在启动观察交接处的边界停止，不是定位故障停车。没有继续修复该流程、改目标或重跑世界。

评价窗口固定为首次真实配对 0.752s 至终止状态 33.438s，共 **32.686 仿真秒**；清理尾段单列。真值路径仅 **0.205mm**，累计偏航变化约 **0.000435rad**，按预设速度阈值分类的车体移动和转向时间均为零。云台观察产生了视角变化，但不能替代底盘行驶/转弯覆盖。

## 误差、连续性和延迟

| 指标 | 本轮结果 |
|---|---:|
| 真实同曝光 RGBD 配对 | 327 |
| 前端实际处理曝光 | 132 |
| 被最新曝光替换 / 拒绝 | 193 / 2 |
| 固定计划输出 | 36/66，54.5% |
| 无输出计划 | 30 |
| 最大连续无输出计划 | 17 |
| 按结果实际交付计算的最长无新输出时间 | 7.725 仿真秒 |
| 可评分输出 | 36，均为静止工况 |
| 静止段实际 XY 误差 P95 / 最大值 | 1.156 / 1.297mm |
| 条件模型 XY σ P95 / 最大值 | 4.843 / 4.880mm |
| 交付时观测年龄≤0.3s | 4/66 个计划，6.1% |
| 成功输出交付时观测年龄 P95 / 最大值 | 0.887 / 0.913 仿真秒 |
| 配对接收至结果就绪墙时 P95 / 最大值 | 1.584 / 1.680 秒 |

无输出的 30 次计划包括：24 次曝光在处理前被最新待处理曝光替换、4 次共享或空间支撑不足、1 次曝光 TF 等待超时、1 次关闭前尚未处理。所有失败及迟到结果均保留，没有使用 0.3 秒年龄门筛掉坏结果，也没有重打位置时间戳。

延迟统计有 41 次具备开始/结束记录的计划尝试，包含失败；未开始计算便被替换的计划没有虚构计算耗时。成功输出的年龄统计分母为 36。按原曝光时间与实际交付仿真时钟回看，新鲜状态仅覆盖窗口的约 0.79%，最长不新鲜区间约 30.338 仿真秒。这是旁路记录的事后时效统计，不是对旁路授予生产健康或运动许可。

误差仅用独立真值的时间插值评分，固定首个成功曝光 1.3s 的平移原点，按明确的初始 IMU 偏航和本次已声明零偏航偏置转换坐标；没有拟合轨迹旋转或尺度。初始化的零误差不算精度收益。C 是条件模型量，没有外加 C0，也未完成真实跨时误差标定。

本轮观察器精确 TF 记录与 36 个成功曝光没有共同时间戳，**没有同时间的新旧定位精度对照结论**。静止段较小误差不能说明行驶时仍保持上一轮离线精度，也不能用较小的条件 C 证明在线累计不确定性已改善。

## 原方案与采集收尾

就绪后 273 次工作台状态轮询中，原定位均为健康，故障计数保持 1→1。该计数只描述已轮询状态，未冒称逐条原生健康独立审计。

旁路、观察器和 bag 均退出码 0、未强制终止；旁路线程正常结束并写出完整 `run_end`。原始观察器全档 3357 个真值记录由 3325 个匹配和 32 个明确缺失构成，全部有去向。它包含清理尾段，与评价窗口内的 3318 个真值记录分母不同。

只停止本轮实际实验身份，最终 `phase=stopped`，所属仿真子进程为 0。整个驱动及清理用时约 78.101 墙秒。工作台服务保留，默认定位与回访源文件保持原样；没有第二轮实验。

## 可核查证据

- [完整统计](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-shadow-v2-20261022/evaluation/summary.json)及[逐计划评分](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-shadow-v2-20261022/evaluation/plans.jsonl)。
- [旁路原始事件](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-shadow-v2-20261022/shadow.jsonl)、[策略与预算](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-shadow-v2-20261022/policy.json)、[实验结果](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-shadow-v2-20261022/result.json)。
- [终止状态](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-shadow-v2-20261022/terminal-state.json)、[所属世界退出](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-shadow-v2-20261022/owned-stop.json)、[观察器完整性](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-shadow-v2-20261022/observer/complete.json)。
- [适配测试与 ROS 启停收据](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-shadow-v2-20261022/tests/adapter/adapter-tests-01.json)、[统计测试](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-shadow-v2-20261022/tests/evaluation-tests-01.log)。
- [源文件与证据哈希](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-shadow-v2-20261022/integrity.json)、[原始大文件位置与哈希](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-shadow-v2-20261022/raw-files.json)。完整原始 bag 留在该清单指定位置，没有额外复制。

本轮结论是：实时订阅、估计和独立记录可以工作，当前处理与调度下的输出连续性和延迟不足以接管在线定位；固定运动验证也尚未完成。本轮按授权只做最小适配和一次旁路，至此停止，不继续增加优化、回访修复或实验。
