# 离线原型跟踪断链修复与同录像对比

完成：2026-10-22，Asia/Tokyo。

**本段录像上的断链已修复，实际误差和模型不确定性增长均有改善。** Sequence 23 全段 1361 个已有 RGBD 曝光、136 秒、273 次计划定位全部完成，原型从修复前的 44/273 次输出提高到 273/273 次。没有重设坐标原点或删除失败计划。

本次只修离线原型的特征跟踪，并复用同录像的封存旧定位结果进行对照；没有接入工作台、修改回访或导航，也没有启动新的仿真实验。

## 结果

连续性按全部 273 次计划计数：

| 方法 | 有效输出 | 最后输出的录像时间戳 |
|---|---:|---:|
| 修复前的离线原型 | 44/273 | 22.4s |
| 当前旧定位方案 | 269/273 | 134.9s |
| 修复后的离线原型 | 273/273 | 136.9s |

以下误差和不确定性严格比较双方均有输出的 **269 个相同曝光**，共同时间范围为 0.9–134.9s，269 点均有独立真值：

| 指标 | 当前旧定位方案 | 修复后的离线原型 |
|---|---:|---:|
| 实际 XY 误差 P95 | 22.665mm | 9.396mm |
| 实际 XY 最大误差 | 24.860mm | 12.113mm |
| 模型 XY σ：共同起点 → 共同终点 | 5.243 → 49.896mm | 6.041 → 9.291mm |
| 模型 XY σ 增长量 | 44.653mm | 3.250mm |

共同曝光上的实际误差 P95 下降约 **58.5%**。模型不确定性增长也明显减缓，且此次结果覆盖整段录像，不再只来自断链前的短前缀。

σ 指 XY 协方差最大特征值的平方根。新原型采用条件协方差 C 额外加入一次旧方案初始平移协方差 C0 的敏感性口径，C0 不进入优化器、不随帧反复累加。此口径依赖独立起点误差假设，不能当作已标定的完整后验。条件 C 在共同起终点的 σ 为 3.000 → 7.751mm。

补充全段计数：新原型 273/273 次输出，272 点有真值，实际误差 P95 为 9.597mm，最后一次输出 σ 为 9.258mm。末端一个曝光没有真值，仍计入输出及 50mm 模型门统计，不外推真值；这些全段数值不与上表的共同曝光数值混用。

## 两处具体修复

1. **微小运动时错误删除特征身份。** 旧代码把“亚像素精化没有改善图像块误差”当成轨迹失效。现在只有在真实正反光流已通过、位移不超过原有 0.5px 像素误差时，保留当前图像中未精化的像素位置；仍重新采样当前深度和协方差。22.9s 的共享点由 2 条恢复为 73 条。
2. **转动时搜索初值不合适。** 旧代码总从原像素开始搜索，61s 附近转弯时丢失了仍在视野内的点。现在用当前曝光已有的 IMU 姿态、云台外参与相机内参投影上一帧观测，仅作 KLT 搜索初值。投影不直接产生对应关系或位姿约束；无效投影回退原搜索。61.4s 的共享点由 14 条恢复为 42 条。

真实图像的正反检查、当前深度、三维一致性、空间支撑及至少 24 个共享点的门槛继续执行。没有使用真值、未来曝光或新估计位置来创建匹配。

## 验证口径与范围

- 使用与上一轮相同的 Sequence 23 完整 manifest 和原始曝光数据，逐 chunk 核验哈希；优化默认六帧、每帧最多 96 个观测、0.5 秒计划间隔、0.1° 姿态先验与 3mm 测量参数均未调整。
- 复用原封存基线，核验旧定位器源码、输入 manifest、完整时间序列及输出哈希；1361 行定位字段逐项一致。缓存位姿不进入新估计器。本次新增的 `--baseline-run` 只避免重算这份相同基线。
- 真值在全部估计结束后读取，仅按时间插值并固定初始平移；没有拟合轨迹旋转或尺度。
- 35/35 项专用测试通过，耗时 3.527s。正式回放首末 10 个源码哈希一致；核心模型、数据读取、旧定位器以及导航/回访等 6 个文件与上一轮封存哈希一致。
- 中间一版在 60.9s 后仍断链，已停止该次回放并保留失败前缀。上述结果仅来自最终冻结版本的一次完整回放，未混入中间尝试的输出。
- 这是对同段已有录像的离线管线比较；不同曝光间姿态与公共误差目前仍采用独立先验。结果支持本段数据上的改善，不能据此认定真实跨时相关性已标定或在线时效已通过。

## 可核查证据

- [完整统计与曲线数据](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-tracking-repair-20261022/summary.json)
- [共同曝光起终点](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-tracking-repair-20261022/common_endpoints.json)
- [源文件与证据完整性核验](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-tracking-repair-20261022/integrity.json)
- [测试记录](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-tracking-repair-20261022/tracking-final-tests-01.log)
- [逐计划原型输出](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-tracking-repair-20261022/sequence23-tracking-final-01/window.jsonl)
- [原型说明与复现入口](D:/Documents/Projects/Repositories/MoonCar-mapping/scripts/offline_rgbd_window/README.md)

![同一录像的不确定性与实际误差曲线](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/rgbd-window-tracking-repair-20261022/comparison.png)
