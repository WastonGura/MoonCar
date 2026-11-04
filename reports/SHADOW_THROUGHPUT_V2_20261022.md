# 旁路丢帧与迟到：原因、局部优化及单轮 V2 结果

日期：2026-10-22，Asia/Tokyo。

**完成了直接瓶颈确认、等价筛选优化和一轮 V2 平移/转向验证；剩余丢帧和迟到没有完全解决，实时性验收未通过。** 配对曝光丢弃率由37.33%降至24.95%，但墙钟端到端P95由0.609升至0.914秒。默认定位方案、回访及导航流程保持原样，未追加世界或扩大修复。

## 已确认的原因与修改

上一轮525个精确RGBD配对中，195个在配对后被单槽pending覆盖，另1个相机TF就绪等待超时；关闭时另有1张RGB未配对。配对接收墙钟间隔中位数0.188秒，单worker处理墙钟中位数0.295秒/P95 0.527秒。输入到达快于处理能力，主要丢失发生在已配对后的等待队列。前端累计墙钟58.70秒、后端28.89秒，前端是最大的已记录处理阶段。墙钟包含调度等待，这些记录不能独立证明CPU热点、网络绝对无丢失，或nice19是唯一原因。

离线严格复现前19个已处理曝光至2.5秒，位置、完整C、前端及诊断均精确一致；下一曝光2.6秒缺少满足原30ms约束的真实IMU括号，因此在此停止，没有放宽插值或补数据。2.5秒上下文的一次CPU采样中，间距筛选占前端约23%。另取35.1/35.3/35.6/35.7秒四个真实移动输入独立建立局部地图，用于筛选等价性和小样本耗时比较；它不是原在线运动轨迹的完整复现。

最终只改前端`_spaced`：cKDTree以4像素轴向包围盒查询潜在邻居，最终仍用原数据类型、原贪心顺序及严格平方欧氏距离判断；异常类型和非有限数回退原扫描。没有减少特征、观测或改变跟踪/质量门。首个Python字典分桶候选虽数值等价，但耗时不稳定，已弃用。KD版本四帧局部处理墙钟0.373817→0.319685秒（约14.48%），其中筛选0.075688→0.025940秒；最终索引边界检查版重新确认位置、完整C和前端全部一致。这是局部样本收益，不是在线加速承诺。

运行配置增加显式可选`shadow_nice=5`，本轮旁路与Gazebo同为nice5、仍共享CPU2–5；默认定位仍为启动CPU0,1/nice0，总6CPU、6GiB、1GiB共享内存保持。helper普通调用默认仍为nice19，生产默认没有改。该配置与筛选优化一起验证，不能分别归因效果。

最终前端/跟踪22项、运行资源11项，共 **33项针对性检查通过**；最终前端SHA `5e7f0362b746f5fdd9d9b1ede88949abbc4253d0f88ee65416f0ef77f00f134f`，runtime SHA `9fc7c20fcf3c9f6c2c3a276e4d41f67c8c00aa6317dbd7efcd619e24cb3cd669`。核心、6关键帧/96观测/24共享门、100ms输出计划、500ms持久关键帧、0.3仿真秒/0.5墙秒参考门全部保持。

## 唯一新 V2 验证

`experiment-700e5c41a749`，原生无标柱V2、起点`[0,0,0]`、legacy/nogui，原默认RGBD控制。固定6次云台动作、等待、导航`[1,0]`、请求转向`π/4`、等待，10项动作全部完成。独立真值确认平移动作净XY 0.771730米、路径0.774699米；转向动作真实净转55.6319°、漂移45.886毫米。这里确认运动覆盖，不能把45°请求视为角度跟踪通过，本轮未修控制器。

完整评价窗为首个真实配对1.104至终态49.221，共48.117仿真秒，包含初始化及停车观察。所有482个100ms计划保留，没有筛掉迟到、初始化失败或缺曝光。

| 指标 | 上轮 | 本轮 |
|---|---:|---:|
| 精确配对 / 被覆盖 / 被拒绝 | 525 / 195 / 1 | 481 / 117 / 3 |
| 配对曝光丢弃比例（覆盖+拒绝） | 37.33% | 24.95% |
| 输出 / 全部计划 | 327/527（62.05%） | 361/482（74.90%） |
| 端到端墙秒P95 / 最大 | 0.609 / 0.836 | 0.914 / 1.285 |
| 输出交付年龄P95 / 最大，仿真秒 | 0.397 / 0.500 | 0.320 / 0.392 |
| 时间新鲜覆盖 / 最长不新鲜，仿真秒 | 29.19% / 2.272 | 43.04% / 1.050 |
| 同时满足仿真与墙钟参考门 / 全部计划 | 239/527 | 235/482 |
| 移动输出 / 计划，条件误差P95 | 37/69，6.338mm | 48/68，8.559mm |
| 转向输出 / 计划，条件误差P95 | 28/54，6.261mm | 38/56，8.645mm |
| 全输出条件XY误差P95 / 最大 | 6.304 / 6.703mm | 8.573 / 9.254mm |

本轮481配对=361处理+117覆盖+3就绪拒绝，无剩余未配对。两个初始化校准拒绝进入计划，一个1.2秒曝光未归属计划：原100ms计划锚定1.104秒，而后续原生曝光为1.2/1.3秒；同一计划间隔的曝光只计一次计划，原始曝光账另保留，不混用两个分母。482计划=361输出+117覆盖+2校准等待超时+2关闭缺曝光。所有输出在终态窗内交付；这不等于没有迟到，37个成功输出交付年龄超过0.3仿真秒，只有235个计划同时满足仿真及墙钟参考门。

成功输出之间内部最大交付间隔0.245仿真秒；**包含初始化和终止边界的完整窗最长无交付为0.976仿真秒**，不能拿内部间隔冒充全窗最大值。初始化首个结果交付较晚，最长不新鲜仍为1.050秒。

本轮配对墙钟接收间隔中位数0.241秒/P95 0.451秒；对应曝光时间跨度/墙钟接收跨度之比由0.520降至0.378。仿真数据在墙钟中到得更慢，同时worker处理墙钟中位数0.311秒/P95 0.644秒、排队P95 0.311秒，仍不足以跟上输入。**仿真时间中的密度改善伴随更慢的到达节奏，不能解释为实时吞吐已提升。** 两轮实际轨迹、初始化、共享负载及调度不同，不能从一轮联合改变中定量拆分前端优化和优先级贡献；墙钟变慢的单一调度原因尚未被隔离证明。

移动6.751仿真秒、转向5.530仿真秒均由真值独立分类。动态误差只覆盖成功的48个移动/38个转向样本（分类可重叠），纯转向仅5/8，不据此宣称全程精度通过。50个真实同stamp基线共同样本新/旧误差P95为8.571/2.198毫米，也没有证明比默认更准。条件模型σ P95 4.937毫米是模型量；坐标只用预先声明的初始IMU偏航、零安装偏置及首曝光平移原点，没有拟合轨迹旋转/尺度。

## 默认健康、闭合与证据

就绪屏障9.329秒后502次API、1509条已采集源质量状态均健康，fault counter保持0；计算前/后观测超时累计均为0。启动前另有初始化、stale、unsynchronized和recovering，生命周期普通stale计数1，不能写全程无任何初始化异常。

driver/shadow/observer/bag均exit0且未强制，所属世界正常停止、子进程0，含清理144.680墙秒。27个受保护源码和本轮候选首末哈希一致。工作台服务保留，世界已停；没有第二次世界、默认替换或回访修改。

- [修前供需证据](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-throughput-v2-20261022/cause-before.json)、[修后供需与墙钟比较](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-throughput-v2-20261022/throughput-after.json)。
- [最终等价检查](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-throughput-v2-20261022/comparison/final-equivalence-01.json)、[前端22项](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-throughput-v2-20261022/comparison/frontend-validation-02.json)、[运行11项](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-throughput-v2-20261022/tests/runtime-tests-01.json)。
- [全部计划与评价](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-throughput-v2-20261022/evaluation/summary.json)、[默认健康](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-throughput-v2-20261022/default-health-summary.json)、[实际动作真值](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-throughput-v2-20261022/physical-actions.json)。
- [87份副本哈希](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-throughput-v2-20261022/integrity.json)、[22份原始文件哈希](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-throughput-v2-20261022/raw-files.json)、[详细尝试与取舍日志第18节](D:/Documents/Projects/Repositories/MoonCar-mapping/docs/work_logs/2026-10-22-rgbd-window-localization.md)。大bag与上下文保留原位。

本次结果是局部优化有效、线上实时目标仍未完成；没有以降低仿真推进速度、删失败分母或低条件协方差宣称完整修好。
