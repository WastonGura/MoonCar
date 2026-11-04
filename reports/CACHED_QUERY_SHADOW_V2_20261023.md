# 缓存查询在线旁路：单轮V2平移与转向验证

日期：2026-10-23，Asia/Tokyo。

**已完成最小在线接入和一轮V2验证。输出连续性、丢帧和墙钟延迟较上轮改善，但仍未达到连续实时输出要求，不能替换默认定位。** 配对丢弃率24.95%→15.42%，端到端墙钟P95为0.914→0.606秒，仍超过原0.5秒参考门。默认定位、回访和导航源码未改，本轮未追加实验或修复。

## 接入内容与检查

原旁路普通帧仍调用`WindowEstimator.query_frame`，因此本轮必须显式接入已离线验证的`query_cache`。仅新增`ShadowProcessor(..., cached_query=False)`和节点`--cached-query`开关；默认调用仍用原求解路径。没有把新方案设为工作台默认，也没有新增ROS位姿、TF、质量或控制发布。

启用后，成功关键帧递增版本并冻结新缓存，普通帧使用该版本的`query_cached`。到期关键帧尝试前禁用旧缓存；关键帧拒绝或缓存构建失败后不沿用旧缓存、不偷偷退回旧求解，普通查询失败不提交窗口、前端或派生逆列。构建耗时完整包含在`backend_s`中，`cache_build_s`只是其子项，不重复相加。

**39/39专项通过**：22项旁路适配、17项缓存检查，0失败/错误/跳过，2.586墙秒。新增合同涵盖默认禁用、版本替换和真实曝光时间戳、失败关键帧/特征不足清空缓存、构建失败后的恢复、构建计时，以及真实缓存查询失败时的事务性。`query_cache.py`保持离线版SHA `d20a9f5642c55d08c5ce35a49571dbca07a4e648234ae50fa93a2f1bcf094b54`；新online SHA `37cd5645d6803153ea0dbbc3a0b8173cb790462de234860edf73a6f4d22fc3ed`，节点SHA `b826b21655e7c4a4e66d6d2f292bd7f72a85b2fbbe1dc53400c0ecda7d36b2c6`。

模型、前端、6关键帧/96观测/24共享门、100ms输出计划、500ms持久关键帧、噪声和协方差规则均保持。没有为在线运行再调算法或门槛。

## 唯一实验与计时口径

`experiment-390e19d95767`：原生无标柱`survey_v2_native`、起点`[0,0,0]`、legacy/nogui，默认RGBD控制；复用原6次云台动作→等待5秒→导航`[1,0]`→转向请求`π/4`→等待5秒，10项动作全部完成。

资源沿用6CPU/6GiB/1GiB共享内存、BLAS/OMP各1线程；默认定位CPU0,1/nice0，Gazebo和旁路共享CPU2–5/nice5，OpenCV2线程。世界内默认健康丢失或fault counter变化即终止，不恢复或重试。与历史轮次的轨迹、初始化和共享调度并不完全相同，以下对照是观测结果，不能将差值全部归因于缓存。

评价窗从首个实际配对0.372到终态47.054，共46.682仿真秒，包含初始化和停车观察。全部467个100ms计划保留，未删除缺曝光、拒绝、迟到或初始化失败。原参考门仍为交付年龄≤0.3仿真秒、端到端≤0.5墙秒；条件模型σ的50mm参考值不代表已标定全局精度。估计/计时关闭并封存后才用独立真值评分；坐标仅用预先声明的初始IMU偏航、零安装偏置和首曝光平移原点，不拟合轨迹。

## 结果

| 指标 | 上轮700e旁路 | 本轮缓存旁路 |
|---|---:|---:|
| 配对曝光 / 被覆盖 / 被拒绝 | 481 / 117 / 3 | 467 / 70 / 2 |
| 配对丢弃比例 | 24.95% | **15.42%** |
| 输出 / 全部计划 | 361/482（74.90%） | **394/467（84.37%）** |
| 端到端墙秒P95 / 最大 | 0.914 / 1.285 | **0.606 / 0.929** |
| 交付年龄P95 / 最大，仿真秒 | 0.320 / 0.392 | 0.297 / 0.413 |
| 同时满足仿真和墙钟参考门 / 全部计划 | 235/482（48.76%） | **326/467（69.81%）** |
| 时间新鲜覆盖比例 / 最长不新鲜，仿真秒 | 43.04% / 1.050 | **63.08% / 0.396** |
| 含边界最长无交付，仿真秒 | 0.976 | **0.311** |
| 内部相邻交付最大间隔，仿真秒 | 0.245 | 0.270 |

全窗空窗改善，内部最大间隔则略增，两者分别保留。15个成功输出超过0.3仿真秒年龄，全部467计划只有326个同时满足两个时效参考门，故不能用P95年龄0.297秒宣称全程新鲜。配对曝光跨度/墙钟接收跨度比由0.378变为0.395，墙钟到达间隔中位0.240秒、worker处理墙钟中位0.268秒，仍有等待与覆盖；这些墙时含调度，不能当CPU热点采样，也没有启动额外排查。

账目闭合：467配对=395处理+70覆盖+2拒绝；395处理包含394个有计划输出及1个0.4秒无独立计划曝光，后者与0.372秒锚定的首计划落在同一间隔。467计划=394输出+70覆盖+1相机TF就绪超时+1关闭前未处理+1关闭缺曝光。没有未终结queued或遗漏计划记录。

本轮实际执行302次普通缓存查询和92次关键帧提交/构建，均成功输出；版本1–92连续，普通查询使用当时版本，没有legacy joint-query记录。剩余丢失主要发生在待处理曝光覆盖，而不是缓存求解拒绝。

| 实际处理阶段 | 数量 | 墙钟P95 | 墙钟累计 |
|---|---:|---:|---:|
| 普通缓存query后端 | 302 | 88.293ms | 11.7337s |
| 关键帧后端，含缓存 | 92 | 235.967ms | 13.9323s |
| 其中缓存构建子项 | 92 | 100.984ms | 4.9917s |
| 前端，所有已开始计划 | 395 | 307.165ms | 72.5056s |

缓存构建已计入关键帧和后端总成本，没有移到计时外。不能相加各阶段P95得到端到端P95，在线共享负载下的墙时也不能直接与离线独占CPU计时比较。

## 平移、转向与定位误差

独立原始真值确认：平移动作净XY位移0.778408米、路径0.780119米；转向动作净转54.419°并平移44.244毫米。请求45°只代表输入，不当作精确角度跟踪通过；本轮只核实运动覆盖，不修导航控制。

394个成功输出全部可评分，XY误差P95 **5.214mm**、最大7.115mm。移动56/68计划有输出，成功样本误差P95 **4.830mm**；转向38/45，P95 **4.887mm**；纯转向5/7，P95 **4.869mm**。缺输出仍保留在各自计划分母，误差统计不代表其定位质量。

条件模型σ P95为4.894mm，全部394输出小于50mm参考值；它不是已校准的全局后验。38个真实同曝光共同样本中，新方案/默认方案误差P95为5.208/2.964mm，没有证明新方案比默认更准。历史轮次误差、局部离线误差的变化也不能单独归因于这次等价计算优化。

## 默认健康、收尾及边界

就绪屏障7.549秒后，454次API采样和1380条已采集源质量均健康，fault counter保持0；计算前/后观测超时累计均0。初始化前另有initializing、stale和recovering，普通stale生命周期计数1、屏障窗内增量0，不表述为全生命周期没有异常。没有自主探索竞争。

含收尾132.842墙秒。driver、shadow、observer、bag均exit0且非强制，owned世界正常停止、所属子进程0；工作台后端保留。30个受保护源与候选源首末哈希一致，80份证据副本/12份原始文件哈希已封存，大bag与observer原始样本留原位。

本轮验证证明缓存方案已能在线运行并覆盖平移/转向，观察到可用性和延迟改善；**实时性及连续可用性仍未通过，维持旁路，不切默认。** 到此停止，无第二轮V2、追加调参或回访修改。

- [完整评价与全部计划](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/cached-query-shadow-v2-20261023/evaluation/summary.json)、[供需和阶段耗时](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/cached-query-shadow-v2-20261023/throughput-after.json)、[实际缓存生命周期](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/cached-query-shadow-v2-20261023/cache-lifecycle-summary.json)。
- [真实运动](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/cached-query-shadow-v2-20261023/physical-actions.json)、[默认健康](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/cached-query-shadow-v2-20261023/default-health-summary.json)、[39项检查](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/cached-query-shadow-v2-20261023/adapter-tests-01.json)。
- [唯一世界及退出结果](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/cached-query-shadow-v2-20261023/result.json)、[源码及80份副本哈希](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/cached-query-shadow-v2-20261023/integrity.json)、[原始文件原位哈希](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/cached-query-shadow-v2-20261023/raw-files.json)。
