# 关键帧缓存构建优化与同录像离线对照

日期：2026-10-23，Asia/Tokyo。

**本次缓存构建优化通过：构建墙钟P95减少37.40%，含关键帧的全计划后端P95减少17.05%；普通query基本不变，没有将开销转移到下一帧。** 126个计划的位置、完整协方差及状态与原求解器数值等价，12项专项检查全部通过。结论只针对缓存构建，整体方案尚未完成全部性能验收；未接工作台、未开新V2、未改默认定位或回访。

## 原因与最小修改

旧 `freeze_query_cache()` 先递归深拷贝整个estimator，包括全部已缓存因子和与此次冻结无关的解状态；随后又为返回缓存复制测量数组和历史残差因子，存在重复复制。

在同一已有满6关键帧快照28.5–31.0秒上，仅对旧版freeze做了一次cProfile：deepcopy递归19,181次，是最大的累计阶段。该次插桩总墙钟约108.6ms、deepcopy累计58.6ms；插桩显著增加Python调用成本，不能把这些绝对时间当作正常freeze或在线耗时。正式对比不带profile。

修改仅为：用隔离的浅副本，并独立复制`_frames`和`_frame_terms`两个可能被装配路径使用/填充的容器；本次装配只读已有因子和先验，不复制无关的`_solution`。返回缓存中的H、eta、均值、测量及残差数组仍独立持有且只读，没有将源数据引用留给后续查询，也没有把构建延迟到首个query。调用期间仍按原estimator合同串行提交/修改。

求解数学、共享地标相关性、3°门、C下限、版本规则和诊断均未改。旧10项专项保留，新增2项：禁用deepcopy并让隔离副本补算缺失compiled entry、确认原缓存不变；freeze后将源先验、测量、编译J/c/W及解写入NaN/清空，确认已冻结缓存的位置/完整C/残差仍不变。最终 **12/12通过**，0失败/错误/跳过。新源码SHA `2b15f25f5538eb9c463b833c002376486fc4ce28613a0c1d0984d535e199aa3e`，旧版 `3eddbabdd8f9d510cd03f345b508f521b4d73fc813b29a35e1f660ab3fa89f64` 完整封存。

## 固定输入与比较口径

直接复用上轮已封存的126帧Frame序列，SHA `4301aa1cb70d25ca86a078eb3d3788b3116669e33ad3e977801dc084507c1290`。来源仍为700e录像28.0–40.5秒，包括原在线未处理帧；没有重新提取、重跑前端或改变身份/选点/关键帧时间线。评分仍为31.4–40.5秒的92计划：73query、19关键帧，评分前真实6KF预热保持。

三臂为原始联合求解、修改前缓存版、修改后缓存版。旧源码独立加载；每臂每轮均重新创建窗口和缓存，profile快照不进入正式计时。逐帧即时核对原参考结果及输入/持久窗口不变后，一次预热、三组轮转ABC/BCA/CAB。CPU0–5/nice0、BLAS/OMP各1线程，期间无世界或其它计算agent。所有freeze、新选择列和query成本均计入，不把指纹校验或profile开销算作求解耗时。

开跑前固定以三组配对比值的中位数验收：freeze P95减少至少30%；全计划后端P95下降；query P95及后端总耗时各不退超过5%。原始联合求解的30% query/20%总耗时门另列，不替换或放宽。

## 实际结果

| 组 | 缓存构建P95，旧→新 | 全计划后端P95，旧→新 | 后端总墙秒，旧→新 |
|---|---:|---:|---:|
| 1 | 40.553→24.731ms | 72.465→60.107ms | 2.52644→2.25873 |
| 2 | 40.288→27.897ms | 72.314→62.491ms | 2.51098→2.29369 |
| 3 | 39.038→24.437ms | 73.473→59.176ms | 2.52370→2.25690 |

对修改前缓存版，三组配对中位结果为：freeze P95下降 **37.4009%**；全计划后端P95下降 **17.0538%**；query P95比值 **1.001051**（约+0.105%，基本不变）；后端总耗时下降 **10.5718%**。四项本次验收均通过。关键帧含freeze的P95由约77.8–80.4ms降至62.8–66.5ms，仍有真实构建开销，没有宣称消除全部尾部耗时。

上述降幅都来自本轮配对旧/新，未拿上轮历史时间算收益。对原始联合求解，本轮新缓存版query P95下降中位 **29.3643%**，仍未达到原30%门；后端总墙钟下降 **30.7572%**，达到20%门。因此可以说缓存构建优化通过，不能说整个方案性能验收通过或在线实时性问题已解决。

126个计划三臂状态一致，新/旧缓存版相对原联合求解的位置最大差均1.92686e-13m、完整C最大差均8.10441e-18m²，满足原容差；窗口及输入指纹不变。估计封存后才读取真值。三臂92/92评分输出；新/旧缓存版误差统计相同，XY误差P95约3.793635mm，条件σ评分起终4.469122→4.745214mm。这里没有精度或不确定性改善，只是保持原模型结果、减少计算。局部初始化和离线92/92不代表在线零丢帧。

最大缓存数组/稀疏存储估计仍为4,998,936字节，不是RSS，也没有测量或声称常驻内存下降。普通帧及前端仍是原实现；未据这些离线墙钟结果推断ROS/渲染负载下的实时交付。

## 封档

唯一正式比较正常完成，87.949墙秒/87.457进程CPU秒。31个保护源（含默认/回访/原core/frontend/在线及上轮比较器）原样，比较期间8个源码及helper哈希保持一致，所有计算已停止。没有启动工作台服务、ROS节点或世界，也没有追加调优或第二轮比较。

- [完整摘要与各验收门](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-cache-build-offline-20261023/comparison-01/summary.json)、[三臂逐组计时](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-cache-build-offline-20261023/comparison-01/timing.json)、[等价检查](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-cache-build-offline-20261023/comparison-01/equivalence.json)。
- [重复复制直接证据](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-cache-build-offline-20261023/freeze-cause-before.json)、[12项检查](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-cache-build-offline-20261023/cache-tests-01.json)、[封存与真值隔离](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-cache-build-offline-20261023/comparison-01/estimator-seal.json)。
- [29份副本哈希](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-cache-build-offline-20261023/integrity.json)、[复用输入原位哈希](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-cache-build-offline-20261023/raw-files.json)。原录像与输入保持原位，旧版源码一并归档。
