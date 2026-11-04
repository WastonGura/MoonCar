# 剩余覆盖与迟到：时间线、小范围去重计算及单轮V2结果

日期：2026-10-23，Asia/Tokyo。

**已消除关键帧缓存的重复装配和分解，离线正确性与性能门通过；本轮在线丢帧和迟到目标未达成。** V2平移/转向完成，但配对丢弃率3.32%→4.76%，端到端P95为0.377→0.415秒，成功输出墙钟迟到3→7次。P95仍低于原0.5秒参考门，整体连续性没有改善。没有据此追加调参、第二轮实验、默认替换或回访修改。

## 先核对上轮19个事件

只读上轮cd2f闭合记录，对16次原始曝光覆盖与3次成功输出迟到逐个按真实monotonic接收、开始、完成及替换时间关联。覆盖等待的前序工作为启动TF就绪1次、普通query10次、关键帧5次，不能统一归因为关键帧。16.8/36.0秒两个替换事件发生在前worker记录完成后1.801/1.556ms的交接间隙；其等待期与前worker重叠，不把替换瞬间说成仍在矩阵求解。

| 上轮迟到曝光 | 队列等待 | 准备 | 视觉处理 | 后端 | 端到端 |
|---|---:|---:|---:|---:|---:|
| 31.0 query | 236.2ms | 16.1ms | 225.5ms | 42.7ms | 527.3ms |
| 36.8 query | 241.9ms | 27.6ms | 274.7ms | 11.8ms | 562.4ms |
| 36.9 KF | 300.8ms | 10.5ms | 85.8ms | 199.6ms | 606.8ms |

直接阻塞前驱分别为30.9KF、36.7query、36.8query；后两者更早的积压链包含36.4KF。共同机制是排队叠加处理尾部，准备/TF等待不是三者共同原因。记录没有阶段内部的起止点，墙时也含调度，未把它当成新的CPU热点采样。

代码确认关键帧`_solve`已装配H、分解LU并求均值，成功后`freeze_query_cache`对相同窗口再执行一次。去掉此重复工作是本轮唯一修复方向，只能预期缓解部分上游积压，不能预先承诺覆盖全部事件。

## 最小显式交接与55项检查

独立原型新增：

```python
answer, posterior = estimator.add_frame_with_posterior(frame)
cache = freeze_query_cache(estimator, version, posterior=posterior)
```

只在`cached_query=True`的关键帧路径使用。全部原求解、角度、协方差、pivot和有限值检查成功后，返回本次H/eta/完整均值/LU的一次性临时产物；冻结时校验owner、提交时间/计数、keys、gauge、配置及源内容指纹，再转交LU并独立持有缓存数组。过期、修改、错owner或重复消费均拒绝，不隐式重建或回退。LU没有存入估计器持久状态，原`add_frame/query_frame`的返回和pickle状态保持；不传posterior的freeze仍支持原构建路径。

核心数学、普通query、完整相关协方差、边界检查、前端744、节点b826、时间门、周期、资源及默认生产定位未改。handoff指纹与所有复制成本均计入实际后端/缓存构建墙时。55/55通过（core12/cache21/adapter22），0失败/错误/跳过，2.251墙秒，覆盖满窗Schur、特征更替、原路径状态、零重复assemble/LU、失败回滚、所有权、修改/过期/重复消费和缓存隔离。

最终core SHA `133179c8d6f0575db23669329c94f9c5466cd0dca538a8bbad2cbef7cceb648e`，query_cache `d8c823f419c77d411d6fa3e89da95b807bddd3a9e5894e92c30019d1bbaa745c`，online `19db3d738b7cc9abbe96346d21dcfcd11c939d74bcaad339bded7f080f1c68ce`；原件完整封存。

## 固定录像后端对照

复用SHA4301aa1c…的126个冻结Frame和计划，28.0–40.5秒，评分92计划=73query+19KF，无提取/前端重跑/真值。before使用原d20缓存构建，after显式交接；两臂共享新core的普通路径，另逐帧核对已封存旧联合解，防止共同回归。两臂位置、完整C、接受状态和全部数值诊断逐位相同（仅排实测构建时间），持久状态逐帧pickle相同，查询/冻结没有修改持久窗口。

一次warm各臂后AB/BA/AB三组，fresh estimator/cache，CPU0–5/nice0、BLAS/OMP1。全部KF提交、指纹、交接及缓存构建计入后端，状态验证在计时外。预设KF含cache P95降≥15%、cachebuild P95降≥30%、后端总墙时下降、普通queryP95不退超5%，四门通过。

| 组 | KF含cache P95，旧→新 | cachebuild P95，旧→新 | 后端总墙秒，旧→新 |
|---|---:|---:|---:|
| 1 | 76.255→56.867ms | 29.352→14.651ms | 2.13694→1.89001 |
| 2 | 118.373→52.908ms | 46.729→11.757ms | 3.25756→1.88750 |
| 3 | 75.563→62.222ms | 27.930→13.107ms | 2.23706→2.06173 |

三组配对中位：KF含cache P95下降25.4247%、构建P95下降53.0717%、后端总耗时下降11.5552%；普通query P95比值0.879095，仅作无成本转移检查，不称其算法也优化。第2组原版计时偏高仍完整保留，各组KF改善均达15%；没有挑组或重跑。最大缓存数组/稀疏存储两臂仍4,998,936字节，非RSS。正式63.038墙秒/62.349进程CPU秒，一次比较封闭。

## 唯一V2：整体目标未达成

`experiment-ffe3e6183c0c`，原生无标柱V2/legacy/nogui/start0，显式缓存旁路。原10动作（6云台、等待、导航[1,0]、转向请求π/4、等待）全完成；6CPU/6GiB/1GiB、default CPU0,1/nice0、Gazebo与旁路CPU2–5/nice5、线程数与健康守卫保持。评价窗首pair1.585→终态49.841=48.256sim秒，保留启动、动作间和结束边界。

| 指标 | 上轮视觉优化 | 本轮handoff |
|---|---:|---:|
| 配对 / 覆盖 / 拒绝 | 512 / 16 / 1 | 483 / 21 / 2 |
| 配对丢弃率 | 3.32% | **4.76%** |
| 输出 / 全计划 | 495/513（96.49%） | **460/483（95.24%）** |
| 端到端墙秒P95 / 最大 | 0.377 / 0.607 | **0.415 / 0.661** |
| 成功输出wall>0.5s | 3 | **7** |
| 成功输出sim年龄>0.3s | 0 | 2 |
| 双时效门 / 全计划 | 492/513（95.91%） | 453/483（93.79%） |
| 时间fresh覆盖 / 最长notfresh，sim秒 | 89.41% / 0.352 | 86.94% / 0.394 |
| 完整交付空窗 / 内部相邻最大，sim秒 | 0.352 / 0.229 | 0.646 / 0.646 |

483pair=460processed+21replace+2reject；483plan=460output+20replace+TF超时1+IMU括号超时1+关闭缺曝光1。1.6秒替换无计划编号，故两种覆盖分母相差1。无未终结queued或漏计划记录，96次KF/build和364次cached query均成功。

7次墙钟迟到为11.3/12.4/14.2/15.3/15.7/16.2/41.3秒；其中11.3、16.2也超仿真年龄。不能用458个simfresh减453个双fresh推成“只有5次wall迟到”，两种迟到存在交集。7次按冻结driver动作窗为云台4、动作间3，实际平移/转向窗未出现迟到，但全窗分母全部保留。

本轮另有2.9sim曝光等待精确IMU括号0.505247墙秒，期间覆盖3.0/3.1两次计划曝光；启动1.585sim等待精确相机TF0.504092秒，覆盖1.6秒无计划曝光。没有放宽原等待、插值或质量门，也没有扩大修复这次新记录。原始墙时不能确定新的CPU原因。

在线cachebuild P95为56.139ms（上轮86.638ms），但KF含cache后端P95为211.305ms（上轮204.647ms），实际前端attempt P95为188.934ms（上轮164.772ms），排队P95为166.318ms（上轮142.539ms）。pair到达sim/wall跨度比0.524→0.479。不同实际轨迹、初始化、准备等待和共享调度使这些在线差值不能作为单一因果证明；本轮既没有证明整体改善，也不能仅凭一轮断言去重计算导致退化。离线收益不代替在线验收。

## 误差、默认健康与收尾

独立真值确认move净XY0.777208m、路径0.779488m；turn55.506°并漂移46.361mm，仅证运动覆盖，不当作45°跟踪通过。460成功输出全可评分，XY误差P95为5.635mm、最大10.085mm；移动66/68，成功误差P95为5.574mm；转向50/51，P95为5.702mm；纯转向8/8，P95为4.008mm。条件σ P95为4.901mm，不是已校准全局后验；36个同stamp新/默认误差P95为5.637/3.797mm，没有证明比默认更准。坐标和封存后truth评价规则保持，不拟合轨迹。

屏障9.834之后392次API及1242条源质量健康，全局fault counter从就绪时1保持到结束。启动有initializing/stale/recovering、unsynchronized/from_future等记录，源计数均在屏障后无增量，计算前/后观测超时累计0。默认健康与旁路本次IMU等待不是同一指标。

含收尾116.808墙秒，driver/shadow/observer/bag均exit0非forced，owned世界停止、子进程0；后端保留。29保护源与候选首末同hash，96份副本/19份原位文件hash封存。没有第二个world、第二修复、默认替换、回访修改或实车工作。本轮仅保留已验证的减算量改动，**在线剩余丢帧与迟到仍未解决**。

- [上轮19事件精确时间线](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-tail-v2-20261023/cause-before.json)、[本轮7迟到与2等待事件](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-tail-v2-20261023/tail-after.json)、[55项检查](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-tail-v2-20261023/tail-tests-01.json)。
- [离线等价及门限](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-tail-v2-20261023/comparison-01/summary.json)、[三组计时](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-tail-v2-20261023/comparison-01/timing.json)、[在线全部计划评价](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-tail-v2-20261023/evaluation/summary.json)。
- [默认健康](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-tail-v2-20261023/default-health-summary.json)、[真实运动](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-tail-v2-20261023/physical-actions.json)、[源码及副本hash](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-tail-v2-20261023/integrity.json)、[原位文件hash](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-tail-v2-20261023/raw-files.json)。
