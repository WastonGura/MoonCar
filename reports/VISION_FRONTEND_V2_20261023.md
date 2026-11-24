# 视觉图像块采样优化与单轮V2验证

日期：2026-10-23，Asia/Tokyo。

**本轮优化有效：同录像视觉处理P95下降32.18%，整个处理流程累计墙时下降23.60%；单轮V2端到端P95由0.606降至0.377秒，达到原0.5秒参考线。** 配对曝光丢弃率15.42%→3.32%，计划输出率84.37%→96.49%。仍有少量丢弃和墙钟迟到，不能宣称完全连续或全部实时验收通过。只优化仿真视觉定位旁路，默认定位、回访、模型和资源配置均未改；没有涉及实车。

## 一个热点、一个等价修改

仅对已有4个实际运动曝光35.1/35.3/35.6/35.7秒建立局部上下文，在35.6秒视觉process上做一次cProfile，不提取新bag或启动世界。插桩前端58.052ms，其中3次双线性图像块采样累计26.770ms，为最大记录热点；KLT两次6.625ms、间距筛选5.578ms。这里只说明这个局部样本的原因，插桩绝对时间及占比不是在线通用结论，也不能相加交叠累计项。

原采样对每个特征点周围21×21共441个整数偏移站点分别求floor和小数权重。标准LK中心为float32、灰度图为uint8，且中心已通过12像素内部边界检查；整数偏移不改变这些中心的小数部分。因此新增`_sample_patch`，每个中心只算一次整数坐标和float64小数权重，再广播到原441个位置。四邻像素、四项乘加顺序及21×21范围保持。非标准类型、非有限值或边界输入调用原`_sample`，其实现逐字未改。

仅替换`_track`内target/before/after三处采样调用，没有增加第二个捷径；ORB、KLT、深度/协方差、光度改善判定、回环距离、几何与空间门、特征身份和数量均未改。26/26前端与跟踪检查通过（原22+4新增），0失败/错误/跳过，1.108墙秒。新增检查覆盖采样逐位一致、改善/未精化保留判定、边界及异常回退。

新frontend SHA `744cc0b8b44483d6678de9af7ded8a5717adecd3486d58e7ad0a0a567148309d`，原`5e7f0362b746f5fdd9d9b1ede88949abbc4253d0f88ee65416f0ef77f00f134f`完整封存。当前缓存后端d20、online37cd、节点b826及其原39项适配/缓存检查所绑定源码保持不变，本轮不重复运行这些未改模块的专项。

## 同录像离线对照

复用已冻结126个真实SensorFrame，28.0–40.5秒，sensor SHA `4496c36889265ec19a053165f806cfb52eaae59eb76f6f91195332d9447af888`；计划表SHA `4301aa1cb70d25ca86a078eb3d3788b3116669e33ad3e977801dc084507c1290`。旧、新前端分别加载，均启用同一cached_query后端，独立从28.0局部初始化，不复现原在线完整前缀。评分31.4–40.5共92计划，73query/19KF，仍完整运行此前预热曝光。

126帧的全部特征字段、IDs/像素/三维点/每点完整C、实际选点、tracking/stats/age、关键帧身份、接受状态以及位置/完整输出C/数值诊断逐位一致；输入指纹保持。只排除opaque token及实测时间。一次warm各臂后AB/BA/AB三组，每次全新processor/cache；CPU0–5/nice0、OpenCV2、BLAS/OMP1。计算时不运行指纹比较，所有前端、关键帧缓存和后端成本都计入原阶段及wholeprocess。

| 组 | 视觉处理P95，旧→新 | 视觉处理总墙秒，旧→新 | wholeprocess总墙秒，旧→新 |
|---|---:|---:|---:|
| 1 | 79.363→53.079ms | 6.23127→4.19617 | 8.55395→6.53492 |
| 2 | 82.730→58.275ms | 6.35487→4.32723 | 8.71963→6.64020 |
| 3 | 83.168→56.407ms | 6.32613→4.34435 | 8.73596→6.71297 |

三组配对比例中位：前端P95下降**32.1769%**、前端总墙时下降**31.9068%**、wholeprocess总墙时下降**23.6034%**；后端总墙时比值0.986463。预设前端P95/总时间各降≥15%、wholeprocess总时间下降、后端总时间不退超5%的四门全部通过。后端源码没改，这个比值只支持未转移开销，不宣称后端算法也得到优化。

初次预检comparison-01在28.1秒把嵌套诊断`cache_build_wall_s`当作数值差异，立即停下，未执行任何性能组。保留原helper和首反例，仅从等价比较中排除该实测时间；全部构建成本仍计时。comparison-02是唯一有效性能比较，107.080墙秒/108.391进程CPU秒；算法、输入和验收门未因预检修正改变，没有为调优再跑性能轮次。

## 唯一V2验证

`experiment-cd2f7b92962f`，原生无标柱V2、legacy/nogui、起点0，显式缓存旁路。沿用6次云台→wait5→导航[1,0]→请求转向π/4→wait5，10动作全部完成。独立真值确认平移净XY0.784763米、路径0.787746米；转向真实57.251°、漂移47.918毫米，不能把45°请求当精确控制通过。

资源仍为6CPU/6GiB/1GiB共享内存，默认定位CPU0,1/nice0，Gazebo/旁路共享CPU2–5/nice5，BLAS/OMP1、OpenCV2；健康守卫、0.3仿真秒交付年龄和0.5墙秒端到端参考门保持。评价窗首实际pair1.103→终态52.315，共51.212sim秒，包含启动/停车边界；估计/计时封存后独立真值评分，不拟合轨迹。

| 指标 | 上轮缓存旁路 | 本轮视觉优化旁路 |
|---|---:|---:|
| 配对曝光 / 覆盖 / 拒绝 | 467 / 70 / 2 | 512 / 16 / 1 |
| 配对丢弃率 | 15.42% | **3.32%** |
| 输出 / 全部计划 | 394/467（84.37%） | **495/513（96.49%）** |
| 端到端墙秒P95 / 最大 | 0.606 / 0.929 | **0.377 / 0.607** |
| 交付年龄P95 / 最大，sim秒 | 0.297 / 0.413 | **0.253 / 0.300** |
| 双时效参考门通过 / 全部计划 | 326/467（69.81%） | **492/513（95.91%）** |
| 时间新鲜覆盖 / 最长不新鲜，sim秒 | 63.08% / 0.396 | **89.41% / 0.352** |
| 完整窗最长无交付 / 内部相邻最大，sim秒 | 0.311 / 0.270 | **0.352 / 0.229** |

仍有3个成功输出超过0.5墙秒；495输出虽全满足0.3sim年龄，也不代表每个时刻都有新鲜结果。完整空窗0.352为首pair1.103到首交付1.455的等待，较上轮略增；内部最大0.229另列，不拿较小值代替全窗。时间fresh为89.41%，非100%。

512pairs=495processed+16replace+1reject；513plans=495outputs+15replace+1TF就绪超时+2关闭缺曝光，账闭合，无unfinished或missing records。1.2秒被覆盖曝光没有计划编号，所以原始覆盖16次与计划覆盖失败15次不同；配对丢弃率17/512与计划无输出18/513不能混用。实际393次缓存query、102次KF及构建都成功输出，没有本轮窗口求解拒绝。

pair到达sim/wall跨度比0.395→0.524，墙钟到达间隔中位0.240→0.188秒；处理中位0.268→0.158秒，排队中位0.061→0.0027秒。改善没有伴随仿真到达节奏更慢。在线前端实际attempt墙时P95为164.772ms，普通query后端54.522ms，KF含cache204.647ms；这些含调度等待，不是CPU热点或可相加的端到端分位数。两轮轨迹、初始化和调度仍不同，不能用在线差值给单个优化算严格因果加速比；离线固定输入对照提供该优化的直接证据。

## 精度、默认健康与收尾

495成功输出全部可评分，XY误差P95 **4.838mm**、最大5.786mm；移动64/69计划，成功样本P95 **4.853mm**；转向52/55，P95 **5.190mm**；纯转向8/8，P95 **4.731mm**。条件σ P95为4.900mm，不是已校准全局后验。38个真实同曝光共同样本中，新/默认误差P95为4.838/2.961mm，没有证明比默认更准。缺输出保留在计划分母，也不把不同轮次误差变化当作本次等价计算带来的精度改善。

就绪屏障12.037秒后371次API、1196条源质量均健康，全局fault counter始终1。启动就绪前已有故障计数1及initializing/stale/recovering；普通stale累计1、屏障窗增量0，计算前/后观测超时累计0。`action_fault_counter`的0→1是动作激活时复制已有全局计数的基准快照，不是新增健康故障。没有在本轮扩大排查启动问题。

含收尾111.382墙秒，driver/shadow/observer/bag均exit0且非强制，owned世界正常停止、子进程0；工作台后端保留。31个保护源及候选首末一致，99份副本与15份原位文件hash封存；大bag、原始samples及复用录像留原位。仅完成一个热点优化、一次有效离线性能对比和一轮V2，未继续调参、切默认、改回访或涉及实车。

- [离线结果与预设门](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/vision-frontend-v2-20261023/comparison-02/summary.json)、[126帧逐位一致](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/vision-frontend-v2-20261023/comparison-02/equivalence.json)、[三组计时](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/vision-frontend-v2-20261023/comparison-02/timing.json)。
- [一次局部热点记录](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/vision-frontend-v2-20261023/frontend-profile-before.json)、[26项专项](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/vision-frontend-v2-20261023/frontend-tests-01.json)、[预检误报及保留证据](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/vision-frontend-v2-20261023/comparison-01/first-mismatch.json)。
- [全部在线计划及评价](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/vision-frontend-v2-20261023/evaluation/summary.json)、[供需和墙时](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/vision-frontend-v2-20261023/throughput-after.json)、[实际运动](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/vision-frontend-v2-20261023/physical-actions.json)、[默认健康](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/vision-frontend-v2-20261023/default-health-summary.json)。
- [源码与副本hash](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/vision-frontend-v2-20261023/integrity.json)、[原位文件hash](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/vision-frontend-v2-20261023/raw-files.json)、[唯一世界结果及退出](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/vision-frontend-v2-20261023/result.json)。
