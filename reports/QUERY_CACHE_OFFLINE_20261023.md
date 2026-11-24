# 普通帧缓存求解：离线原型与固定录像对照

日期：2026-10-23，Asia/Tokyo。

**原型已完成，数值等价通过，存在计算收益；未达到预设的全部性能门槛，结论为“正确但收益不足”。** 普通query墙钟P95下降中位数29.7447%（门槛30%），后端总墙钟下降中位数23.017%（门槛20%，包含缓存构建）。关键帧构建缓存增加了单次尾部耗时。本轮按计划停止，没有追加优化、重跑或在线接入。

## 实现与不确定性处理

新增独立 `query_cache.py`，保留原 `WindowEstimator.query_frame()` 与 `add_frame()` 原字节作为参考。`freeze_query_cache(estimator, version)`拥有已提交窗口的信息矩阵、分解和均值；`query_cached(frame, cache, expected_version)`仅用当前曝光求当前位置。每次成功关键帧换缓存版本，失败不提交；版本不符明确失败，没有偷偷调用完整参考求解器的回退。

普通帧取实际共享地标的完整联合边缘协方差，包括地标之间的交叉块，加上当前观测噪声；对当前位置、姿态小修正和当前公共测量误差求9维系统。仅当前曝光出现的新地标解析消去，不赋予虚假的固定点先验。历史状态均值回代检查原3°门，满窗时不检查参考实现已临时消去的旧角状态。原输出协方差下限只应用一次，没有重设C或旧位姿换时间戳。

后验快照只读；成功查询可以记忆按地标ID选择的信息逆矩阵列，失败不污染memo。历史残差的固定因子在freeze时保存，避免诊断重新构造旧观测。原全窗LU pivot与新9维条件性指标分别记录，**没有声称二者诊断逐值相同或所有极端输入的接受边界已获一般性证明**。此模块没有进入在线或默认调用路径。

新增10项专项检查全部通过，覆盖未满/已满窗口、地标更替及完整交叉块、新地标不增加虚假信息、动态云台杆臂、版本/所有权、历史角度回代、异常回滚、重复query后再commit，以及不调用原全窗求解器。测试及正式比较均保持core/frontend/data/online/node原样。

## 固定数据与公平性

仅使用已闭合实验 `experiment-700e5c41a749` 的录像28.0–40.5秒。前28.0–31.346秒预热至真实6关键帧；评分从下一个曝光31.4至40.5秒，覆盖原录像平移31.346–36.554秒、转向37.686–39.904秒。没有创建新世界或执行新运动。

一次严格提取126/126组实际RGBD配对，零缺失，包含原在线未处理的38帧；保留原30ms IMU括号、曝光TF、标定和未知深度掩码。源bag只读且SHA/大小/mtime一致。共同前端仅运行一次，冻结同一特征身份、选点、Frame和100ms计划/500ms关键帧时间线；两臂各自初始化窗口。每次replay核对冻结输入指纹，候选逐帧立即核对参考结果，出现差异即保留反例并停止。

这次是局部初始化的离线A/B，不复现原在线全部历史。所有126个计划状态一致，最大位置差 **1.92686e-13米**，完整3×3位置协方差最大差 **8.10441e-18平方米**，远低于预设位置`atol=2e-8/rtol=1e-7`、C`atol=2e-11/rtol=1e-6`。查询及freeze未改变持久窗口。评分92个计划均输出，分别73个普通query、19个关键帧；离线92/92不代表在线零丢帧。

计时为一次不计时预热、三组串行交替A/B，每次从新窗口重建缓存，顺序A/B、B/A、A/B。容器6CPU、CPU0–5/nice0，BLAS/OMP各1线程、OpenCV2线程，无仿真和其他计算agent并行。普通query包含缓存新列求解；每次freeze全部计入关键帧后端成本。解码/读取、前端和校验开销单列。

## 实际性能与误差

| 配对计时组 | 原query P95 | 缓存query P95 | query P95降幅 | 后端总墙秒，原→缓存 |
|---|---:|---:|---:|---:|
| 1 | 38.369ms | 26.956ms | 29.7447% | 3.11754→2.39998 |
| 2 | 42.943ms | 27.574ms | 35.7898% | 3.39567→2.52878 |
| 3 | 38.929ms | 27.948ms | 28.2068% | 3.23513→2.58714 |

验收采用三组相对降幅的中位数：query **29.7447%<30%**，后端总耗时 **23.01695%≥20%**。不能将29.7447%四舍五入成30%后称全部通过。

缓存构建每组额外约0.600–0.656墙秒。包含关键帧和缓存构建的**全部计划后端P95**，由38.369/43.568/39.088ms升至71.373/71.282/72.896ms：总耗时下降，同时关键帧尾部变重。缓存最大数组/稀疏存储估计4,998,936字节（约4.77MiB），不等于进程RSS。

共同前端评分92帧总墙钟6.56783秒、P95 82.238ms。将这一次共同前端与各组独立后端时间作组件求和，总耗时降幅中位数约7.41%；**这只是组件算术和，不能作为实测端到端延迟，也不能相加两个P95推断总P95**。当前没有证明仿真与ROS负载下的实时性或连续性通过。

估计结果及计时记录封存后才读取真值；使用原声明的初始IMU偏航0、偏置0，以及局部首关键帧的平移原点，没有拟合旋转或尺度。

| 评分段 | 两臂输出/计划 | 原 / 缓存XY误差P95 |
|---|---:|---:|
| 全评分段 | 92/92 | 3.793635 / 3.793635mm |
| 原录像平移动作区间 | 52/52 | 3.013778 / 3.013778mm |
| 原录像转向动作区间 | 23/23 | 3.673331 / 3.673331mm |

全段最大误差均约3.939893mm。两臂条件XY σ的评分起点/终点均为4.469122→4.745214mm，P95均约4.781416mm；本次保持同一模型的不确定性，没有额外压低C。该误差不能与原在线约8.6mm直接解释为精度提升：初始化、输入完整性和运行方式不同。短片段条件C也不是已标定的长期导航后验。

## 交付与停止

本次新增独立缓存求解模块、严格片段提取器、固定A/B脚本及测试；默认定位、回访、原前端/核心及在线调度共29个冻结源保持一致。比较61.802墙秒/58.197进程CPU秒完成，所有计算进程已结束；容器仅保留sleep，没有启动工作台或仿真服务。

- [完整结论](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-cache-offline-20261023/comparison-01/summary.json)、[等价检查](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-cache-offline-20261023/comparison-01/equivalence.json)、[三组计时](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-cache-offline-20261023/comparison-01/timing.json)。
- [全部输入账本](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-cache-offline-20261023/input-01/ledger.jsonl)、[封存与真值隔离](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-cache-offline-20261023/comparison-01/estimator-seal.json)、[10项专项检查](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-cache-offline-20261023/cache-tests-01.json)。
- [23份副本哈希](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-cache-offline-20261023/integrity.json)、[原始大文件清单](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-cache-offline-20261023/raw-files.json)。输入pickle、冻结Frame和原bag保留原位。

按事先约定的停止条件，本轮不再微调到刚好过门槛，不实现后台并行，也不据此接入工作台或开启V2。
