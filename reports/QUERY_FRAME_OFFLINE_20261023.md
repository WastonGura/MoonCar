# 普通帧查询耗时优化与同录像离线对照

日期：2026-10-23，Asia/Tokyo。

**普通帧查询有明确收益，但未通过本轮全部性能门：相对修改前缓存版，普通帧总墙钟耗时下降26.22%，P95下降14.83%，未达到预设20%的P95改善要求。** 17项专项检查通过，126个计划的位置、完整协方差和状态保持数值等价。仅完成离线优化及同录像比较，未接工作台、未改默认定位或回访，未新开V2。

## 直接原因与修改

沿用上一轮缓存构建优化后的版本，先封存旧源码，再只检查同一冻结窗口中的31.6、31.7秒两个真实普通query。前者需新求255个标量后验逆列，后者仍需18列；后者是部分命中缓存，不是零未命中。cProfile中，逐观测校验累计约6.50/6.29ms，历史残差的小矩阵乘法及Python求和累计约4.17/4.46ms。首帧选择列LU求解约12.97ms，仍是较大成本。这些是带插桩的局部原因证据，绝对时间不能当正常query或在线耗时，各累计项也不能相加。

只修改独立`query_cache.py`的两处：

- 标准float64数组观测使用批量有限值、正深度、对称性及Cholesky校验，保留原公差。异常或非标准输入交回原逐项校验器，保留原首个报错顺序；返回点与协方差仍独立持有。姿态、杆臂和姿态协方差仍用原校验函数。
- 按每个历史帧和当前帧，批量执行残差的小矩阵乘法，保持原因子、block及观测顺序，继续计算RMS并执行非有限值拒绝；没有取消诊断或提前到关键帧计算。

`freeze_query_cache`及其共享辅助、缓存布局、选择列memo/未命中求解和`query_cached`入口未变。完整相关地标协方差、9维求解、噪声、3°门、C下限及失败不提交memo的规则未改。没有减少观测、换数据类型、改前端或提供旧求解回退。

原12项专项保留，新增标准批量校验与所有权、原对称容差及非标准输入、混合异常首错误顺序、残差累加顺序、非有限残差不提交memo等5项，最终**17/17通过**，0失败/错误/跳过。源码SHA `d20a9f5642c55d08c5ce35a49571dbca07a4e648234ae50fa93a2f1bcf094b54`；旧版`2b15f25f5538eb9c463b833c002376486fc4ce28613a0c1d0984d535e199aa3e`完整归档。

## 固定输入与验收

直接复用同一700e录像28.0–40.5秒已冻结的126个Frame与调用表，输入SHA `4301aa1cb70d25ca86a078eb3d3788b3116669e33ad3e977801dc084507c1290`。包含原在线未处理的实际配对，不重新提取或运行前端，不改变特征身份、选点、100ms普通计划、500ms关键帧计划。28.0秒局部原点，评分前已有真实6关键帧；评分31.4–40.5秒共92计划，含73普通query、19关键帧。

三臂为原始联合求解、修改前缓存版、修改后缓存版。旧模块独立加载，每次replay重新建立窗口和缓存，两个局部profile的状态不进入计时。逐帧数值/状态及输入、持久窗口指纹通过后，每臂一次预热，再做ABC/BCA/CAB三组轮转；CPU0–5/nice0，BLAS/OMP各1线程，无其它agent计算或世界负载。验证、首次选择列未命中、矩阵与结果处理均计入query；每次freeze计入关键帧和后端总成本。

开跑前锁定三组配对比值的中位数验收：相对修改前缓存版，query P95下降至少20%、query总耗时下降至少15%，关键帧含缓存P95及后端总耗时各不退超过5%。原始联合求解的30% query P95、20%后端总耗时门独立保留，不改变门槛。

## 本轮结果

| 组 | 普通query P95，旧→新 | 普通query总墙时，旧→新 | 关键帧含缓存P95，旧→新 | 全部后端总墙时，旧→新 |
|---|---:|---:|---:|---:|
| 1 | 27.954→23.074ms | 1.21040→0.87852s | 62.613→66.736ms | 2.26055→1.94937s |
| 2 | 27.717→23.607ms | 1.20220→0.89503s | 63.267→65.679ms | 2.25541→1.97989s |
| 3 | 27.231→23.635ms | 1.19499→0.88164s | 67.450→62.028ms | 2.27599→1.93377s |

| 相对修改前缓存版的配对中位结果 | 实测 | 预设要求 | 判定 |
|---|---:|---:|---|
| 普通query P95下降 | 14.8290% | ≥20% | 未通过 |
| 普通query总耗时下降 | 26.2218% | ≥15% | 通过 |
| 关键帧含缓存P95比值 | 1.038131（+3.8131%） | ≤1.05 | 通过 |
| 全部后端总耗时比值 | 0.862344（下降13.7656%） | ≤1.05 | 通过 |

三组query P95降幅分别17.46%、14.83%、13.21%，均未达到20%。不选最好的一组或四舍五入作达标。关键帧第1组增加6.59%，本轮预先约定的是三组配对中位数门，不代表每组均小于5%。全计划后端P95旧58.815/59.435/60.571ms、新59.369/59.576/60.039ms，配对中位比1.002366，基本未改善；freeze P95配对中位反而增加7.11%，代码及依赖未变，保留波动而不宣称构建获得新收益。最大缓存数组/稀疏存储估计两版均4,998,936字节，不是RSS。

相对**本轮**原始联合求解，新版query P95下降中位39.9710%，后端总墙时下降39.4388%，通过原30%/20%的离线门；这与本轮新增的20%增量目标是两个不同分母。不能把原门通过写成本轮全部门通过。结论保持`correct_but_query_gate_not_met`，没有追加调优或第二轮比较。

## 数值、误差与范围

126个计划三臂接受/拒绝状态一致；旧、新缓存版相对联合求解的位置最大差均1.92686e-13m，完整3×3协方差最大差均8.10441e-18m²，满足原容差。输入和窗口未被查询污染。估计与计时封存后才打开原真值，原yaw0/offset0及局部首帧平移原点保持，不拟合旋转或尺度。

评分92/92输出，旧、新缓存版误差及协方差统计相同：XY误差P95为3.793635mm；平移动作52/52，P95为3.013778mm；转向23/23，P95为3.673331mm。模型条件XYσ起终4.469122→4.745214mm，P95为4.781416mm。本次保持原模型结果，未减少实际误差或不确定性累积。局部初始化与原在线前缀不同，不能与native的8.6mm作精度改善比较，离线92/92也不能当作在线无丢帧。

本次仅测冻结前端后的后端，未测ROS/渲染负载下的端到端延迟、实时交付或连续性。普通帧耗时降低是有效进展，在线实时性仍未验收。

## 封档

唯一正式比较84.449墙秒/83.904进程CPU秒正常结束。31个保护源未变，8个比较源码及helper/input哈希首末一致；AST核对确认freeze及共享辅助未改。31份证据副本与1份复用冻结输入原位哈希已保存，旧源码、原测试、两次局部profile和比较器一并归档。所有计算已停，容器仅保留`sleep infinity`。

- [完整结论与验收门](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-frame-offline-20261023/comparison-01/summary.json)、[逐组值](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-frame-offline-20261023/comparison-01/observations.json)、[原始计时](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-frame-offline-20261023/comparison-01/timing.json)。
- [126计划等价检查](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-frame-offline-20261023/comparison-01/equivalence.json)、[17项检查](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-frame-offline-20261023/cache-tests-01.json)、[局部原因证据](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-frame-offline-20261023/query-cause-before.json)。
- [副本及源码哈希](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-frame-offline-20261023/integrity.json)、[复用输入原位哈希](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-frame-offline-20261023/raw-files.json)、[估计封存及真值隔离](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/query-frame-offline-20261023/comparison-01/estimator-seal.json)。
