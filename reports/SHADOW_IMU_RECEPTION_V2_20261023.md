# 旁路IMU接收修复与单轮V2验证

2026-10-23，Asia/Tokyo。

**本轮未再出现IMU括号等待超时，最大交付空窗缩短；整体丢帧和迟到仍未解决，端到端P95升至0.518秒，超过原0.5秒参考门。** 只完成一次最小接收修复和一次V2，没有追加调参、第二个世界或替换默认定位。

## 改动及选择理由

上一轮已确认发布端有2.9秒附近连续IMU，而浅best-effort旁路未能取得合格历史括号。将独立旁路的IMU订阅显式改为RELIABLE / KEEP_LAST200 / VOLATILE；现有仿真发布端支持reliable，与前次可靠录包的接收配置一致。这个选择只适用于已核对的仿真发布端，不能推成任意硬件发布者都兼容。

保持图像QoS、500样本历史缓存、真实header、精确曝光TF、原IMU跨度/邻近门、0.5墙秒等待、10Hz计划、0.5sim秒关键帧周期、模型和全部质量门。更深DDS接收队列用于承接突发，不是扩大图像待处理队列；仍只有一个latest-pending图像槽。

增加256条IMU回调环形记录，成功只附邻居摘要，失败附有界最近记录；不逐IMU写出。run_end保留全程回调计数及最大header间隔。增加实际offer/take、准备/前端/选择/观测构造/求解/冻结/plain阶段墙时、worker线程CPU、进程CPU，以及原plan/exposure写完后的worker_complete事件。旧finished和评价门语义保持；后写出事件自身仍有成本。原型`ShadowProcessor(timing_diagnostics=False)`默认不输出阶段诊断，独立旁路显式开启。

只改node与online适配。node SHA `f720f185cedad7570f3786c43490a8d05fb7ec10c20826b0e619426c626efef8`，online `2658aabacbd835ed86f8d83af726cebfc916c9ddfae48bc5fec74ef671163907`；core、query_cache、视觉实现、生产默认、回访及资源保持原hash。

## 定向检查

26/26通过，0失败/错误/跳过，0.191墙秒。执行真实构造器、on_imu、prepare和原imu_at函数：验证可靠有界QoS、200条回调保留原header、迟滞clock不重写曝光、合法括号可用；2.84→2.91秒真实缺口即使继续收到未来IMU仍按原0.5秒门拒绝，无外推。fake传输不冒称DDS压力测试。

真实小窗口cached路径另验证诊断开关前后每帧位置、完整协方差、状态/时间戳与持久估计器pickle相同。没有修改估计数学或重跑126帧性能比较。独立闭合审计脚本的3项纯解析/分母测试也通过。

## 唯一V2结果

`experiment-321d6e744da5`，原无标柱native V2、legacy/nogui/start0，原6次云台→等待→平移→转向→等待路线全部完成。资源仍6CPU/6GiB/1GiB，默认定位CPU0–1/nice0、Gazebo与旁路CPU2–5/nice5，BLAS/OMP1/OpenCV2。全窗0.948→47.595sim，共46.647sim秒，启动失败与结束缺失全部计入。

| 全窗指标 | 上轮ffe3 | 本轮321d |
|---|---:|---:|
| IMU括号等待超时 | 1 | **0** |
| 配对曝光 / 覆盖 / 拒绝 | 483 / 21 / 2 | 467 / 25 / 2 |
| 配对丢弃率 | 4.76% | **5.78%** |
| 计划成功输出 | 460/483（95.24%） | 439/467（94.00%） |
| 端到端墙时P95 / 最大 | 0.415 / 0.661s | **0.518 / 0.627s** |
| 成功输出wall>0.5s | 7 | **27** |
| 成功输出sim年龄>0.3s | 2 | 1 |
| 双时效门成功 / 全计划 | 453/483（93.79%） | 411/467（88.01%） |
| 时间fresh覆盖 | 86.94% | 83.09% |
| 完整交付最大空窗 | 0.646sim | **0.327sim** |
| 最长notfresh | 0.394sim | 0.269sim |

467原始pair=440processed+25replace+2reject；440processed含1个无独立计划的真实曝光。467计划=439output+25replace+启动TF超时1+关闭前未处理1+关闭缺曝光1。93次KF/cache、346次cached query全部求解成功。尚有启动精确相机TF等待超时，不能声称所有就绪问题消除。

本轮实际旁路回调4662条全部结束并接受，拒绝0；累计最大header间隔60ms，超过30ms一次、超过60ms零，**不宣称零IMU样本遗漏**。440份准备记录均取得真实合格括号，最大约10ms，无bracket-unavailable、无IMU等待超时。原袋recorder最大间隔50ms与旁路回调是不同证据流，不能混用；每次准备均保存实际缓存和邻居。更深可靠队列在这轮起到了预期作用，但不同启动/输入且同时启用诊断，单次差值不足以证明永久修复。

## 新计时查到的剩余瓶颈

439成功输出中27次原墙时迟到（12query/15KF），其中25次最大内部耗时段为视觉前端，2次为求解。前端累计62.821墙秒，占processor父区间96.071墙秒约65.4%，前端P95为237.895ms。求解累计25.592秒/P95147.064ms，冻结累计2.715秒/P9559.386ms；父子区间及分位数不相加。

原finished到原plan/exposure写完P9510.492ms/max27.934ms；已offer的后继交接间隙P954.715ms，当前证据没有显示序列化/写出/交接主导这批迟到。按成功输出的原finished口径P95517.965ms，原写完口径P95520.520ms，后者超0.5秒31次，单列而不替换旧评价口径。

前端累计worker线程CPU/墙时约0.507、全旁路进程CPU/墙时约0.679；后者包含ROS回调/OpenCV其他线程，前者不含原生并行线程。墙时差不能直接命名为GIL、纯算法计算或系统调度。solve/freeze仍包含签名及校验，未从总阶段反推签名热点。新诊断本身也有成本；配对到达sim/wall比0.479→0.392，无法将整轮变差单因果归给QoS。

下一优先段已明确为视觉前端及其运行时长尾；本轮没有继续修改它，也未继续给回访打补丁。

## 误差、健康与闭合

439输出均可真值评分，XY误差P957.970mm/max7.983mm；条件σP954.890mm，不是已标定全局后验。移动63/68、转向59/65、纯转向7/8；实际平移净0.781134m/路径0.786165m，实际转向54.917°并漂移41.586mm，证运动覆盖，不当45°控制跟踪通过。55个相同stamp新/默认误差P95为7.970/4.280mm，不宣称新方案更准。

就绪8.071sim之后474次API及1423条源质量全healthy，global fault counter保持1；启动stale/recovering另保留。driver/shadow/observer/bag均exit0、非forced，owned世界停止、子进程0，含收尾134.773墙秒；工作台后端保留。30项保护源与16项候选首末一致，估计记录封存后才用独立truth评价。默认定位、回访、实车范围未改。

证据：[26项检查](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-imu-reception-v2-20261023/tests-01.json)、[完整评价](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-imu-reception-v2-20261023/evaluation/summary.json)、[实际IMU准备与原袋对照](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-imu-reception-v2-20261023/imu-reception-audit/summary.json)、[阶段计时](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-imu-reception-v2-20261023/phase-audit.json)、[源码与闭合校验](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-imu-reception-v2-20261023/integrity.json)。
