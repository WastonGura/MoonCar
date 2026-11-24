# RGBD 多关键帧滑窗：独立离线原型

本目录包含独立的离线原型，以及显式开启的实时旁路适配。旁路由单独脚本启动，只订阅传感器并记录 JSONL，没有接管工作台默认定位，也不修改导航或回访。

最新[固定输入完整链AB/BA对照](../../reports/FULL_CHAIN_AB_20261023.md)：最近IMU修复前后版本共用完整录包、492pairs/491plans及资源，从源计划发布量到实际JSON flush。四run均489输出、raw丢失3/492、成功walllate0；两对P95为130.5→132.7ms与130.5→125.2ms，未达到预设整体改善门。489共同输出p/fullC相同；没有复现先前在线大幅退化。此为无Gazebo/default并发的隔离回放，未替代工作台验收；本轮没有产品修改。以下保留历史。

最新[IMU接收修复与V2结果](../../reports/SHADOW_IMU_RECEPTION_V2_20261023.md)：独立旁路IMU改RELIABLE/KEEP_LAST200/VOLATILE，原500历史缓存、真实括号和0.5秒等待保持。26项检查通过；唯一V2未再IMU超时，最大空窗.646→.327sim，但整体丢弃4.76→5.78%、wallP95.415→.518秒、成功迟到7→27。新增有界IMU/bracket及阶段墙/线程CPU/进程CPU诊断，原型`timing_diagnostics=False`默认不输出，node显式开启。25/27迟到最大内部段是视觉前端；实时性仍未通过。默认、回访、估计数学及资源保持；以下为历史。

显式缓存KF现用`answer, posterior = estimator.add_frame_with_posterior(frame)`，随后`freeze_query_cache(estimator, version, posterior=posterior)`，仅消费同次成功提交的H/eta/mean/LU，去掉重复装配/分解。owner/提交/keys/配置/源指纹匹配且只消费一次，失败/过期拒绝；native LU不进入持久estimator状态，cache独立持有数据。原add/query返回和不传posterior的freeze路径保留。[尾部验证报告](../../reports/SHADOW_TAIL_V2_20261023.md)：55专项通过，126两臂位置/完整C/数值诊断与持久状态逐位一致，KF含cache离线P95降25.42%、构建降53.07%。唯一V2整体未改善：丢弃4.76%、输出460/483、wallP95.415秒、成功walllate7（上轮3）；不能以离线收益宣称连续性修好。默认/回访/视觉/节点/资源未改，下文为历史。

视觉前端新增固定21×21`_sample_patch`：标准uint8图/内部float32中心仅按点求floor/小数权重，原四邻像素和四项乘加顺序保持；其它输入回原`_sample`。[本轮报告](../../reports/VISION_FRONTEND_V2_20261023.md)：26专项通过，126帧全部特征/选择/跟踪/位置/完整C逐位一致，前端P95减少32.18%、wholeprocess总墙时减少23.60%。唯一V2平移/转向完成，丢弃15.42→3.32%、输出495/513=96.49%、墙钟端到端P95.377秒已低于.5参考线；仍3个成功输出超墙时门，时间新鲜覆盖89.41%，未声称完全连续。默认、回访、后端和资源不变，仅仿真优化；下文为历史。

现可在`scripts/rgbd_window_shadow.py`显式传`--cached-query`启用缓存普通帧，或用`ShadowProcessor(cached_query=True)`；不传仍为原路径。成功KF递增版本并freeze，失败KF/构建禁用旧cache，query不提交、不回退旧求解。cache_build_s已包含backend_s。见[单轮缓存V2报告](../../reports/CACHED_QUERY_SHADOW_V2_20261023.md)：39专项通过，真实平移/转向完成，302缓存query+92KF构建成功；全计划输出394/467、配对丢弃15.42%、墙钟端到端P950.606秒仍超0.5秒参考门。实时连续性尚未通过，默认与回访未改。下文“未接在线”等描述属于各历史轮次。

[普通帧查询优化报告](../../reports/QUERY_FRAME_OFFLINE_20261023.md)：仅批量校验标准float64观测及按帧批量计算残差；非标准/异常输入回原校验器，保留首个报错顺序及数据独立性。freeze、完整相关协方差、选择列求解、原门及RMS诊断不变。17专项通过，同冻结126帧三臂等价；相对修改前cache，query总墙时减少26.22%、P95减少14.83%，未达本轮20% P95目标；关键帧含cache P95配对中位+3.81%、全部后端总墙时减少13.77%。原legacy相对30%/20%离线门现通过，但不能当作本轮全部验收或在线通过。未接工作台/默认/回访，原在线processor仍未替换；下文报告保留历史口径。

[关键帧构建缓存后续报告](../../reports/QUERY_CACHE_BUILD_OFFLINE_20261023.md)：freeze去掉整个estimator的递归deepcopy，仅隔离装配所需可变容器；返回缓存仍独立持有数组，调用期间提交/修改须串行。12项检查通过。同一冻结126帧、三臂轮转比较中，构建P95减少37.40%、全计划后端P95减少17.05%、query基本不变；新/旧缓存数值等价。仅缓存构建验收通过，原整体query提速门仍未过，未接在线或新增V2；旧报告保留各自历史口径。

新增独立离线 `query_cache`：`freeze_query_cache(estimator, version)`复制已提交窗口后验，`query_cached(frame, cache, expected_version)`利用完整相关地标边缘求当前9维状态；普通query不提交观测。新地标单次观测解析消去，保留历史角度门和原C下限。版本由调用方在成功KF后递增；cache拥有快照，不会跟随原窗口自动更新。原 `query_frame`/`add_frame`、在线processor均未替换。`estimated_bytes()`为数组/稀疏存储估计，不是RSS；新9维条件性诊断不冒充原全窗LU的pivot检查。

新增专项 `tests/test_offline_rgbd_window_query_cache.py` 10项通过。固定录像比较见[离线缓存求解报告](../../reports/QUERY_CACHE_OFFLINE_20261023.md)：126计划位置/完整C数值等价，评分92/92；query P95中位降29.7447%未到预设30%，含cache的后端总墙时降23.017%，但关键帧尾部变重。严格提取器 `extract_query_clip.py` 与 `compare_cached_query.py` 只处理指定闭合录像，输出目录必须为新目录；不启动ROS节点或世界。此次原型没有证明在线实时性改善，按计划停止，没有继续扩大。

最新[吞吐修复与单轮V2报告](../../reports/SHADOW_THROUGHPUT_V2_20261022.md)：间距筛选新增cKDTree粗筛，仍保留原严格平方距离终判、顺序与异常回退；局部真实输入位置/完整C/特征结果精确一致，22项前端相关检查通过。运行helper显式可选`apply(..., shadow_nice=5)`（仅5或19，普通调用默认仍19），本轮旁路与Gazebo在原CPU2–5共享nice5；默认CPU0,1及总配额不变，11项运行检查通过。唯一V2完成实际平移/转向，丢弃率降至24.95%、输出361/482，但墙钟P95升至0.914秒，时间新鲜覆盖43.04%。仿真到达节奏也变慢，不能据仿真时间改善宣称实时性通过；默认和回访保持原样。

## 模型与范围

- ORB 检测只补充新点，金字塔正反 KLT 持续跟踪已有像素身份；每帧重新读取真实深度及曝光时刻的动态云台外参。失踪地标不凭几何邻近伪造重连。
- 跨帧共享地标；窗口内联合估计平移、IMU 名义姿态的小角修正和每曝光公共测量误差。
- 姿态先验标准差 0.1°，每曝光公共误差标准差 3mm，独立点噪声沿用原 3mm 深度和 0.5px/ORB 尺度模型。
- 这是固定名义姿态的一阶 MAP 模型，姿态修正超过 3°即拒绝；不包含完整非线性 BA、速度、惯性预积分或 IMU 零偏。
- 默认六帧、每帧最多 96 个实际观测。移除旧状态时 Schur 边缘化保留交叉信息，不在新窗口重设原点。
- 不同曝光的姿态/公共误差先验目前假定独立，未标定真实 AHRS、深度标定与 LK 的跨时相关性。因此协方差是该模型条件下的估计。

跟踪中的可选亚像素精化若没有改善图像块误差，但正反光流已通过且位移不超过原有 0.5px 定位误差，则保留当前图像中未精化的像素位置，而不是删除整个特征身份。该像素仍重新读取当前深度和协方差，并通过原几何、空间支撑及共享地标检查；不沿用上一帧的三维点或位姿。

转动时，使用当前曝光已有的 IMU 姿态、云台外参和相机内参，将上一帧的三维观测投影到当前图像，作为 KLT 的搜索初值；平移仅在搜索提示中假定为零。投影不直接生成观测，也不提供位姿约束，仍须通过真实图像的正反光流及后续检查。缺少有效投影时回退原像素初值，不使用真值、未来帧或新估计位姿。

## 公平比较

`run.py` 默认在相同合法曝光上完整重放当前 `MetricRGBDOdometry`，也可按下文复用同录像的封存基线；不改变旧定位器的接受门槛。新原型默认在 10Hz 图像上维护轨迹，每 0.5 秒计划一次优化输出。所有计划失败都保留，不仅统计两法均成功的样本；高于 50mm 的输出也不删除。

新解固定第一真实曝光的位置为零。报告同时保留条件协方差及外加一次旧方案初始平移协方差 `C0` 的敏感性口径；`C0` 不进入优化器，也不按帧累加。后者依赖独立起点误差的假设，不能冒充已标定的完整后验。

真值文件在全部估计完成后才打开，只按时间插值评分，不外推、不向估计器提供真值、不拟合轨迹旋转或尺度。数据末端无真值时仍计输出/50mm通过率，只缺少误差评分。

2Hz 原型与原 10Hz 定位器的差异属于整体管线对照，不能把收益单独归因于滑窗。原记录路径也不能证明新控制器会走出相同的自主探索路径。

## 运行

需要 Python 3.10+、NumPy、SciPy、OpenCV。现有验证容器已提供这些依赖，不需安装；`/source` 为源码只读挂载，输出写入 `/evidence`。

```sh
docker exec -e PYTHONDONTWRITEBYTECODE=1 -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 \
  mooncar-terrain-validation python3 /source/scripts/offline_rgbd_window/run.py \
  --manifest /source/tmp/ros_acceptance/localization-expansion-repair-20261020/rgbd-observability/native25-continuity-repair-01/full-sequence/manifest.json \
  --truth /source/tmp/ros_acceptance/localization-expansion-repair-20261020/rgbd-autonomy-25-revisit-retarget-6cpu/observer/truth.jsonl \
  --output /evidence/offline-rgbd-window-20261021/example-new-output
```

输出目录必须不存在，避免覆盖旧证据。`--max-sensor-frames` 只用于明确标注的截断 smoke 检查，正式比较保持默认完整输入。

可用 `--baseline-run <已封存的同录像完整run目录>` 复用旧定位器的冻结输出，避免重复计算基线。此模式核验旧输出哈希、当前旧定位器源码哈希、输入 manifest、逐帧时间戳和完整尾数；缓存位姿不进入新估计器。基线耗时标为历史测量，本次前端及后端耗时另行记录。

输出包括逐曝光 `baseline.jsonl`、逐计划 `window.jsonl`、误差/覆盖率/耗时 `metrics.json`、源文件和数据哈希 `run.json`。输入 chunk 哈希及校准几何在读取时检查。计算耗时不包含档案 I/O、ROS、解码和调度，不能用它认定在线时效通过。

专用测试：

```sh
python3 -m unittest discover -s tests -p 'test_offline_rgbd_window_*.py' -v
```

测试包含滑窗与批量解的一致性、动态云台杆臂、公共误差不随点数消失、失败回滚、数据时间与真值隔离，以及重复图案不能生成独立身份。

## 最小实时旁路

在已启动的工作台实验对应 ROS domain/Gazebo partition 中，显式运行 `scripts/rgbd_window_shadow.py`。继续使用现有定位控制车辆；该节点不发布定位、TF、健康状态或控制消息，估计输入不含真值、地图、旧定位位置或 C0。

```sh
source /opt/ros/jazzy/setup.bash
source /work/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=89 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
# GZ_PARTITION 应与当前实际实验身份一致，不能复用历史实验编号。
python3 -B /source/scripts/rgbd_window_shadow.py \
  --output /evidence/NEW_TRIAL/shadow.jsonl --image-reliability reliable \
  --ros-args -p use_sim_time:=true
```

RGB/深度按真实曝光时间精确配对，CameraInfo 必须完整一致，IMU 使用传感器 QoS，并使用曝光时刻的真实相机 TF。复用已安装生产节点的图像/深度解码和车体遮罩；相机/IMU就绪等待有界。前端只处理实际能收到并调度到的曝光，不保证达到离线的 10Hz。

计算线程之外最多保留一个最新待处理曝光，每路未配对图像最多 8 份、IMU 最多 500 份。替换、拒绝和关闭前未处理的曝光均记录；首个真实配对锚定 0.1s 固定计划，丢失计划显式失败。实时tick只检查时钟；缺失区间由后继真实配对的曝光时间确认，停止采集后补齐尾部，不用先行的仿真时钟提前取消尚未抵达的曝光。只在同一计划区间内将尚未开始计算的待处理计划交给更新的真实曝光，保留新曝光本来的采集和接收时间；跨区间替换及未执行计划照常计入失败。慢解仍保留原曝光时间和误差，不套用生产 0.3s 门把迟到结果删掉，也不重新打时间戳假装新测量。退出时等待线程终结，并记录完整 `run_end`。

`online.py` 保持原六帧、96观测、0.1°/3mm 模型和至少24共享点等前端要求，只输出条件协方差 C，不外加离线敏感性对照的 C0。在线每 0.5s 提交一次持久关键帧，其间以当前真实曝光临时加入同一联合优化，输出后恢复持久窗口与前端地图；查询不会重复累计观测，也不重置协方差。离线 `run.py` 的原 2Hz 计划不受在线查询频率影响。`evaluate_shadow.py` 在采集结束后读取独立真值，按全部固定计划统计误差、丢失、最长空窗、交付时测量年龄与墙钟延迟；同时间基线比较只接受真实精确 TF 样本。评价需要明确的 IMU→世界偏航约定，不拟合轨迹。

新增专项检查可分别运行 `test_rgbd_window_shadow.py` 和 `test_offline_rgbd_window_shadow_evaluation.py`。首次 V2 实际旁路结果见 [验证报告](../../reports/RGBD_WINDOW_SHADOW_V2_20261022.md)：适配和采集已接通，实时可用性未通过；启动观察切换时触发验证边界停止，未执行固定行驶路线，不能据静止误差宣称移动精度通过。

矩阵固定项缓存和特征间距筛选的数组复用只减少重复构建工作；已在固定真实输入上验证位置、完整协方差及前端结果一致。在线延迟与运动覆盖以 [本轮 V2 报告](../../reports/RGBD_WINDOW_LATENCY_V2_20261022.md) 为准。

默认定位超时修复及实际平移、转向旁路结果见 [V2运动报告](../../reports/DEFAULT_RGBD_TIMEOUT_V2_20261022.md)。该轮显式在默认节点导入原生库前用 `MOONCAR_RGBD_CPUSET=0,1` 分配CPU，并用 `scripts/rgbd_shadow_runtime.py` 核对实验进程身份、限制该实验渲染与旁路到CPU2–5（旁路nice19）。它是共享6CPU额度下的显式实验配置，其他ROS进程仍共享资源；未替换默认算法或更改0.3sim/0.5wall门。普通decoder模块导入不执行启动绑定。默认本轮无计算前/后观测超时，实际运动已覆盖；新旁路成功输出的运动误差约4mm，但最长交付空窗2.051仿真秒、时间新鲜率23.04%，仍不能接管默认。

[调度连续性修复与单轮V2报告](../../reports/SHADOW_CONTINUITY_V2_20261022.md) 记录了原2.051仿真秒交付空窗的漏计划/关键帧断接原因、同184输入对照以及新一轮移动/转向结果。新最大交付空窗0.454仿真秒，所有实际处理曝光均有计划；仍有替换、共享门拒绝和迟到，时间新鲜覆盖29.19%，不能据此替换默认定位。
