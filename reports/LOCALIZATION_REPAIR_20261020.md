# MoonCar 局部定位修复与验证（2026-10-20）

本轮新增的显式 `direct` 局部定位模式已通过开发用 +120° 及冻结后 -75°、+135°、-165° 三组新角度的完整 Gazebo 勘测流程。四组转向末 XY 误差最大 0.827cm、转后静止绝对误差 P95 最大 1.018cm、静止位置波动最大 0.879cm，均满足原 3cm／3cm／1cm 门槛。原默认模式保留，不把局部固定参考的结果推广为长距离全局定位或实车验收。

使用方式见[局部模式说明](../docs/local_metric_localization.md)。证据总目录为 [localization-repair-20261020](evidence/localization-repair-20261020)，包括全部失败尝试、原始传感器 bag、独立真值、冻结源码及评价脚本。没有提交或推送。

## 修复依据与实现

旧匹配器把越来越密的参考点中最近八点作为法向支持。原 +120° 数据中的典型支持尺度缩小到约 23.5mm，与 10mm 量程噪声接近；线性检查因此频繁失败。新模块使用有物理尺度的法向支持和固定的初始实测参考，防止反复加入同一表面后把邻域缩成噪声团。参考不足、二维约束退化或修正突变仍会拒绝。

完整在线 V3 又发现少数轮廓边缘坏对应可占残差平方和的 85%–89%。最终回退只在原全点 RMSE 超过 3cm 时执行一次 3σ 内点重拟合，要求至少六点、严格超过当前全部有效回波的一半，重算对应并保留二维秩和 10cm 突变界限。原普通成功更新保持一致；没有放宽原残差数值。法向诊断、失败的全量裁剪方案及最终输出冻结／等价核对均保留在 [offline-matcher](evidence/localization-repair-20261020/offline-matcher)。

历史 +120° 记录还显示：激光观测中断后，旧位置到 EKF 的方案继续积分潜在速度，静止 odom 波动达 16.31cm。单纯绕开 EKF 仍有约 1.15–1.57cm 原始激光波动，因此新模式只在真实接受的新扫描上输出 XY＋同步 IMU 朝向，并以最多 0.4 秒／八个测量的窗口抑制噪声。窗口不施加 `vy=0`；真实拒绝会清空窗口，突变时返回当前新匹配。等间隔等权的小运动可能约有 0.2 秒平均滞后，不能承诺普遍硬上限。详见[融合诊断](evidence/localization-repair-20261020/fusion-diagnosis/REPORT.md)；`sensor_timeout` 不是逐激光消息的冻结开关，参见 [robot_localization 官方实现](https://github.com/cra-ros-pkg/robot_localization/blob/jazzy-devel/src/ros_filter.cpp)。

V3 的同时间戳实际 TF 分解还发现：峰值处直接位姿误差仅 0.683cm，SLAM 加入实际 (+4,+2)cm 网格修正后，map 误差达到 5.090cm。显式 `direct` 模式因此在全新 mapper 中关闭二次扫描匹配和回环，让占据图使用同一测得位姿；地图内部与 TF 一起保持一致。核对了安装的 slam_toolbox 2.8.5 官方固定提交，并用实际安装的 Karto 库验证关闭匹配后仍可生成占据图。已有图不能仅靠运行中改开关清除旧偏移，必须新建会话。[实际 TF 分解](evidence/localization-repair-20261020/fusion-diagnosis/new-v3-diagnostics.json)、[官方代码与安装库验证](evidence/localization-repair-20261020/fusion-diagnosis/slam-toolbox-2.8.5-source/FINDINGS.md)、[官方 Mapper::Process](https://github.com/SteveMacenski/slam_toolbox/blob/ec8f7635dea317b531c419f798f87d90a336f32e/lib/karto_sdk/src/Mapper.cpp#L2716)。

质量摄取与时钟放在独立执行器，故障计数锁存到旧动作取消。V4 首次前进仍误停：新位姿已到达，但质量状态延迟约 91.9ms 才公布其 accepted stamp，旧广告时间戳因此过期。V5 在每个真实接受扫描发布后立即报告状态，同时保留 0.1 墙秒心跳。0.3 仿真秒测量限时、0.5 墙秒状态限时及三次新测量恢复保持；心跳和重复状态不刷新位置或增加成功次数。[精确时序证据](evidence/localization-repair-20261020/fusion-diagnosis/new-v4-first-action-diagnostics.json)。

## 冻结后完整流程

开发旧记录和新 +120° 均参与方案选择；只有最终 V5 冻结后采集的三个新角度列为独立验证。程序每组只执行一次，任一功能或定位门槛失败即停止。未用真值初始化、修正定位或拟合 map/world 对齐；真值只供事后评价。协议和 465 份源码冻结见 [design-protocol.json](evidence/localization-repair-20261020/design-protocol.json)、[candidate-v5-freeze.json](evidence/localization-repair-20261020/candidate-v5-freeze.json)。

下表单位均为 cm；全程误差列如实列出，预先登记的定位门槛是转末≤3、静止 P95≤3、静止波动≤1。

| 请求转角／用途 | 转末 XY | 静止 XY P95 | 静止波动 | 全程 XY P95／最大 | 帧数 | 完整验收 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| +120°／开发回归 | 0.463 | 0.959 | 0.879 | 0.811／1.511 | 30 | 通过 |
| -75°／新工况 | 0.406 | 0.867 | 0.776 | 0.957／1.418 | 29 | 通过 |
| +135°／新工况 | 0.098 | 0.823 | 0.640 | 0.775／1.452 | 28 | 通过 |
| -165°／新工况 | 0.827 | 1.018 | 0.482 | 1.035／1.925 | 35 | 通过 |

四组都完成两次真实前进、完整转向、静止、Gazebo 暂停、接收端断开、积压产生及同会话补传清空。沿用原动作成功条件，实际转角分别为 +123.472°、-78.131°、+139.084°、-167.540°，没有改转向控制来制造精确角度。所有功能／定位门槛见各组 `validation.json`、`localization-evaluation.json` 及 [independent-results.json](evidence/localization-repair-20261020/independent-results.json)。

![四组冻结结果与原门槛](evidence/localization-repair-20261020/localization-summary.png)

## 失败尝试保全

| 新 Gazebo 尝试 | 实际结果 | 完整验收 |
| --- | --- | --- |
| 原基线 +120° | 两次前进及完整转向成功；转末／静止 P95 均约 4.275cm | 失败 |
| V1 | 三次动作均被联锁取消，实际转角 0° | 失败；不能算转向精度通过 |
| V2 | 前进约 4.3mm 后取消，后续转向请求未就绪，流程不完整 | 未完成；不补造统计 |
| V3 | 首次前进成功，转向约 22.633° 后因残差拒绝取消 | 失败；不能算完整 +120° |
| V4 | 完整转向的定位数值通过，两次前进因滞后质量时间戳取消 | 失败 |
| V5 | 开发用 +120° 完整流程通过，之后冻结代码执行上述三个新工况 | 通过 |

完整六次尝试、动作量、门槛、原文件哈希与因果边界见[开发尝试证据](evidence/localization-repair-20261020/fusion-diagnosis/attempt-summary-draft.md)。不把 `trial.completed=true` 等同于动作成功或全部门槛通过，也不把不同运行的误差差值当作普遍因果提升。

基线原传输精确检查另有一个分析适配器误报：重载时对两个已经归一化的四元数再次归一化，改变约一个 float64 ULP。原 NPZ、发送队列和接收字段规范化比较全部精确。保留原失败结果，仅增加[独立审计与资格说明](evidence/localization-repair-20261020/baseline-left120/functional-qualification.json)；后续分析器直接比较持久化原值，不加容差。基线的定位失败结论未变。

## 断测联锁与交付核验

首次 `live-fault-01` 的测试观察线程因 ROS DiagnosticStatus 字节字段的 JSON 序列化错误退出，观察器缺失后流程超时。`live-fault-02` 完成动作序列，但故障和恢复期间的原始扫描、直接 odom 未完整录下，两项证据门槛失败，不能靠质量消息补过。两次原始失败、bag 和[仪器诊断](evidence/localization-repair-20261020/fault-harness/fault02-visibility-diagnosis.md)均保留。

`live-fault-03` 在独立观察器和原生 bag 都收到至少三份真实消息、保持原0.3秒新鲜度并连续推进0.3仿真秒后，才开始动作和中断。日志改在容器原生目录落盘，避免回调同步刷写 Windows 挂载。仅暂停本实验的水平扫描过滤进程，保留原始扫描、时钟和质量心跳；没有合成状态／位置或改变生产代码。

这次中断年龄达到0.455仿真秒，原始扫描持续。旧动作取消，直接位置时间戳在中断中不刷新；未知窗口没有采集帧或合格SLAM扫描。恢复时真实 direct odom 的33.2／33.3／33.4秒三个不同时间戳逐一匹配质量消息，重复心跳没有增加测量次数。同一生产者恢复后原动作保持取消，只有新 lease 动作再次运动。

第三次原始 `fault-evaluation.json` 对速度命令仍有两项误报，保留为false：深队列回调在较晚时刻收到的非零值可逐项匹配更早的历史输出，其到达时刻不能当作实际发布时刻。独立原生 bag 明确区分 `/mooncar/cmd_vel_safe` 与 `/cmd_vel`，故障窗口12条、恢复后到覆盖新动作边界的10条安全速度均严格为零，首次非零位于新动作之后；原浅队列观察器的同墙钟记录亦为零。最终[独立资格审计](evidence/localization-repair-20261020/fault-harness/fault03-safe-command-qualification.md)以这些证据复核原两项门槛，全部原门槛通过，未更改阈值或覆写失败记录；不对具体DDS内部原因作过度推断。完整原值和录包位于 [live-fault-03](evidence/localization-repair-20261020/live-fault-03)。

独立真值事后显示，中断时约0.16m/s，按≤0.005m/s持续0.1仿真秒的报告判据，观测过期后约0.349仿真秒停车，过期起位移约3.14cm。这是一次有限仿真观测，不是制动距离认证或位置精度门槛。原评价中因缺少该回调零命令而回退计算的 `stop_delay_after_first_observed_zero` 字段不适用，不能把它当成真实零输出时间。

四组共 122 帧、257957 个原始点，源档到接收器的声明规范字段精确一致；12 包共 972 项完整 checksum 自校验、Linux 与 Windows 重算的 summary／cloud／观测档案字段均核对。不同平台 NumPy 版本使 `cells.csv` 三个拟合浮点列末位不同，最大高程差约 2.78e-16m，不能声称所有派生文件逐字相同；原始点／位姿未通过容差或重新归一化掩盖差异。[核验记录](evidence/localization-repair-20261020/delivery-v5-final-windows-verification-v3.json)、[总帧数及逐帧复核](evidence/localization-repair-20261020/delivery-v5-frame-count-erratum.json)。本轮未追加浏览器人工验证或 HTTP ZIP 导出验证。

## 保留范围

原激光匹配核心、原及旧候选 EKF 配置、原 SLAM 配置文件、共享质量策略保持原哈希；92 份车辆／Gazebo 相关源文件自本轮首次候选冻结起核对不变。新 SLAM 行为只由显式 direct 启动分支覆盖。原 5 秒目标确认、TF／质量门槛、车辆、土壤、接触、传感器及转向参数不变，覆盖仅作统计。见[源码核验](evidence/localization-repair-20261020/final-source-audit.json)。

共执行12次新Gazebo启动（6次基线／开发、3次冻结独立工况、3次断测尝试），0次硬件；没有隐藏重试或只保留成功。126项生产／服务检查、11项实际ROS观察器／可观测性检查和4项合成评价检查通过；这些检查不增加物理运行计数。八个ROS包构建通过，4组场景文件与本轮基线的SHA256完全一致。最终[验证汇总](evidence/localization-repair-20261020/verification.json)与[运行环境清理记录](evidence/localization-repair-20261020/runtime-cleanup.json)分别记录源／证据与容器状态。

这是初始实测特征仍可见时的局部定位修复。离开初始参考、遮挡、重复／动态几何、共同 IMU 偏差、坡上扫描过滤、长任务、全局回环、R5 及实体标定仍需专项工作。协方差下限不代表实物精度，四次有限仿真也不是统计性能保证。最后一组退出日志保留了 rclpy Destroyable 警告；进程清理另行核验，不把正常流程通过写成无警告退出。
