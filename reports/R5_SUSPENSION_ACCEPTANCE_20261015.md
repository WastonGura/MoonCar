# R5 模型修正与悬架验收记录（2026-10-15）

TF 模型绑定、实验 TF 发布时序和平地参考检查已修正；静态精度目标通过。
**完整验收未通过，不能标为“支撑模型修复候选”。**

最终 v3 批次独立通过 95 项无物理步进回归和 12/12 次新诊断。正式矩阵声明恰好
17 轮，启动 10 轮，其中 9 轮通过执行资格检查；第 10 轮在物理步进前因 DDS
生命周期观察器的权限错误停止，余下 7 轮未启动。没有补判、拼接旧成功或原批重试。

## 已修正的内容

1. 每轮 ROS `robot_description` 按刚性／悬架选择绑定相应 URDF，冻结描述文件哈希，
   运行时以实际 JointState／TF 核对轮轴和被动悬架变换。
2. 实验 `robot_state_publisher.publish_frequency` 明确设为 30 Hz，与原有 30 Hz
   JointState 节流匹配。默认 20 Hz 的过滤会令 33.5／67 ms 输入出现 100.5 ms
   TF 匹配间隔。无物理步进 ROS 时钟回放复现旧失败，修正后 32 个采样全部匹配，
   最大间隔 67 ms，平移误差为零。100 ms 验收上限保持不变。
3. 实验平地使用 `qualified_flat_mesh_v1` 地面参考。先检查实际加载网格的三角面覆盖、
   高程、孔洞、障碍物及路线余量，再在已核验范围内使用零高程。旧归档 fixture 的
   `ramp` 路径曾令检查器读取原坡道高度图，在 2 m 处制造约 40 mm 假穿透。
   高度门槛仍为 −10～30 mm；非平面、孔洞、越界和非有限查询均拒绝。
4. 保留并回归已有精度与见证收尾修复：`r5_precision_v1` 主步长 0.25 ms，支撑选择
   与精度选择独立；事件排空与原生读流退出并行，见证绝对截止 5 秒、父进程 8 秒，
   flush／关闭、计数和事件 ID 核对完成后才报告成功。

生产默认仍为 `rigid`，三个生产世界仍为 1 ms；默认 R5、旧模型展开和安装的公共模型
一致。中性总质量、质心和整体惯量检查通过。刚度、阻尼、参考位置、质量、摩擦、
侧滑策略、四轮固定分配和转向控制均未调整，没有修改物理引擎。

## 最终 v3 新诊断结果

八次前后悬架加载、释放衰减、上下限位试验全部通过。整车静置六秒，检查末秒；
以下取左右轮较大值，显示值作近似舍入，判定使用原始完整精度。

| 步长与驱动 | 前轮最大压缩偏差 | 后轮最大压缩偏差 | 最大轴向力平衡误差 |
| --- | ---: | ---: | ---: |
| 1 ms，开启对照 | 0.994 mm | 1.072 mm | 10.73% 以下 |
| 0.25 ms，开启第一次 | 0.2474 mm 以下 | 0.2661 mm 以下 | 2.661% 以下 |
| 0.25 ms，开启第二次 | 0.2474 mm 以下 | 0.2661 mm 以下 | 2.661% 以下 |
| 0.25 ms，关闭 | 0.2474 mm 以下 | 0.2661 mm 以下 | 2.661% 以下 |

精度模式每轮最大偏差 ≤0.3 mm、连续轴向平衡误差 ≤5%；两次开启驱动的平均原生
关节位置差均为 0，满足 ≤0.01 mm。各轮静置空接触比例为 0%，实际车体速度均
低于 1 mm/s。驱动关闭与开启的平均位置差最大约 0.000000610 mm。

原生加载参数、关节位置／速度、父子链接位姿及传递力共同支持有限步长解释：

`F_axis = k*(q-ref) + (c*h+k*h*h)*(a_post-delta_v_constraint/h)`。

原生状态和力是测量值，式中重建力为计算值。0.25 ms 降低了隐式弹簧／阻尼与接触
速度修正耦合造成的偏差，没有通过改变弹簧或摩擦来压小误差。

![静态精度](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-acceptance-v3-20261015/runtime/delivery/static-precision.png)

## 已合格九轮的工程结果

主步长均为 0.25 ms。位移相对于运动开始时各自的位置，实际转角来自姿态展开。
空接触比例取稳定区间中最差车轮。空接触记录本身不等同于已经证明该轮几何悬空。

| 场景／支撑／方向 | 参考点最大位移 | 质心最大位移 | 实际转角 | 最差轮空接触 |
| --- | ---: | ---: | ---: | ---: |
| 曲面／刚性／左 | 167.204 mm | 153.737 mm | 198.208° | 63.149% |
| 曲面／刚性／右 | 182.205 mm | 168.475 mm | −198.226° | 62.130% |
| 曲面／悬架／左 | 86.935 mm | 74.211 mm | 193.052° | 0.04063% |
| 曲面／悬架／右 | 84.434 mm | 71.852 mm | −192.828° | 0.02813% |
| 平地／悬架／左 | 0.1792 mm | 12.477 mm | 205.989° | 0% |
| 平地／悬架／右 | 0.1792 mm | 12.477 mm | −205.989° | 0% |
| 斜面／悬架／左 | 79.925 mm | 67.800 mm | 207.328° | 0.01094% |
| 斜面／悬架／右 | 86.428 mm | 75.795 mm | −207.628° | 0.01407% |
| 斜面法向重力／悬架／左 | 0.5533 mm | 12.685 mm | 207.044° | 0.00938% |

两组曲面刚性／悬架配对的逐步输入和四轮目标完全一致。在共同实际转角范围比较：

| 共同转角 | 刚性参考点最大位移 | 悬架参考点最大位移 | 减少比例 |
| --- | ---: | ---: | ---: |
| 左转 193.052° | 166.467 mm | 86.935 mm | 约 47.8% |
| 右转 192.828° | 182.205 mm | 84.434 mm | 约 53.7% |

共同转角曲线使用累计实际转角首次到达位置插值为 2001 点；输入一致性仍核对全部
物理步。这支持支撑改善对偏移有贡献，不能将全部剩余位移归为支撑模型缺陷。

![共同实际左转角轨迹](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-acceptance-v3-20261015/runtime/delivery/common-angle-01-03.png)

已合格悬架轮次的完整行程范围为 **−8.4745～+6.5971 mm**，限位占比均为 0，
行程内检查全部通过；曲面稳定支撑指标通过。平地两轮静置速度和空接触指标通过。
九轮停车末秒实际速度最大约 **0.000241 mm/s**，低于 1 mm/s。

曲面左转四轮平均竖直接触载荷依次为前左 3.547 N、前右 3.622 N、后左 3.877 N、
后右 3.835 N。所有轮次的逐轮载荷、完整／稳定行程极值、限位比例保存在分析 JSON。

曲面左转参考点轨迹的世界坐标圆拟合中心由刚性的 (−66.304, −59.506) mm 变为
悬架的 (−38.286, −22.598) mm，径向残差 RMS 分别约 2.290／1.275 mm；瞬时转向
中心中位数分别为 (−57.683, −48.955)／(−34.308, −19.757) mm。它们是轨迹诊断估计，
不意味着坡面上存在一个固定不动的转向中心。原 10 mm 中心漂移筛查仍保留失败记录，
没有将该旧筛查改成此次“偏移归零”的验收目标。

曲面悬架左右转的参考点下坡端点移动约 86.934／84.137 mm，斜面为 79.900／
86.290 mm。相同斜面左转将重力改到法向后，参考点最大位移降到约 0.553 mm，
支持沿坡重力对剩余移动有贡献；右转法向重力轮失败，不能声称本批完成了双向因果验证。
平地参考点仅移动约 0.179 mm，而质心约 12.477 mm，需结合质心相对参考点的约
6.24 mm 初始偏置随转向旋转解释，不能把两个指标互换。

## 本批停止的确切原因与剩余项目

第 10 轮：`prepared-10-plane_normal-suspended--0.2-0.00025`。
`Lifecycle` 在 DDS 所有权清单读取中记录 PID 5775 的
`PermissionError(13, 'Permission denied')`，健康检查按失败关闭原则停止。
仿真结束时间为 0，调度阶段为空，物理步 CSV 只有表头。

现有错误记录没有失败的具体 `/proc` 操作／路径、PID 启动身份／当时状态和堆栈。
它不足以证明错误属于进程正常退出的读取竞争。补充的无 ROS、无 Gazebo 探针执行
413 次清单扫描、启动并回收 800 个短命子进程，未复现该错误；该阴性结果不证明根因。

异常收尾的零速度发送、暂停应答和进程清理均通过，所有所属进程（包括僵尸）残留为零；
242 条事件完成 flush、关闭及 ID 核对。无物理步导致原生观察器完整采集资格未满足，
原失败判定及其 SHA-256 保留，未进行补判或自动重试。

因此本批 **0.5／0.25 ms 收敛四轮、刚性恢复两轮、平地直行一轮均未启动**。
整体比较、恢复、收敛和完整矩阵门槛为不通过。下一前置工作是补齐 DDS 清单读取的
操作／路径、堆栈、PID 启动身份、状态与角色证据，在无物理步进条件下复现并回归。
在这一见证失败解释清楚前，停止新增物理批次，不吞掉活动进程权限错误或未解析所有权。

## 独立批次、参数和交付

| 本次请求中的批次 | 新诊断启动 | 正式启动／资格合格 | 停止原因 |
| --- | ---: | ---: | --- |
| v1 | 12 | 1／0 | 默认 TF 20 Hz 产生 100.5 ms 间隔 |
| v2 | 12 | 17／16 | 末轮直行错误读取旧坡道高度图 |
| v3（最终） | 12 | 10／9 | DDS 清单读取权限错误，尚未步进 |

合计 36 次诊断、28 次正式启动，分别独立冻结和计数。v2 的收敛、刚性恢复及静态
结果仍是原失败批次中的补充证据，没有计入 v3。更早的 2026-10-14 记录原样保留。
最终 v3 记录了 270,000 个诊断步和 1,015,200 个合格正式物理步。

未标定参数沿用批准版本：每轮原 0.4 kg 按 0.05 kg 支座＋0.35 kg 旋转轮拆分，
惯量同比拆分；中性总质量 9.186 kg、簧载质量 7.586 kg。前／后每轮静载约
2.924515／3.220145 N，以 `k=F/0.01` 得 292.4515／322.0145 N/m，
以 `c=2*0.7*sqrt(k*(F/1.62))` 得 32.1680／35.4198 N·s/m。无载参考位置
−10 mm，行程 ±20 mm，月球重力 1.62 m/s²。没有为减小转向偏移再次标定这些参数。

交付包含候选 URDF／SDF、刚性对照、参数推导、精度配置、源码和网格冻结、运行库哈希、
全部原始记录、失败记录、输入／实际转角对照及绘图后处理源码。精度验收范围为
0.25 ms；参数未经实车标定，模型未经实车验证，生产默认不切换。

最终证据清单含 1,306 个文件、12,151,200,259 字节，601 份去重源码归档的内容哈希
核验通过，后处理源码／输入哈希和失败判定保留检查通过。两个结果图已检查可读性。
所有本次专用容器均已停止；最终容器状态为 exited、PID 0，取证文件保留。

- [最终交付汇总](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-acceptance-v3-20261015/runtime/delivery/summary.json)
- [失败诊断及原判定哈希](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-acceptance-v3-20261015/runtime/failure-diagnosis.json)
- [静态门槛](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-acceptance-v3-20261015/runtime/diagnostic/gate.json)
- [逐轮工程分析](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-acceptance-v3-20261015/runtime/formal-analysis/summary.json)
- [源码索引](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-acceptance-v3-20261015/runtime/delivery/source-index.json)
- [原始证据 SHA-256 清单](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-acceptance-v3-20261015/runtime/delivery/raw-evidence-inventory.json)
- [冻结与进程清理审计](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-acceptance-v3-20261015/runtime/final-audit.json)
- [交付文件核验](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-acceptance-v3-20261015/runtime/delivery-qa.json)
- [最终容器状态](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-acceptance-v3-20261015/runtime/container-final-state.json)
- [v2 停止批次](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-acceptance-v2-20261015/README.md)

## 后续查因补充

后续无物理步进诊断已稳定复现：进度查询的 `docker exec` 创建短暂的 `runc init`，
全 PID 清单读取其文件描述符时遭到权限拒绝。原观察器三次 exec 注入全部失败，
三次宿主文件读取对照全部通过。原 PID 5775 的程序身份仍无法从旧记录恢复；原轮
失败判定和本报告的未通过结论保持不变。详见
[DDS 权限错误查因报告](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/R5_DDS_PERMISSION_CAUSE_20261015.md)。
