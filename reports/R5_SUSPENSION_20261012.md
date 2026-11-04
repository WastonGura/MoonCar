# R5 独立悬架实现与阶段验证（2026-10-12）

候选模型和验证工具已实现。**完整工程验收未通过，暂不能标为“支撑模型修复候选”。** 冻结矩阵共启动 13/17 轮，前 12 轮数据资格通过，第 13 轮主动中断，后 4 轮未运行。停止原因是场景准备错误：最后一轮“平地直行”会驶入坡道，违反平地验收条件。发现后立即停止，保留原批次及失败证据；没有修改冻结输入后续跑，也没有把新准备的平面世界计入旧结果。

已有结果支持继续验证这一模型：曲面左右转参考点最大偏移由 171.74 / 179.41 mm 降至 92.34 / 88.22 mm，稳定区间各轮空接触比例降至 0.019%～0.056%。同一输入下实际转角也发生变化，必须连同位移一起看。生产悬架默认仍 `rigid`，转向默认及发布批准状态保持原状；没有位置保持、动态轮速补偿或实车验证声明。

## 实现及参数推导

R5 Xacro 增加 `suspension_profile`：默认 `rigid`，研究参数为 `independent_linear_v1`，未知值直接拒绝。连接顺序为 `base_footprint → *_wheel_carrier_link → *_wheel_link`；新增四个 `*_wheel_suspension_joint` 为沿车体 Z 轴的被动位移关节。原四个 `*_wheel_joint` 名称、旋转轴、轮胎碰撞及驱动插件保持，候选 JointStatePublisher 额外发布四个悬架关节，供 TF 和诊断使用。支座没有额外碰撞体。

行程 ±20 mm、无载参考 −10 mm、阻尼比 0.7。原每轮 0.4 kg 拆为支座 0.05 kg、旋转轮 0.35 kg，惯量按 1/8、7/8 拆分。中性位置总质量 9.186 kg，质心 (−0.0062378892, 0, 0.1279833170) m；总惯量主对角为 (0.0962440105, 0.1203805029, 0.1662453190) kg·m²，xz 项 0.0046113985。修改前后质量、质心和完整惯量一致，误差处于浮点舍入范围。R5 默认与旧模型展开 URDF 分别与修改前逐字节相同。

簧载质量 M=9.186−4×0.4=7.586 kg；簧载关于车体原点的纵向一阶矩 P=−0.05730125 kg·m。前后轮中心为 x=±a，a=0.157 m，左右对称。由静载与力矩平衡得到：

\[
F_\text{front}=1.62\left(\frac M4+\frac P{4a}\right),\quad
F_\text{rear}=1.62\left(\frac M4-\frac P{4a}\right)
\]

\[
k_i=F_i/0.01,\quad c_i=2\times0.7\sqrt{k_i(F_i/1.62)}
\]

| 每轮参数 | 前轮 | 后轮 |
| --- | ---: | ---: |
| 簧载静载 F (N) | 2.924514674 | 3.220145326 |
| 刚度 k (N/m) | 292.4514674 | 322.0145326 |
| 阻尼 c (N·s/m) | 32.16802023 | 35.41979150 |

参数由冻结模型独立推导，并与公共 Xacro 转换后的 SDF 参数核对。没有扫描刚度、阻尼、行程或摩擦以追求更小位移。全部参数登记为未标定研究假设；改变车体质量或安装位置后必须重新推导，不能照搬。

启用模型参数的展开方式：

```bash
xacro $(ros2 pkg prefix mooncar_description)/share/mooncar_description/urdf/mooncar_r5.urdf.xacro suspension_profile:=independent_linear_v1 > /tmp/r5-suspended.urdf
gz sdf -p /tmp/r5-suspended.urdf > /tmp/r5-suspended.sdf
```

本轮实验采用历史 D=0、固定前后轴轮速分配的独立夹具。公共模型只增加悬架选择参数；该实验夹具的驱动配置没有变成生产默认。

## 基础力学与采集资格

实际加载的 DART 库为 `/opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so`，SHA256 `cfe9b7c6bf72c204645df81f1be7bc646ffbd342acea52c0684a1ad24848274c`。核验进程映射及哈希后，三项原生单关节试验分别运行 6,000 步：

| 试验 | 理论终值 | 实际终值 | 结果 |
| --- | ---: | ---: | --- |
| 2 kg、1.62 m/s²、k=324 N/m 加载 | −10 mm | −9.999999999999988 mm | 压缩误差远低于 5%，方向正确 |
| 零重力、参考位置 +10 mm 的释放衰减 | +10 mm | +9.999999999999988 mm | 误差包络从约 10 mm 衰减至浮点精度 |
| 十倍重力过载 | 下限 −20 mm | −20.00000000001273 mm | 行程限制执行，误差为数值舍入 |

整车平坦起点零命令试验沿用完整采集流程。静置末秒最大车体速度为 0，稳定区间各轮空接触比例为 0；悬架未撞限位，ROS 关节记录包含原 6 个及新增 4 个关节。平均轮胎竖直接触载荷：前轮各 3.572204 N，后轮各 3.868456 N，总计 14.88132 N，与整车月球重力一致。

整车平地静态悬架位置仍有一个未解释偏差：简化静载预测 q≈0，实际前轮约 +0.993 mm、后轮 +1.073 mm。原生单关节验证通过不等于整车所有静力关系均已解释；该偏差保留为待查问题，未通过调参数消除。

每个合格矩阵轮次均经过真实 ROS、安全中继、桥接及 DiffDrive，沿用预热 0.2 秒、静置 5 秒、运动 20 秒、停车 3 秒。逐步物理、接触、原生输入、固定轮速分配、库加载、实际世界、传感器图片文件及回执、尾部 flush/排空均核验。左右刚性/悬架配对的物理输入和四轮目标逐步完全相同。诊断观察器只读状态，不施加力或控制命令。停车和零输入尾部门槛在已完成轮次通过。

旧原始夹具对零命令仍保留“一米前进”门槛失败字段。本轮另用零输入、停车、静置、完整采集资格判断静置试验，没有改写旧结果或把距离失败隐藏。

## 冻结矩阵结果

参考点为 `base_footprint`；以下是 20 秒输入窗口内的最大水平位移。实际转角由姿态展开得到，正为左转。稳定接触统计窗口为运动开始后第 3 秒至结束前第 1 秒。轮胎空接触统计使用显式空记录，缺失记录不当作零载荷。

| 轮次 | 场景 / 模型 | 步长 (ms) | 最大参考点偏移 (mm) | 最大质心位移 (mm) | 实际转角 (°) |
| --- | --- | ---: | ---: | ---: | ---: |
| 01 | 原曲面 / 刚性左转 | 1 | 171.735 | 158.208 | +198.197 |
| 02 | 原曲面 / 刚性右转 | 1 | 179.407 | 165.936 | −198.807 |
| 03 | 原曲面 / 悬架左转 | 1 | 92.339 | 79.522 | +192.982 |
| 04 | 原曲面 / 悬架右转 | 1 | 88.215 | 75.625 | −192.889 |
| 05 | 平坦起点 / 悬架左转 | 1 | 0.196 | 12.497 | +205.985 |
| 06 | 平坦起点 / 悬架右转 | 1 | 0.196 | 12.497 | −205.985 |
| 07 | 同坡度斜面 / 悬架左转 | 1 | 80.234 | 68.061 | +207.475 |
| 08 | 同坡度斜面 / 悬架右转 | 1 | 87.169 | 76.449 | −207.838 |
| 09 | 斜面法向重力 / 悬架左转 | 1 | 0.518 | 12.669 | +207.024 |
| 10 | 斜面法向重力 / 悬架右转 | 1 | 0.709 | 12.698 | −207.170 |
| 11 | 原曲面 / 刚性左转 | 0.5 | 168.344 | 154.864 | +198.205 |
| 12 | 原曲面 / 刚性左转 | 0.25 | 167.204 | 153.737 | +198.208 |
| 13 | 原曲面 / 悬架左转 | 0.5 | 中断，不计入 | — | — |
| 14 | 原曲面 / 悬架左转 | 0.25 | 未运行 | — | — |
| 15–16 | 恢复刚性曲面左右转 | 1 | 未运行 | — | — |
| 17 | 悬架平地直行 | 1 | 场景资格失败，未运行 | — | — |

05/06 是坡道世界原点附近的平坦区域原地转向，不能扩展成整段直行的平地资格。世界高程数据表明中心路线 x=0～0.75 m 为零高程，但到 x=2 m 已升高 40.48 mm；这就是最后一轮准备错误的直接证据。

质心位移也包含车体旋转造成的几何运动。平地参考点仅移动约 0.20 mm，质心仍可移动约 12.50 mm，因为质心距转向参考点约 6.24 mm；不能把这部分误判为整车下坡漂移。

### 支撑、限位与接触几何

| 模型 / 方向 | 前左空接触 | 前右空接触 | 后左空接触 | 后右空接触 |
| --- | ---: | ---: | ---: | ---: |
| 刚性左转 | 63.052% | 37.035% | 0.769% | 5.331% |
| 刚性右转 | 38.979% | 61.096% | 18.468% | 0.794% |
| 悬架左转 | 0.056% | 0.031% | 0.019% | 0.031% |
| 悬架右转 | 0.031% | 0.025% | 0.025% | 0.019% |

已完成的悬架曲面轮次满足各轮空接触不超过 5% 的工程目标。稳定区间四轮行程合计范围：左转 −0.760～+3.293 mm，右转 −0.672～+3.063 mm；限位占比均为零，完整采集区间也未越界。平均竖直接触载荷 (前左、前右、后左、后右) 为：左转 (3.553, 3.616, 3.876, 3.836) N，右转 (3.626, 3.577, 3.799, 3.880) N。刚性左转对应 (2.597, 4.547, 4.927, 2.810) N，支撑显著偏斜。

另对四个曲面轮次抽样进行原生 ODE 轮胎圆柱与实际三类三角网格的碰撞重放，共 518 组轮胎位姿（其中 198 组显式空接触），每组比较前一物理步和本步更新后两种位姿，未增加仿真轮次。**前一物理步的全部 518 组接触存在性与记录一致**，本步更新后有 62 组不一致。结果与接触在步内求解、随后位姿更新的时序一致；不应把 post-step 位姿直接配给本步接触判空。在这些抽样范围内，空记录对应实际轮胎碰撞几何无交叠，不能归因于 ROS 漏消息。此核验只证明抽样接触存在性，不能替代间隙精度、接触力或实车参数标定。

### 转向中心与下坡移动

| 模型 / 方向 | 输入末端参考点 ΔX / ΔY (mm) | 圈拟合中心世界 X / Y (mm) | 瞬时中心中位数世界 X / Y (mm) |
| --- | --- | --- | --- |
| 刚性左转 | −164.285 / −50.035 | −68.833 / −62.796 | −63.975 / −47.370 |
| 刚性右转 | −179.199 / −3.652 | −90.888 / +26.024 | −97.573 / +17.043 |
| 悬架左转 | −90.716 / −17.233 | −40.310 / −26.739 | −35.233 / −20.948 |
| 悬架右转 | −87.535 / −10.933 | −44.840 / +7.938 | −42.790 / +3.378 |

中心是轨迹拟合及瞬时运动的统计量，并非固定机械转轴。两方向主要仍沿参考下坡方向 −X 移动。均匀斜面左转去掉沿坡重力后，偏移由 80.234 mm 降为 0.518 mm，实际转角由 207.475° 变为 207.024°，说明重力作用仍然存在。悬架改善四轮支撑并不自动抵消坡面滑移；历史刚性均匀斜面左转偏移约 67.87 mm，候选在该对照上也没有全面变小。

### 步长、恢复与判定边界

刚性模型 0.5 与 0.25 ms 的最大偏移差 1.140 mm，小于容许值 8.417 mm；转角差约 0.00166%，通过本项要求。悬架相应两轮未完成，行程极值差及悬架轨迹收敛不能判定。末尾恢复两轮及平地直行未执行，不能借用历史恢复结果替代本轮验收。

因此总体 `engineering_targets_passed=false`。模型目前状态为**被动独立悬架研究实现，等待完整验证**；尚未获得“支撑模型修复候选”资格。剩余坡面移动登记为待校验的物理响应，不能宣称 17 cm 偏移已经彻底修复。

## 停止记录、准备器修正与交付

发现第 17 轮场景资格错误时，第 13 轮正在执行，立即中断该轮；原始 loader、部分数据及采集失败都保留，未把部分轨迹算作成功结果。冻结至停止时源码和实际库签名没有变化。之后才修正准备器：将地面网格顶点投影为平面，保留三角形拓扑及材料摩擦分区；启动前检查平面、单位变换、整段输入路线覆盖和走廊障碍物。原错误夹具现在在仿真启动前被拒绝。

新平面世界和完整 17 轮声明已准备到 `runtime/prepared-next-flat`，**仅完成准备，没有运行新的矩阵或恢复旧批次**。旧准备器、旧执行器和旧测试源码的快照与冻结 SHA256 一致；停止后三个文件改动另行登记。46 项相关回归（悬架/场景 9、转向侧滑 8、轴分配 9、转向资格 4、原点诊断 16）及 10 个 Python AST 检查通过。观察器按 C++17 和 `-Wall -Wextra -Werror` 编译。

前置准备及静置尝试中的失败同样保留：首个单关节世界重名导致无法加载；静置夹具不适用前进门槛、观察器/控制输出文件冲突；缺少新增关节发布时 runner 的前置拒绝。后续分别另建目录验证，没有覆盖失败。悬架常数在这些修正中没有变化。

本轮独立容器已停止，最终状态 `Running=false`；没有停止其他工作容器。原始证据约 5.37 GiB 保存在本地，未提交 Git。保留仓库原有未提交内容。本轮没有提交、推送或发布。

主要文件与证据：

- 模型：[r5_suspension.xacro](D:/Documents/Projects/Repositories/MoonCar-mapping/ros2_ws/src/mooncar_description/urdf/common/r5_suspension.xacro)、[R5 入口](D:/Documents/Projects/Repositories/MoonCar-mapping/ros2_ws/src/mooncar_description/urdf/mooncar_r5.urdf.xacro)。
- 参数与转换：[r5_suspension.py](D:/Documents/Projects/Repositories/MoonCar-mapping/scripts/r5_suspension.py)、[准备器](D:/Documents/Projects/Repositories/MoonCar-mapping/scripts/prepare_r5_suspension.py)、[平地资格检查](D:/Documents/Projects/Repositories/MoonCar-mapping/scripts/r5_flat_scene.py)。
- 汇总：[SUMMARY.json](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-20261012/SUMMARY.json)、[协议](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-20261012/PROTOCOL.md)。
- 原始账本：[campaign.json](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-20261012/runtime/campaign/campaign.json)、[冻结签名](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-20261012/runtime/campaign/freeze.json)、[停止根因](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-20261012/runtime/campaign/external-scene-qualification-failure.json)。
- 基础力学：[bench-v2/result.json](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-20261012/runtime/bench-v2/result.json)、[静置汇总](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-20261012/runtime/static-complete-analysis/summary.json)。
- 几何重放：[support-replay/result.json](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-20261012/runtime/support-replay/result.json)；每轮目录保留 world、physics/contact/suspension CSV、传感器记录、图片及资格文件。
- 回归及收尾：[regression-final.log](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-20261012/runtime/regression-final.log)、[verification-final.json](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-20261012/runtime/verification-final.json)、[container-final.json](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-20261012/container-final.json)。

![已完成轮次的偏移、曲面轨迹及悬架行程](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-suspension-20261012/suspension-comparison.png)
