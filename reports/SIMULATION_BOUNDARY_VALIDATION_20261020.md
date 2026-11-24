# V2 扩展仿真验证（2026-10-20）

本轮已实际执行全场、长时间、坡面及无标柱验证。**30 分钟局部耐久通过本轮工程条件；其余三类能力未通过，仿真阶段不能标记全部验收完成。** 另发现非零起始朝向的真实坐标错误，以及工作台跨实验时钟竞态。

生产源、定位算法、物理模型和安全阈值均未修改。失败与仪器问题全部保留，没有事后旋转轨迹、缩小范围或放宽门槛形成通过。

## 结果

| 工况与证据 | 实际范围 | 结果 |
| --- | --- | --- |
| [全场](../tmp/ros_acceptance/simulation-boundary-20261020/full-field-02/analysis-mission-v2/mission-findings.json) | 22m 活动半径、1800 仿真秒预算；实际 101.8 秒、10.542m 路程，距起点最远 8.560m | **未通过。** 第 6 次动作因 `unobservable_translation` 中断，未达到外围 10m／四象限目标。任务窗 9780/9782 真值有同时间 TF；这些匹配的 XY P95 0.994cm、最大 2.877cm。较小的条件误差不代表全程可用。 |
| [30 分钟局部耐久](../tmp/ros_acceptance/simulation-boundary-20261020/endurance-01/analysis-mission/mission-findings.json) | 1800.128 仿真秒、161.214m；原 Nav2 局部循环路线 | **通过本轮工程条件。** 129987/129987 已记录真值均匹配同时间 TF；XY P95 0.6843cm、最大 1.8941cm；工作窗无定位故障增量，采集持续。 |
| [坡脚导航](../tmp/ros_acceptance/simulation-boundary-20261020/slope-01/analysis-mission/mission-findings.json) | `(-5,3,π/2)→(-5,4.5)`；实际约 0.907m，随后 Nav2 返回 208 | **未通过。** 真值倾角最高 1.393°，没有实际进入预定超限坡面；同时暴露起始朝向坐标错误，XY P95 约 92.13cm、最大 92.50cm，质量状态仍显示健康。 |
| [坡面冷启动补充](../tmp/ros_acceptance/simulation-boundary-20261020/slope-cold-start-01/analysis-mission/mission-findings.json) | 正式创建检查通过的 `(-4.2,-7.8,π/2)`，约 30 仿真秒 | **坡上定位未通过。** 真实稳态倾角约 1.842°，主窗 2018 份真值全部缺 TF，无采集档案。超限扫描被拒绝、安全速度为零，支持边界保护有效。 |
| [原生无标柱 V2](../tmp/ros_acceptance/simulation-boundary-20261020/unmarked-01/analysis-mission/mission-findings.json) | 原点 30.34 仿真秒；实际 Scene 确认无新增标柱 | **未通过。** 主窗 2551 份真值全部缺 TF；全包 328 份原始扫描和 328 份水平扫描的有效回波均为 0。没有合格扫描、滤波定位或地图档案；正常导航请求被拒绝，安全速度均为零。 |

全场首次拒绝时车体倾角未超过 1.5°，首因是平移约束不可观测；后续 `observation_stale` 是没有新合格测量的表现。首个故障状态之后记录到的 160 条安全速度均为零。不能把此次全场失败解释成坡度过滤，也不能把停车保护通过改写为全场能力通过。

## 30 分钟耐久的范围与数据

路线为 `(1,0)→(1,1)→(0,1)→(0,0)`，经原工作台许可、Nav2、定位联锁和速度中继反复执行，包含正常目标间停稳。它是局部工作负载，不是 30 分钟长程自主探测。

驱动器工作窗为 `16.265–1816.393` 仿真秒，195 次导航成功、1 次因全局预算正常中断，完成 48 个四目标循环及之后的 3 个目标。按真值平移速度 >5mm/s 或转速 >0.01rad/s 的描述阈值，运动占比约 76.53%，不是静止等待凑时长。

消费者故障计数在启动屏障前已为 2，工作窗保持 2→2，同一生产者／订阅代际增量为 0。工作窗 43518 条生产者质量状态全部健康；完整 bag 的初始化、短暂过期记录仍保留，未混入工作窗或删除。

工作窗新增 1295 帧、2767458 个原始点。末帧时刻 1814.3，比工作结束早 2.093 仿真秒，不能称为终点瞬时地表精度。末端额外停车观察不计入 1800.128 秒工作时长。

| 相对工作时间（秒） | 精确 TF／真值 | XY P95／最大（cm） | 新增帧／原始点 | 定位故障增量 |
| --- | --- | --- | --- | --- |
| 0–300 | 20666/20666 | 0.6922／1.1640 | 287／612,865 | 0 |
| 300–600 | 20525/20525 | 0.6968／1.8941 | 221／471,511 | 0 |
| 600–900 | 21359/21359 | 0.6769／1.3232 | 236／504,171 | 0 |
| 900–1200 | 22379/22379 | 0.6806／1.4320 | 204／437,087 | 0 |
| 1200–1500 | 23556/23556 | 0.6929／1.4169 | 175／373,697 | 0 |
| 1500–1800.128 | 21502/21502 | 0.6696／1.3773 | 172／368,127 | 0 |

[耐久误差图（加宽标签边距）](../tmp/ros_acceptance/simulation-boundary-20261020/endurance-01/analysis-mission/localization-readable-v2.png)、[原图](../tmp/ros_acceptance/simulation-boundary-20261020/endurance-01/analysis-mission/localization.png)、[轨迹图](../tmp/ros_acceptance/simulation-boundary-20261020/endurance-01/analysis-mission/trajectory.png)、[主分析](../tmp/ros_acceptance/simulation-boundary-20261020/endurance-01/analysis-mission/analysis.json)。渲染副本使用同一窗口与原始样本，未改变数值验收结果。

## 新发现的起始朝向错误

`yaw=π/2` 的坡脚工况中，原始仿真 IMU 初始 yaw 已接近 0；direct 节点又减去配置的 `heading_origin=π/2`，重复扣除了初始朝向。静态 `survey_odom→odom` 的 +π/2 正确，`map→survey_odom` 近于单位变换，但输出 `map→base_footprint` 朝向固定比真值少 90°。匹配在错误坐标系中仍能自洽地保持健康。

最坏样本在 31.600 秒：真值 XY `(-5.377800,3.532800)`，地图 XY `(-4.466168,3.376181)`，误差 `0.924988m`。本轮保留原始 world/map 对比，未事后旋转或拟合。该问题是进入后续大范围验证前的优先修复项。

依据为两份独立只读复核及[无拟合原始复现](../tmp/ros_acceptance/simulation-boundary-20261020/yaw-issue/reproduction.json)。0° 起始朝向的局部耐久通过，不能覆盖这个 90° 工况失败。

## 坡面和无标柱的边界

原水平扫描过滤条件为车体倾角 ≤1.5°、IMU 与扫描相差 ≤0.03 秒，本轮未放宽。坡面冷启动主窗的 301 份原始扫描均对应超限 IMU，均没有同时间的水平扫描；全捕获仅在早期姿态尚未倾斜时有 3 份水平扫描，未能建立定位。保护拒绝不代表坡上定位能力具备。

无标柱使用测试设置副本加载原始 `mooncar_survey_v2.sdf`，仅取消追加四个定位设施。生成 XML 与 canonical 语义完全一致，原文件 SHA256 为 `08e9837e09b53e06fa0569423e0dc4775b1456508ca999cfef6fdf3444fb1555`。实际 Gazebo Scene 返回 80258 字节、车辆存在、标柱名称出现次数为 0，见[世界核验](../tmp/ros_acceptance/simulation-boundary-20261020/unmarked-01/world-verification.json)。

当前原点处水平激光有效回波实测为 0，质量报告 `too_few_returns`，不是回退到轮速或注入真值后获得的位置。全包 2123 条安全速度全部为零。本结论限定此原生 V2 原点配置，不推广为所有凹地或任意雷达安装方式均无法定位。

## 协议、仪器和限制

事前[协议](../tmp/ros_acceptance/simulation-boundary-20261020/protocol.json)冻结 369 份生产源；[最终核验](../tmp/ros_acceptance/simulation-boundary-20261020/verification.json)确认全部一致。镜像为 `sha256:6dc9c3677650c1f238d54ca67859e637980fe7052c64430eb699b826da26285d`，容器 4 CPU／6GiB，legacy 车型、DART 物理。全场保留 GUI；耐久及后三个工况关闭可选诊断 GUI，Ogre2 传感器和物理保持。

全部已记录真值均进入 TF 可用率分母；缺 TF 不填零误差。只比较相同时间、相同真值序号的 map 位姿，初始静态变换由真实 TF 链提供，真值仅用于离线评价。原生 SQLite bag 均正常关闭，五个有效工况的 `quick_check` 通过；已审计的扫描、时钟、质量及安全速度主题 CDR 解码错误为 0。TF、IMU、里程计等其余原始主题仍保留，不把上述结果扩写为所有主题逐条解码保证。记录完整性与任务／定位能力分开判断。

本轮运动 XY P95≤5cm 是事前工程预算。旧转末≤3cm、静止 P95≤3cm、静止波动≤1cm 规则未更改，但本轮未重新执行原固定角度的独立验收序列，不新增该分项通过声明。地表覆盖率仅描述；未认证 V2 高度／坡度重建精度、碰撞零发生、R5 或实车能力。

以下仪器和启动问题均保留：

- `full-field-01` 未开始运动：观察器把 Bool 许可订成 String；修正类型后在新目录重跑，原始无效尝试不计能力成绩。
- 暖切换期间，消费者可能在新时钟到来前绑定旧 ROS 时间并锁存 `clock_moved_backwards`。后续各例冷启后端隔离，见[问题及复现](../tmp/ros_acceptance/simulation-boundary-20261020/warm-reset-issue.md)。[候选补丁](../tmp/ros_acceptance/simulation-boundary-20261020/proposed-warm-reset/README.md)只有离线方法测试／审查，本轮未部署，也未运行真实暖切换验收脚本，不能称已修复。
- 全场首版分析混合了上游与安全速度，保留原件并另存 v2，数值定位与独立 bag 结果未变。后续分析修正了任意窗末端落在采样间隔中的完整性字段，完整原始真值实际跨越所需窗口，采样边缘差单列，没有移动窗口选取更好成绩。
- 坡面冷启动的额外 Scene 查询误与停机并行而超时，保留空文件；不声称该次运行时查询成功。场景设置／源文件和独立姿态记录仍可核对。无标柱及恢复场景的运行时 Scene 查询成功。

## 当前工作台与下一步

标准工作台已冷启动恢复为 `experiment-e8c7b9f92bc8`：V2＋四标柱、原点 yaw=0、legacy/default direct、GUI。记录快照为 `recording / ready / healthy`、故障计数 0、车辆静止、自主任务 idle，见[恢复状态](../tmp/ros_acceptance/simulation-boundary-20261020/restore-workbench.json)和[进程记录](../tmp/ros_acceptance/simulation-boundary-20261020/restored-processes.txt)。测试观察器、bag 和驾驶器均已正常关闭；服务保留供后续操作。

本轮新增 7 次 Gazebo（1 次无效尝试、5 个有效工况、1 次恢复），0 次实车；未提交或推送。下一步优先修正初始朝向约定并补非零 yaw 真值回归，再处理跨区域参考维护、坡面／无标柱感知及工作台暖切换。当前结果不支持直接把仿真阶段全部验收通过。

完整证据位于 [simulation-boundary-20261020](../tmp/ros_acceptance/simulation-boundary-20261020/)；[文件完整性清单](../tmp/ros_acceptance/simulation-boundary-20261020/integrity.json)在分析与报告闭卷后生成。
