# 观察位净空修复后三策略重验证（2026-09-18）

本批因共同安全缺陷而重新运行，原批 `strategy-v2-shared-fix-small-batch-20260918` 保留且不混算。先前补采轮已导航到观察点，但原地转向因 `Collision Ahead` 被 Nav2 中止；存档导航图显示候选车体净空圆内有代价 99，而候选检查只拒绝 ≥100。[Nav2 Jazzy 栅格发布代码](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/src/costmap_2d_publisher.cpp)定义 99 为内接障碍。共同候选检查的普通和边界兜底入口均改为拒绝 ≥99；定向用例先失败后通过，地形测试 78 项通过。没有针对某策略的分数、阈值或评分模型修改。

运行前协议：`tmp/ros_acceptance/strategy-v2-clearance-small-batch-20260918-protocol.json`。条件仍为 `visible_curve_v1`、起点 `(6.5,-2,2.4)`、8 m 半径、每轮 300 仿真秒；就近、覆盖、地表补采各运行一次，分别新建 Gazebo、空白 SLAM 和原始采集。三轮的在线源码哈希均为 `2af83cef756a854c457acc8e9f9b1c90081f1d6f5839adf0260e7f8c8ef4fd10`，场景哈希均为 `f9a90165b21f47207c08d7c4ef55072e472cf0de1d4eb7d3004b74801beaf68a`；协议中各文件哈希与现有源码回读一致。固定评价 `mapping-acceptance-v1.1`、分母和 5 cm/5°、总体及两分层 C≥80%、P≥90% 暂定仿真门槛均未改。

| 策略 | 任务结束 | 动作失败 | 路程 | 总体 C | 总体 P | S | 非平地 C / P | 评价状态 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `nearest_view_v2` | 295.6 s，预算到达 | 0/38 | 8.78 m | 66.00% | 99.90% | 65.93 | 51.61% / 100% | 未通过覆盖门槛 |
| `coverage_view_v2` | 282.1 s，预算到达 | 0/14 | 24.86 m | 88.21% | 99.86% | 88.09 | 79.03% / 100% | 未通过非平地 C（49/62，差 1 格） |
| `surface_view_v2` | 297.4 s，预算到达 | 0/16 | 28.03 m | 87.49% | 99.78% | 87.30 | 82.26% / 96.08% | 几何门槛满足，安全未认证 |

三轮原始捕获均完成，分别用 439、394、406 帧评价；传感器来源、坐标、时间截止与预先建立的参考审计均通过，运行故障字段为空。总体原始观测 C_obs 为 75.71%、95.07%、94.94%。非平地固定 62 格中，就近/覆盖/补采有效重建 32/49/51 格，补采中 49 格几何合格。三轮安全认证字段均为 `null`；补采轮的 `geometry_pass_safety_unverified` 只表示几何门槛满足，**不表示整机或实物安全验收通过**。

在这一场景和起点下，补采的非平地 C 较高，但覆盖优先的总体 S 较高；单条件各一次不能证明稳定优势。旧批补采的转向失败未在本次重现，但单轮零失败不能证明以后绝不失败。三轮结果均保留，不择优替换旧批。下一步按事前固定的 `docs/formal_strategy_v2_300s_protocol.md` 做多场景、两起点、三重复对照，届时继续并列报告覆盖、误差、耗时、路程和失败。

核验入口：`tmp/ros_acceptance/strategy-v2-clearance-small-batch-20260918/plan.json`、`trials.json`、`captures/trial-01..03/manifest.json`、`evaluations/trial-01..03/summary.json` 和 `cells.csv`，任务档案路径见 `trials.json`。
