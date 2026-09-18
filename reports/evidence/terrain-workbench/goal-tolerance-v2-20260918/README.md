# 共用导航到点判定诊断（2026-09-18）

正式 300 仿真秒对照的首个 `survey` 起点在前三轮发现共同导航问题，和三策略分数无关：`nearest_view_v2` 一次、`coverage_view_v2` 两次导航动作驶近目标后仍绕行，最终被现有停滞保护取消。三次失败目标分别为 `(-9.604,-17.170)`、`(-12.424,-15.700)`、`(-16.088,-20.255)`；动作与目标、原因记录在各轮 `actions.csv`。后两次所对应的保存导航栅格中，目标和当时车位的中心格以及 0.65 m 净空圆均无代价 ≥99 的障碍，Nav2 和传感器仍在发布。`coverage_view_v2` 两次失败动作的逐秒位姿距目标最小值为 0.180 m 和 0.240 m，`nearest_view_v2` 的失败动作为 0.199 m；三段记录均进入 0.25 m，均未进入 0.15 m，随后在目标附近摆动直至停滞保护取消。

运行中的 `/controller_server` 参数实测 `goal_checker.xy_goal_tolerance=0.15`，来源是共用 `terrain_navigation.launch.py` 的工作台配置。Nav2 Jazzy 的 [`PositionGoalChecker`](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_controller/plugins/position_goal_checker.cpp) 只按 XY 距离判到点，默认半径为 0.25 m。现把工作台共用值从 0.15 m 恢复为 0.25 m；策略、预算、评价算法和门槛未变。这是修正已复现的到点失效，不是按分数调整某个策略。单独的导航复现记录放在 `tmp/ros_acceptance/goal-tolerance-v2-20260918/`，通过后再登记新的正式对照版本。

旧正式批次 `tmp/ros_acceptance/formal-strategy-v2-300s-20260918/` 原样保留，不纳入修复后统计。其前三轮完整结果仍可作诊断：就近/覆盖/补采的 S 分别为 56.52/86.58/70.87，均未通过非平地门槛；第 4 轮在执行中被人为中断，已捕获 247 帧原始数据但没有评价结果，不能作为完整轮。为避免后续轮次继续使用有缺陷的共同导航，已向比较器进程发送 SIGINT：`survey-exit-code.txt=130`；续跑监护据此停机并写 `campaign-exit-code.txt=1`。之后经 API 停止自主任务与仿真，没有清理上述档案。修复后的实验必须重新登记并重跑受影响的完整同条件矩阵，不和旧版本混算。

独立复测使用同一 `survey` 场景，从 `(-15,-16,0)` 直接导航至旧失败目标 `(-12.424,-15.700)`。`attempt-02` 的运行参数实测 `goal_checker.xy_goal_tolerance=0.25`，Nav2 返回 `succeeded`，终点距目标 0.2326 m，恰好落在旧 0.15 m 半径外、新 0.25 m 半径内；未触发停滞保护。`trace.json` 保留 63 次状态采样，`result.json` 与对应 `experiment-747848374802/simulation.log` 可复核。`attempt-01` 在发送运动请求前因诊断脚本覆盖 ROS Python 路径而中止，日志原样保留，不能算导航失败。这是一条目标复测，证明该失效模式已有针对性修复；跨场景稳定性仍需正式重复实验检验。
