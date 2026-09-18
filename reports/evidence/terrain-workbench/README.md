# 三维地图与地形寻路验收证据

更新于 2026-09-10。软件基线 db1851e，实现位于 MoonCar-mapping，未提交。

## 优先阅读

- `runtime-comparison.json`：重置同一世界、相同初始位姿 (3.5,3,0)、目标 (4.875,3.125)、同一档案与控制器，fixed/full 各执行 1 次。均到达；规划长度都为 1.4268 m，真值误差 0.0880 / 0.0902 m。不能证明算法优越性。
- `gazebo-survey/recording-check.json` 与 NPZ/PLY：181 帧，40278 个体素点，1722 个有效高程格；使用真实 Gazebo 深度与 TF。采集阶段使用既有二维地图上的 AMCL/融合定位。
- `slam-observations/`：首份 SLAM 运行中的采集档案及日志，37 帧、6010 个体素点。与后续巡航观测分开保留。
- `software-final.log`：8 个 ROS 包、35 项 Python 与 C11 控制核心通过。`source-sha256.txt` 是构建时源文件指纹，包含当时的文档版本；后续文档修订不改变已运行代码。
- `m6-01.json`：原 M6 实跑回归，目标真值误差 0.07578 / 0.08338 m，传感器、禁行区及新障碍检查通过。

## 执行与失败记录

`navigation-ui-01-check.json` 验证真实浏览器目标执行，到达误差 0.08288 m。
`navigation-cancel-01-check.json` 和 `navigation-disconnect-01-check.json` 分别验证用户取消和页面刷新丢失租约后的停车。
这些浏览器测试在加入规划跟踪余量之前运行；其后相同取消/租约代码的完整软件回归通过，fixed/full 运行使用最终余量。

`navigation-replan-07.json` 的绕行失败：规划贴近障碍边界，跟踪偏差触发保护；独立检查确认停止。
后续增加每侧 0.15 m 规划余量。`navigation-replan-08.json` 和 `navigation-replan-09.json` 均重规划后无路径。
09 的独立 check 验证终止后 16 个零安全指令。它的 `passed=false` 仍表示“成功动态绕行”不通过；独立 `check.passed=true` 只表示安全停止符合预期。
较早调试失败保留于 `tmp/ros_acceptance/navigation-replan-01..06.json`；其中 02 受重复 Gazebo 进程影响，不能用于算法性能结论。

## 数学地形对照

`synthetic-benchmark-final.json` 为最终版本 5 类地形 × 5 模式，共 25 组。
另外两个 synthetic-benchmark 文件是调试期间旧版本，最终分析不混用。
固定/full 使用相同状态与安全约束；低支持度区域选择不同路线。未知横断区域与 0.4 m 台阶均无路径。
`predicted_max_pitch/roll` 是几何预测，`localized_track_length_m` 是定位轨迹距离，均不能冒充真实 IMU 或车轮测量。
不同定义的 cost 不能直接跨模式比较；路径变长也不是更安全的证明。

## 图片及复验

`gazebo-observed-map.png` 为本轮 Three.js 导出的观测图；截图时仍在采集，不等于最终 181 帧视图。
运行方法见 `containers/README.md`、`web/README.md`；使用原始档案可在本地重载及回放。
原 M6 的场景、人工禁行区、NavFn 与本轮地形搜索属于不同层次，不能把 M6 重规划通过移记为本轮成功绕行。
不包含实车、三维 SLAM 闭环校正、地形动态调速或学术原创性认证。
