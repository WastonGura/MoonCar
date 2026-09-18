# 自主探测：评价指标与论文数据归档

## 2026-09-14：拟定几何验收标准

见 [地形采集验收标准 v1](mapping_acceptance.md)。主分数为 S=100×达标地表面积/固定评价面积，等于有效重建覆盖率与几何达标率的乘积乘 100；高度、坡度、覆盖及安全底线另行检查。同步深度记录的离线自动评价已实现并核验，阈值仍是待确认的仿真目标。下文历史 Q、有效拟合面积及旧报告不能当作新标准的准确率或综合成绩；缺少可信参考时分数为空。新样例及复算入口见 [评价交付](../reports/evidence/terrain-workbench/mapping-acceptance-20260914/README.md)。

## 三策略对照新增字段（2026-09-13）

统一入口和算法定义见 [自主探测说明](autonomous_survey.md)。旧报告不改写；新增字段为空表示当时未记录，不能解释为零。

- `initial_valid_surface_area_m2` / `valid_surface_gain_m2`：同一固定范围内，满足单层平面拟合约束的起始面积和净增量。有效拟合不等于准确、可通行或实测通过。
- `resampling_eligible_cells`：任务开始时已观测、但体素不足或 XY 支撑不足的格子数；`resampling_recovered_cells` 是其中在任务结束时变为有效拟合的数量。
- `resampling_success_fraction`：上述数量之比；分母为零时为空。它是起始格子队列的恢复率，不是所有补采动作的成功概率，也不能将行驶途中顺带恢复全部归因于补采策略。
- 动作记录另存预计视场内的新增观测、恢复格数与启发式效用残差，用来识别预测失败。复杂表面另行分析，不将高 Q 混合边缘算作成功的单层地形。

每轮独立新建仿真、SLAM 和采集记录，并保留实际起始基线、策略版本、场景和源文件。两个起点和各两次重复属于可行性试验，暂不作统计显著性或算法优越性结论。图表只用黑白；所有失败、中断和报告缺失都进入总表。

## 日常使用

启动原有工作台，点击“从当前位置自主探测”。每次任务自动创建独立研究记录，无需另外启动录屏或手动抄数。自动停止、手动停止、导航/传感器故障都保留记录。关闭网页不会终止自主探测和后端记录；请保留服务窗口。

Windows 自动保存位置：

`D:\Document\Repositories\MoonCar-mapping\tmp\ros_acceptance\frontend-experiments\<实验编号>\research\<自主任务编号>\`

跨实验总表：

`D:\Document\Repositories\MoonCar-mapping\tmp\ros_acceptance\frontend-experiments\research-summary\runs.csv`

目录名沿用现有采集环境，不会自动清理。正式论文数据应复制完整任务目录到团队备份位置。没有自动上传云端。

停车后后台生成图表；`manifest.json` 的 `report_status: ready` 表示报告完成。`capture_incomplete` 或 `report_status: failed` 表示证据或报告存在问题，不能当作完整实验。终端突然关闭的任务保留已落盘数据，可以离线恢复汇总；不保证进程被强制终止时生成末尾截图。

## 自动保存内容

| 文件 | 用途 |
|---|---|
| README.md | 单次实验结果与限制说明 |
| manifest.json、task.json | 预算、评价范围、任务起点、算法与指标版本、证据状态 |
| source.zip、survey_yard.sdf | 包含未提交修改的代码/配置快照、实际仿真场景 |
| report_source.py | 生成此份报告的评价程序，避免更新代码后不知道原报告用哪个版本 |
| observations.npz | 原始深度关键帧、时间戳、相机及车体位姿，可重建采集结果 |
| telemetry.jsonl / .csv | 约每仿真秒采样的覆盖、质量、定位路径、运动与健康状态 |
| mission.json / actions.csv | 所有尝试动作、预测收益、实际收益、失败或中断原因 |
| scene-start.jpg、scene-*.jpg、scene-end.jpg | 起始、约每 30 仿真秒及结束时的 Gazebo 现场截图 |
| checkpoint-*.json、quality-*.json、navigation-*.json | 截图检查点、质量原始统计、导航地图与路径 |
| sampling-comparison.png / .pdf | 同一坐标范围、同一色标的开始/结束采样热力图 |
| collected-map.png / .pdf、elevation.csv | 结束时观测点云俯视图、有效单层地表坡度图与格子数据 |
| progress.png / .pdf | 覆盖、质量和定位路径随时间变化曲线 |
| summary.json、hashes.json | 机器可读结果与文件 SHA-256 校验值 |

PNG 可直接插入论文，PDF 图保留矢量绘图元素，CSV 可导入 Excel。现场截图是现有 Gazebo 窗口的异步画面，**不是与某一深度帧严格同步的截图**；检查点记录仿真时刻和画面墙钟年龄。没有画面时明确记录缺失，不用旧图冒充。

## 统一指标 v1

评价区域在任务开始时固定：以当时车位为圆心、指定半径的圆，与 50×50 米场地相交。按 0.25 米网格的**格子中心**判断是否位于区域内。面积分母不会随着探索扩大；不将已发现区域当作完整环境。

| 指标 | 定义与解释 |
|---|---|
| 评价面积 | 固定区域格子数 × 格子面积；离散面积，可能略不同于 πr² |
| 观测投影面积 | 收到过有效深度点的区域内格子数 × 格子面积；可能包含障碍表面，不是可通行地面面积 |
| 投影覆盖比例 | 观测投影面积 / 固定评价面积；不是“未知世界已完成百分比” |
| 采样充分度 Q | 每格 4×4 子格覆盖率 C，8 个方位箱的已观测方向数 V；Q=C×(0.7+0.3×min(V/3,1)) |
| 质量加权面积 | ΣQ × 格子面积；圈外点不计入，重复视角不会无限增益 |
| 充分采样面积 | Q≥0.8 的格子面积；0.8 是 v1 的工程阈值，未经实物标定，不代表 80% 准确率 |
| 新增成果 | 结束指标减去任务开始基线；不会把已有地图算作本轮新采集 |
| 有效地表面积 | 能拟合为有效单层地表的格子面积，与观测投影面积分开 |
| 时间与路径 | 仿真秒、预算、约每仿真秒定位采样累计的平面路径长度；路径包含定位漂移，可能漏计采样间曲线，不是真实轮速里程 |
| 执行情况 | 尝试动作数、未成功动作数、结束原因；中断与失败分别记录，不删除不利样本 |
| 高度/坡度误差 | 只对有效且有对应参考真值的格子计算高度 RMSE（米）、坡度 MAE（度），同时报告匹配格数 |

采样指标来自已发布深度统计，通常滞后当前仿真时钟。每行保存 `quality_stamp`，最终地图重建截断到最后一次评价使用的观测时刻。`observation_lag_at_end_seconds` 可能因为关键帧筛选、停车不产生新帧而较大，不应直接解释成传感器故障。

原始档案可能包含任务开始前的历史帧。`started`、`elapsed_seconds`、开始质量快照和遥测首行共同给出本轮边界。连续从同一车位再次启动是已有地图基础上的续采，**不是重新开始的独立对照实验**。

## 如何比较预测与实际收益

每个候选保存预计可见格子 `visible_cells`。预期新增格子按 Q=1 估计收益，已观测格子的潜在补采按 0.4×(1−Q) 估计；它们共同构成 `expected_quality_gain_m2`。这是启发式预估，没有概率校准。

动作记录同时包含：

- `roi_gain`：整个固定区域内的新增投影面积、新格子的质量面积与老格子的补采质量增量。
- `predicted_view_gain`：只在候选预计可见格子内统计同样的实际增量。
- `prediction_residual_m2`：候选格内实际质量增量减去预期质量增量。量纲与空间范围一致，才可用于分析高估/低估。

实际增量包含执行动作途中及末尾观察阶段在这些格子内得到的数据；不是仅在最终姿态拍摄的单帧收益。未预测到但沿途采到的其他格子只体现在 `roi_gain`。旧版无 `metrics_version` 的动作统计全档案，不能与 v1 直接混用。被中断的动作不应与完整执行动作直接混合拟合预测模型。

## 参考真值

原始实跑报告没有经过轨迹对齐核验的参考地形，验收误差仍为 JSON null / CSV 空值。2026-09-12 新增了独立的场景参考诊断，诊断值与原始验收字段分开保存；不能用采样充分度或点数替代精度。

离线评价支持 JSON 参考文件，字段示例：

```json
{
  "frame": "map",
  "resolution": 0.25,
  "source": "参考数据的生成方法与原始文件",
  "alignment_description": "坐标如何对齐、谁核验，以及误差范围",
  "cells": [{"ix": 0, "iy": 0, "z": 0.0, "slope": 0.0}]
}
```

示例仅展示格式，不是真值。程序检查坐标系名、分辨率、重复格子和有限数值；**不会因字段写了 map 就自动证明坐标对齐**。导入结果标为“提供了参考，对齐未独立核验”，需要另做对齐验收。真值只进入离线报告，不会送给探测策略。

## 重新生成报告与汇总

复用现有 Docker 环境，以下命令可在 PowerShell 运行。将路径中的实验/任务编号替换成实际值，`/evidence` 对应项目 `tmp/ros_acceptance`：

```powershell
docker exec mooncar-terrain-validation python3 /source/scripts/survey_evaluation.py report /evidence/frontend-experiments/<实验编号>/research/<自主任务编号>
docker exec mooncar-terrain-validation python3 /source/scripts/survey_evaluation.py summarize /evidence/frontend-experiments /evidence/frontend-experiments/research-summary
```

有参考文件时，给 `report` 增加 `--reference /evidence/<参考文件>.json`。重建报告使用当时指定的代码和参考，并更新校验文件；正式归档后应复制目录再重新分析，保留原报告。总表仅逐次列出，不自动把不同起点、基线、范围、代码或预算的实验求平均。

这一步交付的是研究数据与证据链，尚未实现三策略批量对照调度，也没有证明当前探测方法优于基线。下一步可以在这套固定口径上安排对照实验。

## 场景参考与误差诊断（2026-09-12）

`terrain_reference.py` 从每次实验冻结的 `survey_yard.sdf` 读取**视觉几何**，不是碰撞外包框。相机能看到的陨坑底面即使没有碰撞体，也进入参考。平面、旋转盒体、旋转椭球和竖直圆柱由解析竖直线求交；不支持的几何或坐标引用报错，不悄悄用平地替代。

固定评价区域、0.25 米分辨率、最终评价时刻均沿用原实验。每格用 5×5 个均匀位置的最高视觉表面拟合高度和坡度。这样得到的是统一地表表示的参考，**不是逐个深度像素的传感器真值**。相机可能只看到障碍侧面，均匀参考与稀疏观测的采样位置也不同；两者的差异可能进入误差。

统计包含全部区域格子，分别报告未观测、已观测但无有效拟合、有效匹配。高度 RMSE、平均偏差、绝对误差 P95 和坡度 MAE 只在有效匹配格计算，分母一并保存。分组为：平地 ground、单一地形表面 terrain、边缘或复杂格 edge_or_complex。表面归属不一致、参考拟合残差超过 0.025 米或坡度至少 60° 时单列为复杂格；这些是分析分类阈值，不是车辆安全阈值。5×5 采样可能遗漏宽度小于 5 厘米的细节。

Q 按 [0,.2)、[.2,.4)、[.4,.6)、[.6,.8)、[.8,1] 分箱，并按地形类别拆开。每箱给出有效和无效样本数，避免只看到平地占据的总体相关性。相邻格高度相关，不能当成大量独立实验；这些结果不能证明补采导致精度改善。

坐标暂按启动配置建立的 map/world 恒等对应，不使用采集地形进行反向配准。三次旧实验没有同步 Gazebo 真值轨迹，因此均标 `nominal_unverified`、`accepted_accuracy: false`。新增 `check_reference_alignment.py` 可读取同一仿真时刻的 Gazebo 模型位置和 ROS map→base_footprint 变换，只读检查当前坐标差，不拟合修正，也不控制车辆。停车点的三次样本不能替代全程、多位置对齐验证。

以后自主实验结束、原始报告与哈希生成完成后，现有后处理入口会自动生成 `research/reference-analysis/<任务编号>-<分析编号>/`，包含三组黑白 PNG/PDF 图、逐格 CSV、分类与 Q 分箱 CSV、摘要及代码快照。诊断失败会另存 `.failure.json`，原始报告继续保留。数据只进入离线分析，不进入导航或自主候选选择。

对已冻结的记录单独分析（输出目录必须不存在，且不能位于原记录内部）：

```powershell
docker exec -e OPENBLAS_NUM_THREADS=1 mooncar-terrain-validation python3 /source/scripts/terrain_reference.py /source/reports/evidence/terrain-workbench/continuous-20260912/autonomy-137ca52327 /evidence/reference-new/autonomy-137ca52327
```

当前结果见 `reports/evidence/terrain-workbench/reference-20260912/README.md`。下一步先补同步真值轨迹、检查高度基准与可见表面对应，再研究 Q 与有效地形精度的关系，最后做同场景同预算的探测策略对照。

## 同步轨迹与高度检查（2026-09-13）

`scripts/trajectory_evaluation.py` 是独立评价工具，不发布真值或运动指令。它通过 Gazebo 的 dynamic_pose/info 连续保存模型位姿，同时选取 `/mooncar/terrain/observations` 的光学点。深度时间戳必须由前后真值包围且间隔不超过 0.05 秒，禁止外推；ROS TF 也查询同一时刻。默认每至少 0.5 仿真秒选取一帧，和地图记录器的关键帧规则不同。

相机真值安装变换从运行时机器人 URDF 经 `gz sdf -p` 转换的 SDF 读取，加上明确的 Gazebo 相机轴到 ROS 光学轴转换。不得以 ROS 定位变换直接充当独立相机安装真值，也不得用地面点拟合修正后再报告原始精度。

工具保存真值 JSONL、光学点与两套位姿 NPZ、内参、拒绝记录、逐帧 CSV、轨迹/高度 PNG/PDF 和代码快照。全平地投影范围与排除非平面视觉体膨胀外包框的平地内部范围分别统计；固定空间余量 0.25 米，不按观测高度误差筛选。

运行前核实实验分区、ROS domain、运行时模型及场景文件，下面的实验编号和路径仅展示格式：

```powershell
docker exec -e ROS_DOMAIN_ID=89 -e GZ_PARTITION=mooncar-<实验编号> -e OPENBLAS_NUM_THREADS=1 mooncar-terrain-validation bash -lc 'source /work/ros2_ws/install/setup.bash; python3 /source/scripts/trajectory_evaluation.py capture /evidence/<新记录目录> --robot-sdf /evidence/<核验后的机器人模型>.sdf --world-sdf /evidence/frontend-experiments/<实验编号>/survey_yard.sdf --seconds 85'
docker exec -e OPENBLAS_NUM_THREADS=1 mooncar-terrain-validation python3 /source/scripts/trajectory_evaluation.py report /evidence/<记录目录>
```

采集必须先启动，再另行启动正常自主任务；采集脚本自身不会让车辆移动。输出目录必须是新目录。当前同步轨迹采集**还不是普通自主实验的默认自动附加项**；普通实验自动输出的仍是既有采样报告和参考诊断。正式归档后重新分析应先复制目录。

本轮实跑结果见 `reports/evidence/terrain-workbench/alignment-20260913/README.md`。局部时间同步与安装坐标核验已完成；全场地轨迹、坡面、原始深度投影标定及实物精度仍待验证。

## 原始深度与像素中心核查（2026-09-13）

后续核查定位到当前 Gazebo Ogre2 深度采样与投影之间的半像素约定差异。共享 `project_depth` 新增 `pixel_offset`，默认 0；Gazebo 导航启动文件与仿真采集脚本明确设置 `depth_pixel_offset=0.5`。地形风险检测和地图观测共用修正后的光学点。实机仍使用其驱动、内参和标定约定，不自动套用仿真偏移。新实验生效，旧档案不改写。

同步采集命令加 `--raw-depth`，会额外保存同时间戳的完整深度图字节与格式，并读取运行节点的像素偏移参数。当前工具要求节点已声明该参数；原始图只是独立核查的可选采集项，尚未自动加入每次常规自主任务。

`depth_projection_evaluation.py` 用独立场景模型与同步相机真值位姿，对每个有效深度像素计算沿视线的最近可见表面。盒体侧面、椭球、圆柱侧面和顶底盖、有限地面均参与遮挡求交。比较的是**光轴深度残差**，不再把侧面直接和最高地表相减。它逐点核对原始深度与实际发布点云，并在同一组观测像素上比较绝对偏移 0 和 0.5；缺失参考及首个可见物体改变的数量单独保留，不按误差删除样本。

```powershell
docker exec -e OPENBLAS_NUM_THREADS=1 mooncar-terrain-validation python3 /source/scripts/depth_projection_evaluation.py /evidence/<已完成的同步记录> /evidence/<新的分析目录>
```

输出包括逐帧与分组 CSV、逐点残差 NPZ、原始深度/两种射线约定的黑白 PNG/PDF、源代码与文件哈希。输入必须完成，输出必须是源目录之外的新目录。运行时模型需另行与冻结 SDF 核对。

结果见 `reports/evidence/terrain-workbench/projection-20260913/README.md`。半像素规律在旧移动样本和新原始深度中复现，并经新的仿真任务验证；这不等于整幅地图、坡度、复杂边缘或实机精度通过验收。解析几何与实际渲染网格也存在差异。下一步应在修正后的新档案上区分地面、斜坡、障碍侧面和边缘，检验采样充分度 Q 与可用表面质量的关系，再开展同预算策略对照。

## 可见表面与 Q 的分组核查（2026-09-13）

`scripts/surface_quality_evaluation.py` 使用已冻结、修正后的同步记录离线重放：实际深度和同射线的最近可见解析表面分别经过现有体素融合、地表拟合与 Q 计算。两者采用同一真值位姿以隔离定位误差；这两套评价地图不能导入导航，也不能作为在线建图成果。它使用同步评价帧，不能与普通任务关键帧的统计混合。

参考类别来自独立预期射线交点，分别标记地面、坡顶、弯曲上表面、陡侧面、同格多体/多面混合。没有按误差大小挑选参考。保留所有 ROI 格子，只有测量和参考均有效时计算拟合差；混合面的成对拟合仅作表示诊断，`single_surface_pair=false`。

输出的 `cells.csv` 增加 `fit_status`，区分少于 6 个体素点、XY 分布不足、拟合过陡、非平面/多高度与有效拟合；它们来自现有拟合结果，不改变生产阈值。`quality-bins.csv` 按类别记录各 Q 档的有效、无效及成对数量，空误差不能作为零。

本次一条轨迹中，坡顶 36 格只有 8 格有效，其余 28 格均为点数不足；Q≥0.8 的混合格有 27 个，其中 16 个无效。总体高 Q 有效率 95.2% 被地面占比主导，不能解释为准确率。反例测试还验证了 Q=1 可以同时伴随固定高度偏差或多层拟合失败。

详细数据、黑白对比图、限制与复现命令见 `reports/evidence/terrain-workbench/surface-quality-20260913/README.md`。当前是独立离线工具，没有自动附加到每次任务，也没有改变在线 Q 或自主策略。后续先用同初始记录、同预算的坡道换位补采实验，检验是否真正增加有效拟合；在线决策只能使用传感器可计算的点数、分布和拟合特征，不能使用离线真值类别。

## 2026-09-13 诊断字段补充

遥测包含 diagnostics 和 stop_diagnostics，动作记录包含 health 与 motion_stop；
summary.json 包含任务 stop_diagnostics 和 last_motion_stop。它们保留数据年龄、原始 TF
报错及 Nav2 返回码；旧记录没有这些字段时为 null，不推断历史故障原因。
启动阶段失败另写实验目录 startup-failure.json，保留首次失败，后续恢复不覆盖。
