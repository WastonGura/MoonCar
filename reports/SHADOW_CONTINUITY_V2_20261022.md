# 旁路2.051秒空窗原因、调度修复与单轮V2验证

日期：2026-10-22，Asia/Tokyo。

**导致长空窗的调度漏算已修复；本轮实际最大交付间隔从上轮2.051降至0.454仿真秒，固定平移和转向路线完成。旁路仍有丢帧及迟到输出，尚未实现完整连续可用。** 默认定位方案、启动CPU配置和回访流程均未改，没有追加第二个世界。

## 查清的原因与最小修改

上轮 `experiment-bd3ef8a69286` 的最长空窗是交付时钟23.015→25.066s，对应成功曝光22.7→24.8s。其间20个真实曝光，18个完成前端处理、2个被替换；18个中7个没有求解计划，其余11个因共享身份只有19或21个、低于原24个要求而拒绝，后端没有运行。20个失败计划为9个缺曝光和11个支撑不足。不是一次2秒的优化计算把全部输出堵住。

直接缺陷在实时 `tick()`：它用当前仿真时钟提前把未交付的曝光计划判为缺失，推进下一计划。真实RGBD随后到达时，虽然仍被正常处理，却得到 `planned_stamp=None`，只更新前端、不进入MAP求解，也不提交持久关键帧。关键帧漏更新使地图中的有效身份不能及时接力，随后出现共享身份不足。

本次只移除实时tick中的提前终结，保留时钟倒退检查。缺少的曝光由后继真实曝光的时间戳确认；退出时再补齐尾部所有100ms计划。没有重发旧位姿、修改曝光时间戳或删掉迟到/失败分母，也没有放宽共享门、空间门、噪声、六帧/96观测预算或500ms关键帧间隔。`online.py`仅更新说明，行为未改；默认定位和回访源码原样。

对同一批已有数据做了一次必要的双臂对照，仅到25.1s：

| 同一184个实际已处理曝光 | 原计划归属 | 修正计划归属 |
|---|---:|---:|
| 成功输出 | 106 | 184 |
| 共享支撑失败 | 11 | 0 |
| 最大成功输出曝光间隔 | 2.1s | 0.6s |

原计划臂的原生位置、完整C、前端及求解诊断均精确复现。修正完整前缀的计划归属后，关键帧持续接力，原失败区间共享数升至49–66。不能把效果归因于单独补22.9s某一帧；离线曝光间隔也不是在线交付时限保证。这次对照只改变计划归属，没有加入真值、原定位位置或控制反馈作为估计输入，累计CPU约36秒，没有重放其它历史长录像。

适配15项、统计17项，共 **32项针对性检查通过**。新增检查覆盖仿真时钟先推进而真实曝光迟到时仍以原槽求解，以及真实跳帧和收尾分母恰好记录一次。

## 唯一新V2运动验证

实验 `experiment-6be5f7d77deb`，原生无标柱V2、起点`[0,0,0]`、legacy车型；原默认RGBD负责控制，原6CPU/6GiB/1GiB共享内存及CPU分配保持。固定路线为6个有界云台动作、等待、导航至`[1,0]`、`yaw=π/4`转向请求、结束等待。保留默认失健或故障计数增加即停的护栏。

实际10项动作均成功，驱动结果 `route_complete`。独立真值确认：平移动作净XY位移0.78685m、路径0.78974m；转向动作净偏航56.29°、伴随XY位移4.56cm。转向请求是45°，这里只确认真实转动覆盖，不能称角度跟踪精度通过，本轮未改控制器。

主窗为首个真实配对0.448s至终止状态53.082s，共52.634仿真秒，包含初始化和停车观察；采集/世界关闭尾段单列。全部527个100ms计划均保留。

| 旁路指标 | 本轮 |
|---|---:|
| RGBD配对 / 实际处理 | 525 / 329 |
| 实际处理但没有求解计划 | 0 |
| 替换 / 拒绝 | 195 / 1 |
| 计划输出 / 无输出 | 327/527（62.05%）/ 200 |
| 窗内 / 窗后交付 | 326 / 1 |
| 最大连续失败计划 | 4（上轮20） |
| 最大实际交付空窗 | 0.454仿真秒（上轮2.051） |
| 移动条件输出 / 条件XY误差P95 | 37/69 / 6.338mm |
| 转向条件输出 / 条件XY误差P95 | 28/54 / 6.261mm |
| 纯转向条件输出 | 3/8 |
| 全部成功输出XY误差P95 / 最大 | 6.304 / 6.703mm |
| 条件模型XY σ P95 / 最大 | 4.930 / 5.069mm |
| 端到端墙时P95 / 最大 | 0.60903 / 0.83612s |
| 成功输出交付年龄P95 / 最大 | 0.397 / 0.500仿真秒 |
| 按实际交付积分的新鲜时间占比 | 29.19% |
| 最长不新鲜区间 | 2.272仿真秒 |

200项无输出包括194项计划曝光被替换、2项共享支撑不足、1项真实区间缺少曝光、1项相机TF等待超时和2项关闭时缺少曝光。两项共享拒绝发生在24.9/25.0s，连接数22/23，原24门限保留，没有为了连续输出放宽。它们和其它失败全部计入。

327个输出均有独立真值评分，其中1个在主窗后交付。动态误差只评价实际成功输出的37个移动样本、28个转向样本；两种分类可重叠，不能据这些样本宣称整个运动窗精度通过。端到端墙时来自330项有起止记录的尝试，包含失败，分母与成功输出不同。

243/527个计划交付年龄≤0.3仿真秒，其中242个在主窗内交付；239/527同时满足端到端≤0.5墙秒。交付空窗缩短不等于持续新鲜，最长不新鲜仍为2.272仿真秒。两轮实际轨迹、窗口、调度及负载不同，2.051→0.454是观测比较，不是受控加速倍数。运动成功样本误差没有比上一轮约4mm更好，不能以输出数增加宣称精度提升。

44个真实同时间精确基线共同样本中，新/旧XY误差P95为6.299/3.633mm，仅是共同样本附表，不宣称新方法更准。C保持条件模型量；坐标只按预先声明的初始IMU偏航、零安装偏置和首个成功曝光平移原点对齐，没有拟合轨迹旋转或尺度。

## 默认定位、收尾和边界

就绪屏障12.678s后，370次工作台状态均健康，故障计数保持1→1；1196条已采集原生质量状态无不健康记录。初始化期间曾有stale/recovering并留下故障计数1，不能写全程零故障。原生计算前、计算后观测超时累计均为0。线程分配监视没有越界。

driver、旁路、观察器、bag均退出码0且未强制终止，世界最终`phase=stopped`、所属仿真子进程0，含清理约114.374墙秒。27个受保护源码首末哈希一致，本次候选源码在实验期间也未变化。工作台服务保留，没有第二轮实验。

- [空窗直接证据](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-continuity-v2-20261022/gap-cause-before.json)、[同输入对照](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-continuity-v2-20261022/replay/comparison.json)、[修复后空窗及残余失败](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-continuity-v2-20261022/gap-after.json)。
- [完整评价](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-continuity-v2-20261022/evaluation/summary.json)、[逐计划记录](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-continuity-v2-20261022/evaluation/plans.jsonl)、[实际动作真值](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-continuity-v2-20261022/physical-actions.json)。
- [专项测试](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-continuity-v2-20261022/tests/adapter-evaluator-tests-01.json)、[默认健康](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-continuity-v2-20261022/default-health-summary.json)、[实验结果](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-continuity-v2-20261022/result.json)。
- [源码和副本哈希](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-continuity-v2-20261022/integrity.json)、[大文件原位清单](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/shadow-continuity-v2-20261022/raw-files.json)。完整bag保留原位。

本次完成了直接缺陷修复和一轮运动验证，确认长空窗缩短；旁路整体连续性及在线时效仍未通过，不替换默认，不改回访，也不继续扩展修复。
