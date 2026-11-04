# R5 DDS 监控干扰修复（2026-10-15）

**监控工具链修复及无物理步进回归通过。完整车辆 17 轮验收尚未重新执行，原失败结论保留。**

## 修复内容

- 新增宿主进度工具 `r5_host_progress.py`，只读取挂载目录中的运行账本。Windows
  文件句柄允许并发读写／替换，暂不可读、半写入或非法样本显示 `pending`，不沿用
  旧合格计数，不发送命令或创建容器内进程。
- 新研究批次明确声明并冻结 `host_files_v1`：`host_mount_readonly`，观察期间不使用
  容器 exec 轮询。准备清单、诊断清单和两种运行账本均保存该配置；缺失或错误的
  监控声明会被拒绝。旧失败批次可以只读显示，不能作为新配置的验收结果。
- DDS 清单错误增加读取操作、具体路径、errno、堆栈、PID 启动身份、前后状态、
  命令和可读的权限字段。每 PID 原有第一次 stat 读取同时保留名字和状态，正常
  扫描不额外读取这些错误详情。读取失败后的进程退出也不会抹掉原权限错误。
- 权限拒绝、PID 身份变化、未知映射及未解析所有权仍会失败。没有忽略所有
  `PermissionError`、按进程名字放行或提高容器权限。
- 正式无物理门禁纳入新监控回归和原 DDS 生命周期回归；真实 ROS 前置证据使用本批
  新生成的结果，并将结果哈希加入冻结签名。

触发机制的原始证据与历史身份限制见
[查因报告](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/R5_DDS_PERMISSION_CAUSE_20261015.md)。

## 验证结果

| 检查 | 结果 |
| --- | --- |
| Windows 本地新监控／错误定位单元回归 | 16 项通过 |
| 独立 overlay 中完整无物理门禁 | **129 项通过，0 失败、0 错误、0 跳过** |
| 本批真实 ROS：短命发送者、接收者及历史映射归属 | 通过 |
| 本批真实 ROS：活跃 peer 所有权、容量与异常收尾 | 通过 |
| Windows 宿主实际进度工具连续轮询，8 秒×3 | **3/3 通过**，156／155／155 个进度样本 |
| 故意注入旧式 `docker exec` 查询×3 | **3/3 正确拒绝**，三次均在一次注入后记录权限错误 |
| 描述模型／安装一致性、中性质量和惯量、历史库哈希、生产世界步长 | 全部通过 |

宿主轮询三轮中，2／3／2 个样本遇到半写入而显示 `pending`；后续样本恢复，最终
状态均为 `completed`。全部观察线程正常关闭。故意注入组的观察器均为失败，测试
因确认严格拒绝和完整错误证据而通过，不能将这些组写成观察器成功。

一个实际注入样本现在完整记录：`readlink('/proc/1354/fd/0')`、errno 13、
启动身份 `(PID=1354, start_ticks=1715682)`，读取前后名字 `runc:[2:INIT]`、运行态，
命令 `runc init`，以及抛错源码堆栈。这些是本批复现 PID，未冒充旧第 10 轮的 PID 5775。

宿主工具读取旧 v3 账本仍显示 `stopped`、启动 10、资格合格 9，并保留
`declared_monitoring_profile=null`。新配置没有改写历史记录或升级其验收状态。

## 运行方式及边界

在宿主运行，`--runtime` 为绑定到容器 `/diagnosis` 的宿主目录；加 `--watch` 持续读取。
具体命令和文件共享说明见
[研究监控说明](D:/Documents/Projects/Repositories/MoonCar-mapping/docs/r5_research_monitoring.md)。

本次没有启动 Gazebo server 或执行物理步，没有正式车辆诊断／验收试验，也没有
拼接旧数据。仅生成模型和 17／12 轮未执行模板用于一致性检查。车辆物理参数、
转向控制、生产默认 `rigid` 和生产世界 1 ms 保持不变，5 秒见证绝对截止和 8 秒
父进程预算不变。

再次进行完整车辆验收时，应使用修复后的宿主监控方式，并重新冻结独立批次。
旧 v3 的第 10 轮仍失败，整体仍未通过，当前不能标为“支撑模型修复候选”。

## 交付

- [完整回归门禁及冻结哈希](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-dds-monitor-fix-20261015/runtime/nonphysics-gate.json)
- [实际宿主轮询与故障注入结果](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-dds-monitor-fix-20261015/runtime/monitor-regression-summary.json)
- [源码与证据核验](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-dds-monitor-fix-20261015/runtime/repair-summary.json)
- [宿主监控代码](D:/Documents/Projects/Repositories/MoonCar-mapping/scripts/r5_host_progress.py)
- [读取错误定位代码](D:/Documents/Projects/Repositories/MoonCar-mapping/scripts/image_dds_ownership.py)
