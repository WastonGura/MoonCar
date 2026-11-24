# R5 DDS 观察器权限错误查因（2026-10-15）

**已确认可稳定复现的触发机制：进度查询的 `docker exec` 与 DDS 观察器相互干扰。**
在同一历史镜像和默认容器权限下，使用未修改的原观察器，注入查询的三次试验全部
出现与原轮相同的 `PermissionError(13, 'Permission denied')`；宿主直接读取进度文件
的三次对照全部通过。全部试验物理步数为零，没有启动 Gazebo，也没有改动车辆参数。

原第 10 轮没有记录 PID 5775 的程序名和失败路径，因此不能追溯证明该历史 PID
就是 `runc`。目前结论是：已找到并重复验证一条与原失败一致、且存在于当时监控
流程中的触发链；这一历史身份缺口仍保留，不把复现 PID 冒充原 PID。

## 触发链与具体缺陷

1. 当时的进度监控通过 `docker exec … python3 …/progress_all.py` 进入容器查询。
2. `docker exec` 的创建阶段，容器 PID 空间中短暂出现 `runc init` 进程。
3. `image_dds_ownership.inventory()` 扫描容器所有 PID。它先读取每个文件描述符的
   符号链接，然后才判断链接是否为 Fast DDS 锁文件。
4. 新复现捕获到：`os.readlink('/proc/42/fd/0')` 返回 EACCES。读取前后状态均为
   `runc:[2:INIT]`、命令 `runc init`、运行态、UID 0。其临时有效 capability 掩码为
   `000001ffffffffff`；观察器为 `00000000a80425fb`，没有 `CAP_SYS_PTRACE`。
5. `inventory()` 把该错误写入全局错误列表；`Lifecycle.observe()` 收到任一错误即
   停止；调度健康检查再抛出 `Lifecycle observation failed`，整轮按失败关闭。

因此，观测范围覆盖了初始化阶段受保护的容器运行时进程，而进度查询恰好制造该进程。
触发点在 DDS 锁文件筛选之前。诊断捕获的权限差异与 Linux 的跨进程文件信息访问
规则一致：读取 `/proc/PID/fd/*` 符号链接需要 ptrace 访问检查，UID 相同本身不足以
保证通过，检查还涉及 capability 等条件。
[Linux proc_pid_fd 手册](https://www.man7.org/linux/man-pages/man5/proc_pid_fd.5.html)、
[Linux ptrace 访问检查](https://man7.org/linux/man-pages/man2/ptrace.2.html)。

上述证据没有直接测量目标进程的 dumpable 属性，也没有内核跟踪来区分具体哪一道
ptrace 子检查返回拒绝；不将 dumpable=0 作为历史或本次 `runc` 的实测事实。

## 无物理步进证据

| 试验 | 结果 | 意义 |
| --- | --- | --- |
| 普通程序同 PID 连续 exec，目标 20,000 次，20,738 次清单扫描 | 32.73 秒，无权限错误 | 一般 exec 本身未复现故障 |
| 明确将子进程设为 nondumpable 的正对照 | 成功捕获权限拒绝 | 验证诊断包装能记录真实权限失败；不是历史根因证明 |
| 诊断读取包装＋宿主注入 `docker exec /bin/true` | 首次注入即捕获 `runc init`、具体路径、状态、堆栈 | 定位实际被拒绝操作和进程 |
| 未修改 Lifecycle，宿主仅读挂载进度文件，8 秒×3 | **3/3 通过**，各 398 次扫描，合计 709 次宿主文件读取 | 读取进度文件不制造容器进程 |
| 未修改 Lifecycle，宿主注入 `docker exec /bin/true`×3 | **3/3 失败**，PID 94／108／122，错误与原记录相同 | 故障不依赖诊断包装或车辆运行 |

三次故障分别在 2、1、1 次 exec 注入内出现。配对试验没有 monkeypatch，原观察器
源码 SHA-256 均为 `84085a9fe66e7c569e1425bfe660b082de7179f954abfcc223f68a55841158a1`，
与 v3 冻结归档一致。历史镜像为
`sha256:6dc9c3677650c1f238d54ca67859e637980fe7052c64430eb699b826da26285d`，
内核 `6.18.54.1-microsoft-standard-WSL2`；仍采用默认 capability、`--init`、4 CPU、
6 GiB 内存限制。故障在没有车辆或大文件采集的条件下复现，不需要满内存这一条件。

首次宿主读取试跑曾因 `ReadAllText` 的 Windows 共享方式遇到文件共享冲突，宿主脚本
中断；该次原观察器仍运行满八秒并通过，记录保留为 `control-1`，未计入三组对照。
正式对照脚本使用允许并发读写／替换的文件句柄；暂时不可读的进度样本仅视为待更新，
不会产生任何验收通过判定。

## 修复方向与本轮边界

优先修复监控流程：观察器活动期间，宿主读取已挂载的 `campaign.json`／进度文件，
停止用反复 `docker exec` 查询进度。若需要实时物理步进度，应在观察开始前启动普通
容器内记录器，写出供宿主读取的进度文件，并纳入新的冻结清单。

同时补齐观察器错误证据：读取操作、路径、errno、堆栈、PID 启动身份、角色及前后状态。
如后续需要修改观察器的瞬态处理，仍需证明最终 DDS 所有权闭合，活动参与者不可读、
未知映射或清理残留继续失败。不能通过笼统忽略 `PermissionError` 或增加容器权限补判。

本次交付是查因结果与复现工具，车辆模型和生产观察器的判定代码均保持原状；没有
重跑正式矩阵。v3 的第 10 轮失败及整体未通过结论保持不变，未用本次对照替代正式验收。
原失败判定 SHA-256 保持
`a036ac941c3a63d5ffc3db2130dbce51f2a12671b3b8e3e5ac4a94678377bf31`。

## 复现文件

- [证据与复现说明](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-dds-permission-cause-20261015/README.md)
- [捕获的具体路径、runc 身份和堆栈](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-dds-permission-cause-20261015/runtime/watch/result.json)
- [未修改观察器的六次 A/B 结果](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-dds-permission-cause-20261015/runtime/ab-summary.json)
- [原观察器源码快照](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-dds-permission-cause-20261015/runtime/original-image_dds_ownership.py)
- [原验收报告](D:/Documents/Projects/Repositories/MoonCar-mapping/reports/R5_SUSPENSION_ACCEPTANCE_20261015.md)
