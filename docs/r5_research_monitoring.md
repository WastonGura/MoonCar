# R5 研究批次进度监控

新批次声明 `monitoring_profile.name=host_files_v1`。观察器活动期间，从宿主读取已挂载
的账本文件。`docker exec` 会制造短暂的 `runc init`；全 PID DDS 观察器无法读取该
初始化进程的部分文件信息，按现有严格规则会停止批次。

在宿主 PowerShell 运行进度工具，`--runtime` 必须指向该批次绑定到容器 `/diagnosis`
的宿主目录。例如以下命令只读取已停止的历史 v3 批次：

```powershell
python ../scripts/r5_host_progress.py --runtime ../reports/evidence/terrain-v2/r5-suspension-acceptance-v3-20261015/runtime
```

监控新批次时，改为新批次宿主路径并加 `--watch`，默认每五秒更新。该工具只有文件
读取和标准输出，不进入容器，不发送控制指令。Windows 读取允许文件同时写入／替换；
半写入、暂不可读和非法账本均显示 `pending`，不沿用上次的合格计数。

进度显示仅引用账本中的原有资格结果，不产生新的验收结论。新批次的准备清单、诊断
清单和运行账本均保存监控配置，缺少配置或声明使用 exec 轮询时拒绝新研究运行。
旧记录可以只读显示，不能据此升级为新配置验收通过。

观察器继续拒绝权限错误和未解析所有权。新增错误字段包括 operation、path、errno、
PID 启动身份、前后状态和 traceback；进程随后退出不会抹掉已经出现的权限错误。
不要忽略所有 `PermissionError`，也不要为进度查询提高容器权限。

本次修复及无物理步进回归见
[修复报告](../reports/R5_DDS_MONITOR_FIX_20261015.md)。
