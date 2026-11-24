# 正式图片采集运行组件接入：停止，未发布默认配置

## 判定

| 项目 | 本轮结论 |
|---|---|
| 最小 RMW 构建、独立镜像和启动适配 | 已完成 |
| 无 Gazebo 的实际桥接容量与库核验 | 通过 |
| 正式 launch 与完整采集验证 | 未通过，首轮模型展开失败 |
| 新四轮校准 | 未执行，0/4 |
| 三场景及旧模型兼容仿真 | 未执行 |
| 正式 `auto` 默认启用 | 未发布，仍为 `legacy` |
| 完整车辆或转向验收 | 本轮不开展 |

本轮授权最多 10 次仿真，实际启动尝试 1 次，失败后立即停止。没有使用旧轮次补数，也没有修正后补跑 Gazebo。

## 已完成的运行组件

原始归档 RMW 8.4.4 发行源码构建的最小补丁仅设置桥接发送段为 8 MiB，并在真实 participant 创建成功后写一次初始化回执。Fast DDS 2.14.6 及其丢弃、重组和返回行为完全使用原安装版本。构建约 64 秒，未超过 45 分钟预算。

原镜像：`sha256:b0097fb21186f20191b71505315e6041ee19ca966504a59f08437172bbe0db1f`。
新镜像：`mooncar-terrain:r5-image-dds-20261008`；完整镜像信息见 `image.json`。

`derived-preflight/result.json` 保存新镜像中实际桥接 PID、映射库与段文件核验。发送 payload 8,388,608 B，映射文件 8,782,368 B，管理开销单独计为 393,760 B；端口队列 512，max_message_size 65,500，两个用户传输描述符、builtin_transports=false。它是发布端单个段容量，不是容器 `/dev/shm` 总容量。

独立库路径为 `/opt/mooncar/image-dds/rmw-8.4.4`，不覆盖系统目录。初始化与进程门禁验证实际使用原 Fast DDS，并阻止诊断环境混入正式桥接。新启动配置只作用于桥接子进程；其他节点保留系统 RMW 和原共享内存设置。

环境选项 `MOONCAR_IMAGE_DDS_PROFILE=auto|legacy|shm_tx8m` 已接入，当前默认 `legacy`。本轮显式选择 `auto` 进行候选验证。只有 R5、三个 v2 世界、LOCALHOST 同时成立时自动模式才选择修复；不符合条件时明确记录旧配置，显式指定不支持的修复环境会失败。ROS 话题、网页 API 和已有 launch 参数未改动。

## 失败轮与停止

`campaign/01-formal-start-ramp-1000us/launch.log` 原始报错：

```text
error: Undefined substitution argument prefix
when processing file: /work/install/share/mooncar_description/urdf/mooncar_r5.urdf.xacro
```

加入只读插件的测试模型被 ElementTree 以 `ns0` 前缀序列化，xacro 参数声明因此无法识别。正式 launch 在模型展开阶段退出；该轮未取得有效 28 秒观察窗口，未取得图片或有效物理性能数据，不能判采集或车辆通过。

`campaign/ledger.json` 保留 1 次启动失败，`campaign/summary.json` 为 stopped、calibrations_passed=0、default_published=false。失败及冻结源码保留在 `campaign/source` 和 `campaign/freeze.json`。

停止后修正命名空间前缀，回归复现错误并验证修正。`post-stop-model-verification.json` 证明实际 R5 xacro 可以展开且去除观察插件后物理与控制定义未变。`post-stop-final-preflight/result.json` 是修正后代码的无 Gazebo 工具检查，不能补入原仿真签名或作为正式运行通过证据。

## 回归、保存与后续

完整软件 276 项通过：control 7、navigation 10、terrain 129、gazebo 130；新增回归 19 项。前端 3 项、固件 UBSan、Python/shell 语法检查通过，日志分别见 `software-final.log`、`frontend-final.log`、`syntax-final.json`。

本轮使用原基础镜像、4 CPU、6 GiB、512 MiB 容器共享内存、ROS95、独立 partition、无网页端口。源码、补丁、运行库、模型、观察工具、70 项资源哈希、容器和镜像信息均保存；原工作台状态及资源保留检查见 `preservation-check.json`。

本轮正式运行速度、图片延迟、CPU/内存物理性能比较缺少有效窗口，明确记为不可判定。当前失败未改变上一轮容量因果对照及四轮校准已经通过的结论。

下一轮需使用修正工具、新容器和新证据目录，重新冻结并完成正式启动 2 次、校准 4 次、三场景 3 次、旧配置兼容 1 次；只有全套通过才能发布限定范围的自动默认行为。约 31 cm 转向偏移及完整车辆验收另行安排。
