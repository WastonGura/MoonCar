# 正式图片采集修复重新验证：首轮收尾失败，停止且未发布默认值

## 本轮判定

| 项目 | 结果 |
|---|---|
| 准备检查、真实模型展开与 SDF 一致性 | 通过，R5 和旧模型均检查 |
| 新 overlay 安装文件、版本、依赖、库哈希 | 通过 |
| 正式启动验证 | 已尝试 1/2；首轮启动成功，验收收尾失败；第二轮未运行 |
| 实际配置生效 | 首轮确认自动选择修复，桥接 8 MiB、接收端 512 KiB，未发现配置泄漏 |
| 新四轮校准 | 未运行，0/4 |
| 三场景专门采集 | 未运行，0/3；首轮使用坡道但不能补足此阶段次数 |
| 旧模型兼容仿真 | 未运行，0/1；模型工具检查不能替代仿真 |
| 默认配置发布 | 未发布，仓库仍为 `legacy` |
| 整车、云台或转向验收 | 未开展，不作通过结论 |

本轮独立上限 10 次，实际仿真尝试 **1 次**，失败后按冻结停止规则结束，没有补跑、恢复目录或跨签名配对。此前 `r5-dds-production-20261008` 的模型展开失败及更早证据全部保留。

## 仿真前实际完成的准备

复用镜像 `mooncar-terrain:r5-image-dds-20261008`，ID `sha256:f819df21c8584f8a3d1eee315106779e9b380f10cf43fdc8b7aa15c6bdec6ee3`。最小 RMW 8.4.4、原 Fast DDS 2.14.6 及两个库哈希重新核验，未使用逐帧诊断运行库，没有覆盖 `/opt/ros` 或重建原工作台。

从当前源码复制并构建独立 `/work/install` overlay。核对实际安装的 launch、辅助脚本、全部模型及 include、桥接和过滤配置、安全中继、轮式里程计、世界、材质和几何资源。候选默认文件是唯一预声明配置差异：仓库 `config/image_dds_defaults.json` 为 `legacy`，隔离 overlay 使用 `containers/image_dds/auto-defaults.json` 的原样内容 `auto`，环境变量仍可覆盖。

`final-preflight/result.json` 保存真实 R5 和旧模型的观察版本生成、xacro 展开、Gazebo SDF 转换结果。保留字面 `xacro` 前缀；去掉 PosePublisher/StepObserver 后，URDF 和 SDF 的语义树与原模型相等；本次 R5 还与此前归档基线相等。观察插件只读取状态。

桥接就绪门禁等待完整 JSON 初始化回执，不能仅凭文件出现判就绪。增加接收进程关闭前的库和段快照、异常关闭后的归档状态保存、安装文件门禁、图片解码和有效内容检查。相关工具回归 29 项通过，之后才冻结并运行。最终冻结签名见 `campaign/freeze.json`，源码快照见 `campaign/source`，实际 overlay 包见 `overlay.tar.gz`。

## 首轮实际行为及失败原因

首轮为 R5、坡道 v2、1 ms、原控制默认值、零命令。正式 launch **未设置** `MOONCAR_IMAGE_DDS_PROFILE`，由安装的候选默认文件选择 `auto`，日志明确选择 `shm_tx8m`。原版本桥接成功启动，车辆创建成功并产生可检查的 RGB/深度数据。

首轮失败发生在测试收尾：脚本已请求暂停并收到 paused 状态，随后按数字读取 `simTime.sec`。Gazebo 的 protobuf JSON 将 int64 秒数表示为字符串，表达式 `sec + nsec / 1e9` 抛出：

```text
TypeError('can only concatenate str (not "float") to str')
```

测试预检遗漏了真实暂停 JSON 的整数类型，这个错误属于验收工具。`pause-json-reproduction.cc` 使用当前安装的 gz-msgs10 / protobuf 库，不启动 Gazebo，产生 `{"simTime":{"sec":"29","nsec":736000000},"paused":true,"iterations":"29736"}` 并复现同一异常，证据见 `pause-type-proof.json`。下一轮应复用已有处理整数类型的 `stats_state` 解析器，并完整执行 ROS 尾部接收确认；**本轮冻结后没有修改执行代码或补跑**。

异常跳过了正常的迭代、观察器 flush 与 ROS 尾部接收联合确认，进入失败清理。归档队列确实排空，但这不能代替消息已接收完整的判定。正式 `result.json` 保留原始错误及 `sim_started` 缺失的分析错误，`ledger.json` 记录 49.21 秒完整墙钟和退出码 1；`summary.json` 保留真实完成次数及 stopped 状态。

## 只读复核：有数据，但不补判为通过

`after-stop-audit.json` 使用已记录的观察起点 1.545 秒和逐步观察器暂停标记 29.736 秒，检查原始数据。由于正常流程没有保存并核对实际 stats 迭代数，以下只是停止后的诊断窗口，不能替代正式结论。

| 数据 | 最大窗口间隔 | 原门槛 | 备注 |
|---|---:|---:|---|
| RGB、深度 | 均 1.1 s | 1.5 s | 58 份选定文件全部可读取、有完成回执；RGB 有画面、深度有有限正值 |
| 激光 | 0.6 s | 0.75 s | 文件有有效数据 |
| IMU | 0.186 s | 0.1 s | 超标位于末尾；内部最大 0.010 s |
| 关节 | 0.214 s | 0.1 s | 超标位于末尾；内部最大 0.068 s |
| 安全速度 | 0.194 s | 0.1 s | 超标位于末尾；内部最大 0.053 s |
| 车体真值 | 0.010 s | 0.1 s | 完整 |
| R5 逐物理步 | 无漏步 | 无漏步 | CSV 完整，联合暂停确认未完成，不能判正式通过 |

所有记录到的安全速度和 Gazebo 实收命令均为零。图片样本未出现超出 1.5 秒的归档空档，但只有一轮，且该轮整体失败，不能证明新正式矩阵通过。尾部缺口与收尾确认被异常中断相符；没有据此认定出现新的 DDS 或相机故障。

实际桥接 PID、加载库和初始化回执见首轮 `bridge-receipts/*/ready.json`。发送 payload 8,388,608 B，段映射文件 8,782,368 B，管理开销 393,760 B；接收 payload 524,288 B，映射文件 549,408 B。8 个其他 ROS participant 的映射和环境快照均保持系统 RMW 与接收段容量，没有容量变量泄漏。Fast DDS 使用原安装库，端口队列 512、用户传输数 2、builtin_transports=false。ROS 图中不可获取的 QoS 历史/深度字段保留为 null，不补零。

首轮观测窗口 28.191 仿真秒、完整尝试墙钟 49.21 秒，按窗口/完整墙钟计算速度约 0.573；采样峰值内存约 1.71 GiB，CPU 累计约 165.46 CPU 秒。容器限制固定为 4 CPU、6 GiB、512 MiB shm、ROS95、无网页端口，独立 partition。详见 `resources.jsonl` 和只读复核记录；软件回归在物理试验结束后单独运行。

## 回归与保存

完整软件 **286 项**通过：control 7、navigation 10、terrain 129、gazebo 140。前端 3 项、两个脚本语法检查、固件 UBSan 检查通过；72 个 Python、4 个 shell 文件语法通过。它们不补足正式仿真判定。

归档包括冻结源码、安装 overlay、镜像和库 manifest、70 项资源哈希、运行限制、启动日志、实际库/段/QoS、原始观测、图片、深度、逐步 CSV、暂停标记、命令及性能、归档回执和全部失败。`archive-manifest.json` 与 `verify-archive.json` 用于文件哈希核验，大体积原始数据留本地，不进入 Git。

结束只停止 `mooncar-r5-dds-revalidation-20261008`。原工作台的状态、原有未提交改动和 70 项资源哈希另行核对保存。提交只包含本轮工具、配置、回归、报告与工作日志。仓库默认继续 `legacy`，不启用 D=0 默认或新控制 profile，约 31 cm 转向偏移保留。
