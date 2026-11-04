# 隔离 ROS 验证环境

工作台现在可在实验设置中选择 [Terrain V2（30×30 米，定位标柱）](../docs/workbench_terrain_v2.md)。该入口需要已安装完整场景资源，以及通过 `physics_guard.py` 校验的项目 DART 库和 `MOONCAR_DART_PLUGIN_ROOT`；旧基础镜像不保证具备该库。当前工作台复用已有 `6dc9c3677650` 镜像运行时，原车辆及默认 direct 定位。V2 世界和 GUI 均使用 ogre2。

## 在线勘测默认定位（2026-10-20）

构建并加载当前工作空间后，工作台及 `containers/experiment.launch.py` 未设置定位环境变量时默认 `MOONCAR_LASER_ODOMETRY=true`、`MOONCAR_LASER_ODOMETRY_MODE=direct`。地形导航的 `online_mapping=true` 也自动采用相同模式。已有服务保持启动时的选择，更新源码后应在结束实验后重启服务；Windows 包装器复用服务，不会强制切换当前会话。

旧研究基线可在**容器内服务进程环境**显式设置 `MOONCAR_LASER_ODOMETRY=false`；旧激光EKF候选用 `true` 加 `MOONCAR_LASER_ODOMETRY_MODE=ekf`。宿主变量不会自动传入既有 Docker 服务。无参离线AMCL和独立M4建图入口保持原模式。R5共享工作台同样继承新默认，定位精度仍待对应R5专项验证。使用及局部参考边界见[说明](../docs/local_metric_localization.md)。

## R5 支撑候选工作台

Windows 从当前目录运行 `./scripts/start_workbench.ps1 -R5Candidate`，在
`http://127.0.0.1:8775` 的“仿真车辆”中明确选择 **R5 支撑候选**，再应用任务。
原入口及 8765／8766 端口保持原状；已有实验不会被接管或重启。

新入口使用专用 `mooncar-r5-workbench` 容器、`--init`、LOCALHOST、独立 ROS
域及每任务 Gazebo 分区；镜像固定为
`sha256:6dc9c3677650c1f238d54ca67859e637980fe7052c64430eb699b826da26285d`。
每次容器启动从当前只读源码构建独立 overlay，并核对已验收的 DART、ODE、
固定轮速分配和侧滑执行库。原始会话保存在 `tmp/r5-workbench/sessions`。
没有本地镜像时明确失败，不自动替换运行库或拉取其他版本。

仅准备后端可运行 `./scripts/start_r5_workbench.ps1 -BackendOnly`；后端端口为
8776，此模式需使用 `docker stop mooncar-r5-workbench` 结束。正常前端入口按
Ctrl+C 会停止其专用容器。候选参数未标定、未经实车验证；原任务缺少配置字段时
继续使用 `legacy`，自主导航仍受原碰撞、禁行区、租约和传感器保护限制。

## 正式 R5/v2 图片修复（2026-10-09，十轮通过，auto 已发布）

默认配置现在为 `auto`：仅 R5、三个 v2 世界、LOCALHOST 启用桥接发送端共享内存 8 MiB；旧模型、v1、其他发现方式使用原配置。`MOONCAR_IMAGE_DDS_PROFILE=legacy` 可显式恢复旧行为。库缺失、版本/哈希不符、初始化或容量未生效时启动失败，禁止回退；只给桥接子进程加载最小运行库，接收端仍为 512 KiB。

正式启动两轮、四轮新采集校准、三个场景及旧模型兼容共 10/10 通过，完整软件 354 项、前端和固件通过后，仓库只发布已实际测试的同一份默认文件。旧模型没有 RGB，兼容检查按实际模型/桥接记录为未配置；R5 仍要求 RGB 和深度。发布配置与实际测试 overlay 的 SHA256 均为 `f3c4413e4c7ac87afe0b0a44d17a5fad0abcebacb8d846dcc5d7f4b1e8dab97f`。

继续使用独立镜像 `mooncar-terrain:r5-image-dds-20261008`（`sha256:f819df21c8584f8a3d1eee315106779e9b380f10cf43fdc8b7aa15c6bdec6ee3`）及现有最小 RMW 路径；从当前源码重建 overlay 以安装新默认文件。镜像库构建方法和显式覆盖入口见下方运行组件记录，不覆盖旧镜像或重建原工作台。

[完成报告](../reports/evidence/terrain-v2/r5-dds-compatible-20261009/README.md) 包含旧失败、资源/库/安装哈希、原始图片和逐步数据、性能及兼容证据。本轮未启用新的云台控制默认值，不构成转向或整车验收。以下 2026-10-08 条目是保留的失败历史，当前发布状态以本条目为准。

## 正式图片修复重新验证（2026-10-08，首轮收尾失败）

默认值现由 `ros2_ws/src/mooncar_gazebo/config/image_dds_defaults.json` 管理，仓库仍为 `legacy`；环境变量继续可以覆盖。隔离测试 overlay 使用 `containers/image_dds/auto-defaults.json` 的候选内容，正式启动不设置 profile 环境变量。只在完整新矩阵通过后才允许把同一份候选内容发布到仓库。

重建 overlay 并通过真实 R5/旧模型展开和安装文件预检后，新矩阵运行 1/10 即停止。正式桥接的 8 MiB 和其他接收端的 512 KiB 实际生效，图片可读取；测试脚本在解析暂停 JSON 的字符串秒数时失败，正常尾部确认被中断。四轮新校准及后续仿真未运行，不能判正式修复验收通过；冻结代码保留，没有补跑。

完整软件 286 项、前端及固件通过。报告：[新验证停止记录](../reports/evidence/terrain-v2/r5-dds-revalidation-20261008/README.md)。上轮失败报告继续保留。

## 正式图片采集运行组件（2026-10-08，默认尚未发布）

独立运行库和启动门禁已实现；无 Gazebo 的实际桥接核验通过，但新验收首轮因测试观察模型 xacro 前缀错误失败，按冻结停止规则结束。修正后的工具只完成软件回归，尚未重跑仿真。正式默认仍为 `legacy`，不要把本轮当作正式接入验收通过。

`MOONCAR_IMAGE_DDS_PROFILE=auto|legacy|shm_tx8m` 仅控制桥接进程。显式 `auto` 对 R5、三个 v2 世界、LOCALHOST 组合选用 8 MiB；其他组合记录旧配置。显式修复配置不受支持、库或版本不符、容量未生效、初始化超时均失败，禁止回退。其他 ROS 进程不加载新库。

基于上一轮已归档的固定 RMW 8.4.4 发行源码，在新隔离容器中执行：

```bash
source /opt/ros/jazzy/setup.bash
python3 /source/containers/image_dds/build_runtime.py --output /evidence/new-build --prefix /work/image-dds
```

仅构建 RMW，继续链接原 Fast DDS 2.14.6，不覆盖 `/opt/ros`。构建器限时 45 分钟，输出补丁、来源、依赖和库哈希 manifest。将两个 RMW `.so` 与 manifest 放入 `containers/runtime/image-dds/`（不入 Git），用 `containers/image-dds.Dockerfile` 构建新派生镜像，保留旧镜像。`MOONCAR_IMAGE_DDS_RUNTIME` 指向 `/opt/mooncar/image-dds/rmw-8.4.4`。

新验收入口为 `scripts/run_image_dds_production_campaign.py`，先通过 `scripts/preflight_image_dds_production.py`，使用全新输出目录；最多 10 次、每次 10 分钟、冻结后遇错停止，不恢复旧目录。测试所需原 D=0 模型及只读观察工具沿用已归档版本，传感器、QoS、安全中继和调度不变。

报告：[正式接入停止记录](../reports/evidence/terrain-v2/r5-dds-production-20261008/README.md)。

## R5 v2 物理后端

2026-10-06：三个 v2 世界及生成器默认使用 DART / 1 ms。v1、默认
`lunar_yard` 和已运行工作台保持原配置。本次 18 轮诊断及完整验收见
[R5 物理验收记录](../reports/evidence/terrain-v2/r5-stability-20261006/README.md)。
这次修正 DART 的网格摩擦导入，未修改任何车辆或摩擦数值。

自定义库必须针对当前 gz-physics 7.8.0 从固定源码重建，不能使用旧 7.6
二进制。在已加载 Jazzy 的隔离容器中执行：

```bash
source /opt/ros/jazzy/setup.bash
python3 /source/containers/build_dart_contact.py --output /evidence/new-dart-build
```

构建器校验上游压缩包 SHA256，保存原文件、修补文件、日志、库哈希和版本，
总构建预算 30 分钟。目标目录必须不存在；可用 `--archive` 指向已归档的
原始 7.8.0 压缩包。将新建的库及 `manifest.json` 复制到项目
`containers/runtime/dart-7.8.0/`，然后从仓库根目录构建独立派生镜像：

```powershell
docker build -f containers/r5-physics.Dockerfile -t mooncar-terrain:r5-dart .
```

该缓存目录不纳入 Git；源码、构建器及本轮归档可以重新创建它。镜像把库放在
`/opt/mooncar/physics/dart-7.8.0`，不会替换系统插件。
本轮派生镜像 ID 为 `b0097fb21186f20191b71505315e6041ee19ca966504a59f08437172bbe0db1f`，
基于现有 `ceab0ab15a7e031d2fba05cf4e7901b641979ac808a4fdef64b0404f92309fc9`。
镜像及启动器均核验库；运行时缺失、版本或哈希不符会直接失败。

复验时另建无端口映射的容器，将项目只读挂载到 `/source`、新证据目录挂载到
`/evidence`，设 `ROS_DOMAIN_ID=95` 和 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`。
复制 description、gazebo、control 三包到 `/work/src` 并用 colcon 构建；每轮
由脚本建立独立 Gazebo partition 和新世界。不要复用工作台的运行目录。

```bash
source /opt/ros/jazzy/setup.bash
source /work/install/setup.bash
xacro /work/install/mooncar_description/share/mooncar_description/urdf/mooncar_r5.urdf.xacro > /evidence/r5.urdf
gz sdf -p /evidence/r5.urdf > /evidence/r5.sdf
python3 /source/scripts/run_r5_acceptance.py \
  --output /evidence/new-acceptance --model /evidence/r5.sdf \
  --plugin "$MOONCAR_DART_PLUGIN_ROOT/libmooncar-physics7-dartsim.so"
bash /source/scripts/check_software.sh
```

`check_r5_physics.py` 提供平地/指定世界、2/1/0.5 ms、关闭两类控制的原因隔离，
最多 20 轮，每轮 10 分钟；`check_dart_contact_entry.py` 提供平板/网格和 μ=0/1
四组入口检查；`run_r5_acceptance.py` 固定 14 轮验收，拒绝覆盖旧结果。
每轮记录 RGB、深度、激光、IMU、关节、连杆/车体真值、ROS 安全速度及 Gazebo
实际接收的速度，记录缺失或超间隔不能通过。云台超时项明确注入反馈丢失。
验收脚本未运行 Nav2 策略比较，Gazebo 真值只进入观察与判定。
完成后停止自己的测试容器，保留原工作台。

以下为原环境与工作台的操作记录，镜像和域不与 v2 独立试验混用。

2026-09-10：镜像构建、8 个 ROS 包、35 项 Python 与 C11 核心检查通过，实际 Gazebo 采集、Nav2 执行及 M6 回归已运行。
证据与已知限制见 `docs/terrain_workbench.md` 和 `reports/evidence/terrain-workbench/`。
镜像为 `mooncar-terrain:jazzy`，本次 image config SHA256 为 `07465b330e3ff467eefcb4d1102e32f9efcbcba91be8b2ac33e26997e249db43`。

## 构建与软件检查

```powershell
docker.exe build -f containers/terrain.Dockerfile -t mooncar-terrain:jazzy containers
New-Item -ItemType Directory -Force tmp/ros_acceptance
$projectRoot = (Resolve-Path .).Path.Replace('\', '/')
$evidenceRoot = (Resolve-Path tmp/ros_acceptance).Path.Replace('\', '/')
docker.exe run -d --init --name mooncar-terrain-validation --cpus=4 --memory=6g --shm-size=1g `
  -p 127.0.0.1:8766:8766 `
  -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 `
  --mount "type=bind,source=$projectRoot,target=/source,readonly" `
  --mount "type=bind,source=$evidenceRoot,target=/evidence" `
  mooncar-terrain:jazzy
docker.exe exec mooncar-terrain-validation bash /source/containers/verify.sh
```

先启动 Docker Desktop，再从当前 `MoonCar-mapping` 根目录执行。Dockerfile 没有复制仓库文件，使用 `containers` 作为构建上下文即可。配置已补入 SciPy、Matplotlib、Pillow、PyYAML 和中文 Noto 字体，避免后续地形评价和出图依赖只存在于临时容器里。恢复后的具体版本以镜像 ID 与包清单为准，不与 2026-09-10 镜像视为字节相同。

构建使用 BuildKit 下载缓存和三次网络重试。默认保留上游仓库，可通过构建参数选择镜像；2026-10-06 恢复时实测清华镜像较快，使用以下命令（Ubuntu/ROS 签名密钥和校验仍保留）：

```powershell
docker.exe build -f containers/terrain.Dockerfile -t mooncar-terrain:jazzy `
  --build-arg UBUNTU_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ubuntu `
  --build-arg UBUNTU_SECURITY_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ubuntu `
  --build-arg ROS_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu containers
```

格式依据：[Ubuntu 镜像说明](https://mirrors.tuna.tsinghua.edu.cn/help/ubuntu/)、[ROS 2 镜像说明](https://mirrors.tuna.tsinghua.edu.cn/help/ros2/)。ROS 仓库仅启用 `deb` 二进制包索引；清华不提供此路径的 `deb-src` 索引，项目自身的 ROS 源码仍正常由 colcon 构建。

软件检查完成后，在 PowerShell 运行 `./scripts/start_workbench.ps1`，Windows 工作台地址为 `http://127.0.0.1:8765/`，容器后端为 `http://127.0.0.1:8766/`。该脚本校验当前路径挂载并复用已有后端；保持其终端打开即可。前端 Three.js 依赖缺失时，在 `web` 目录执行 `npm ci --ignore-scripts`，使用已有锁文件。

容器使用 ROS_DOMAIN_ID=76、软件渲染、容器网络，不连接物理车或旧 VM。

2026-10-06 已重新构建镜像，8 包、133 项 ROS/Python 回归和固件检查通过，Gazebo 实际传感器/地图/静止速度检查通过。版本及日志见 [恢复记录](../reports/evidence/environment-restore-20261006/README.md)。工作台实验使用独立 ROS_DOMAIN_ID=89；启动网页实验后，可执行只读检查（不发运动命令）：

```powershell
docker.exe exec -e ROS_DOMAIN_ID=89 mooncar-terrain-validation bash -lc 'source /work/ros2_ws/install/setup.bash; python3 /source/containers/check_runtime.py --report /evidence/runtime-probe.json --timeout 30'
```

源工作区只读，构建在 `/work`，证据写入 `/evidence`；软件检查与 Gazebo 验收是两类证据。
若容器已存在，先确认是本项目验证容器，不重复创建。不要同时启动两个同域 Gazebo 世界。

## 限次恢复的独立验收

`scripts/check_survey_recovery_runtime.py` 默认执行固定五轮：恢复后真实导航成功两轮、恢复后再次失败停车两轮、90 仿真秒无注入任务一轮；每轮重建世界和空白地图。受控轮次只注入导航结果并提供真实路径已验证的候选序列，云台、采集、安全中继和后续 Nav2 使用实际节点；无注入轮次使用原规划器。报告明确标记两类证据，不能用受控轮次证明真实障碍脱困。

为保留已有工作台且避免两套 Gazebo 争抢原容器的 CPU，另建使用相同镜像的测试容器，无端口映射。以下为 PowerShell 7.4+ 的复验命令（原生程序之间的 tar 管道保留二进制数据）：

```powershell
$projectRoot = (Resolve-Path .).Path.Replace('\', '/')
$evidenceRoot = (Resolve-Path tmp/ros_acceptance).Path.Replace('\', '/')
docker.exe run -d --init --name mooncar-recovery-validation --cpus=6 --memory=6g --shm-size=1g `
  -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1 `
  -e ROS_DOMAIN_ID=90 -e MOONCAR_EXPERIMENT_ROS_DOMAIN_ID=90 `
  --mount "type=bind,source=$projectRoot,target=/source,readonly" `
  --mount "type=bind,source=$evidenceRoot,target=/evidence" mooncar-terrain:jazzy
docker.exe exec mooncar-terrain-validation tar -C /work -cf - ros2_ws |
  docker.exe exec -i mooncar-recovery-validation tar -C /work -xf -
docker.exe exec mooncar-recovery-validation bash -lc 'source /work/ros2_ws/install/setup.bash; python3 /source/scripts/check_survey_recovery_runtime.py /evidence/新建恢复验收目录'
docker.exe stop mooncar-recovery-validation
```

测试容器已存在时复用它，先确认无运行中的验收，再 `docker.exe start mooncar-recovery-validation`，不要重复创建。输出目录必须是新目录，所有失败轮次保留；`--trial success|fault|natural` 仅用于单轮诊断。测试结束关闭测试容器，原工作台容器保持运行。

`MOONCAR_EXPERIMENT_ROS_DOMAIN_ID` 控制实验进程及其启动的 ROS 节点，默认 89，接受 0..232 的整数；工作台网页接口不变。每个实验仍具有独立 Gazebo partition。历史策略对照脚本保持固定域 89，不与域 90 的测试混用。

2026-10-06 的最终五轮及 141 项软件回归已完成，见 [闭环验收记录](../reports/evidence/terrain-workbench/recovery-runtime-20261006/README.md)。90 秒无注入试验指观测窗口；预算保护可以提前结束任务，脚本保留实际停止时刻并继续记录静止车辆至窗口结束。原工作台进程未重启，源码中的云台反馈修复在其下次启动时加载。

## R5/v2 云台逐步诊断（2026-10-07，未发布新控制）

本轮固定 A/B/C 参数未通过更严格的云台门槛，仍使用此前验收的 DART / 1 ms。
`sim_control_profile` 的选择函数和严格反馈判定已准备，尚未接入生产 launch 或模型；
旧模型/v1/工作台的展开参数没有变化。证据见
[诊断报告](../reports/evidence/terrain-v2/r5-control-20261006/README.md)。

在独立 Jazzy 测试容器中复验，使用 ROS_DOMAIN_ID=95、独立 Gazebo partition，
安装完整 R5/control/gazebo overlay 和当前已验收的项目 DART 7.8.0 修复库。
不可在原工作台容器中运行。观察器只读，不创建或修改仿真组件；控制力矩记录为
物理计算前的 `JointForceCmd`，并非独立测得的关节反作用力矩。

```bash
source /work/install/setup.bash
cmake -S /source/containers/r5_step_observer -B /work/control-observer -DCMAKE_BUILD_TYPE=Release
timeout 1800 cmake --build /work/control-observer -j2
python3 /source/scripts/run_r5_control_diagnostics.py \
  --output /evidence/new-control-diagnostic \
  --model /evidence/r5-before.sdf \
  --plugin /opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so \
  --observer-plugin /work/control-observer/libmooncar-step-observer.so
```

`r5-before.sdf` 及同名 URDF 来自完整 R5 原配置的冻结展开；网格路径必须对应此容器
的 `/work/install`，不可使用其他版本的模型。脚本拒绝覆盖目录，冻结源文件和哈希，
按原配置、仅 D=0、恢复原配置、A、B、C 的顺序冷启动六轮。单轮墙钟限制 600 秒，
采集异常停止并保留原始数据；三个候选均失败则停止，不选择 D=0 作为替代方案。
通过单项候选也不会自动发布，后续仍须完成两轮静置/扫描、步长及三场景验收。
运行结束从宿主机停止该独立测试容器。

严格反馈参数为 `gimbal_position_tolerance_rad`、`gimbal_velocity_tolerance_rad_s`、
`gimbal_settle_duration_s`、`gimbal_action_timeout_s`，本轮诊断分别为 1°、0.05 rad/s、
0.3 s 和 6 s；默认仍保留此前完成行为。严格模式拒绝缺失/非有限速度、过期反馈、
超过 0.1 仿真秒的反馈断档；超时失败，无底盘旋转替代。

## D=0 独立正式验证（仅报告，不发布配置）

`scripts/run_r5_d0_acceptance.py` 将原完整 R5 的两轴 D 从 0.5 改为 0，仅在新建
测试目录中生成模型。P=20、I=0.1、积分限幅±1、力矩限幅±5 保持原值。
生产模型、三世界默认步长和网页 API 均不改动，不运行 A/B/C 搜索。

另建同镜像的 Jazzy 容器，ROS95、4 CPU、6 GiB、无端口，并在 `/work` 构建
description/control/gazebo 三包。所有软件回归应在物理试验结束后运行。
观察插件需要当前 Gazebo 8 的 headers 和构建工具；原镜像已包含。

```bash
source /work/install/setup.bash
xacro /work/install/mooncar_description/share/mooncar_description/urdf/mooncar_r5.urdf.xacro \
  use_sim_time:=true > /evidence/r5-before.urdf
gz sdf -p /evidence/r5-before.urdf > /evidence/r5-before.sdf
cmake -S /source/containers/r5_step_observer -B /work/d0-observer -DCMAKE_BUILD_TYPE=Release
timeout 1800 cmake --build /work/d0-observer -j2
python3 /source/scripts/run_r5_d0_acceptance.py \
  --output /evidence/new-d0-acceptance --model /evidence/r5-before.sdf \
  --plugin /opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so \
  --observer-plugin /work/d0-observer/libmooncar-step-observer.so \
  --package /work/install/mooncar_gazebo/share/mooncar_gazebo
```

输出目录拒绝覆盖，冻结执行源码和模型哈希。先完成两轮静置和云台往返，
再各两次直行、左右转向，比较 1/0.5 ms；需要时追加 0.25 ms 参考，最后验证
选定步长的三场景各两轮。逐步记录包括实际输入、轮子速度目标、关节位置/速度、
控制力矩命令和车体姿态；观察器只订阅/读取，不发速度或施加力。
输入时间记录是独立 Gazebo 订阅者在物理步中看到消息的时刻，并非 DiffDrive
内部回调的独立计时。每轮资源记录包含渲染、ROS 和记录开销。

正常 22 轮，0.5 ms 备选流程 32 轮，含采集修复补跑最多 34 轮；每轮墙钟
上限 600 s。缺关键数据、运行异常或初始静置/云台未通过则停止，并保存原因。
`summary.json` 的云台、转向和场景结果分别判定；未形成有效配对不得宣称步长通过。
独立测试结束后从宿主机关闭测试容器，保留原工作台。

## 整数步起停校准与正式验收

`check_r5_runtime.py --motion-schedule legacy|paused_steps` 默认 legacy，只有独立测试
显式选择新调度。paused_steps 保留安全中继与原 DiffDrive，在暂停边界确认两次
连续两级速度，用 WorldControl.multi_step 推进并核对实际暂停迭代与观察器 flush。
首次运行需要完整的 1/0.5 ms 左转校准，最多四次新仿真尝试；不是服务返回成功即通过。

在上节独立容器和冻结模型/库准备完成后执行：

```bash
source /work/install/setup.bash
python3 /source/scripts/run_r5_schedule_calibration.py \
  --output /evidence/new-schedule-calibration --model /evidence/r5-before.sdf \
  --plugin /opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so \
  --observer-plugin /work/d0-observer/libmooncar-step-observer.so --step .001
# 第二次仍使用相同校准目录与全部相同源码/模型/库，只改 --step .0005。
# 两个步长完整通过后才启动以下正式入口；失败记录不可删除或重置计数。
python3 /source/scripts/run_r5_d0_acceptance.py \
  --output /evidence/new-aligned-acceptance --model /evidence/r5-before.sdf \
  --plugin /opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so \
  --observer-plugin /work/d0-observer/libmooncar-step-observer.so \
  --motion-schedule paused_steps --calibration-dir /evidence/new-schedule-calibration
```

正式入口要求两个步长的校准与全部冻结执行源码、模型、DART 及观察库签名完全一致。
输入失效立即停止，不新增参考步长绕过；跨签名结果不能配对。校准最多 4 次、正式
最多 34 次，总尝试最多 38 次；每轮 600 墙钟秒。剩余额度无法完成矩阵时应停止。
本轮三次校准后因采集断档及剩余预算不足停止，正式未运行，见
[起停对齐记录](../reports/evidence/terrain-v2/r5-aligned-validation-20261007/README.md)。
此脚本不会更新正式默认值或启用生产 profile。

## R5/v2 异步采集修复与四轮校准

测试入口增加 `--capture-profile legacy|async`，默认 legacy。async 将 RGB/深度
编码交给独立进程、JSONL 写入交给独立线程；保留采样间隔和订阅 QoS。
图片队列最多 8 项/128 MiB，事件队列最多 8192 项。选定样本不能静默丢弃，
溢出、编码/写盘异常及未排空均使该轮失败。结束时确认物理暂停/flush 后排空，
观察记录保留接收、归档时间与写入回执。

独立同镜像容器仍采用 4 CPU、6 GiB、ROS95、无端口，先构建三包并生成原模型和
观察库，再运行以下入口。禁止在原工作台运行。

```bash
source /work/install/setup.bash
python3 /source/scripts/run_r5_capture_campaign.py \
  --output /evidence/new-capture-campaign --model /evidence/r5-before.sdf \
  --plugin /opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so \
  --observer-plugin /work/d0-observer/libmooncar-step-observer.so
```

入口按两轮同步、两轮异步对照执行，全部为 1 ms 左转，然后冻结配置并执行
1/0.5 ms 各两轮独立校准；总计最多八次新尝试，单轮最多 600 墙钟秒。
异步对照或冻结后的校准失败即停止，不启动整套车辆验收。诊断阶段的采集代码错误
可用 `--prior-diagnostics` 指向旧目录，在新目录继续剩余预算；旧尝试全部继承计数，
不得复用其结果补校准，也不能在正式冻结后继续。只有完整两轮单项对照成立才能
报告确定因果结论。

每轮启用只读 Gazebo 原生 IMU 和独立轻量 ROS 观察；`capture_analysis` 分别核对
原生、独立接收与主记录器。Twist 没有消息时间戳，`sim` 明确为回调接收时的 ROS
仿真时钟，DDS 时间戳仅用于跨观察者匹配，不能作为仿真发布时间。
后续正式入口需要显式传 `--capture-profile async`；采集配置已纳入签名，不能混配。
本轮证据见 [采集修复记录](../reports/evidence/terrain-v2/r5-capture-repair-20261007/README.md)。

## R5/v2 图片空档分层诊断

新测试选项 `--image-trace off|headers` 默认 off；headers 显式要求原生只读图片头
观察程序及正式观测模式。记录每个实际收到的 RGB/深度帧头、DDS 时间、接收墙钟、
选中/跳过及先前选中时间；观察写入有界并核对排空回执。原生观察不输出像素。
采集签名包含观察配置和程序哈希，校准/正式入口透传，原默认签名保持一致。

```bash
cmake -S /source/containers/r5_image_observer -B /work/image-observer
cmake --build /work/image-observer -j4
python3 /source/scripts/run_r5_rgb_gap_campaign.py \
  --output /evidence/new-rgb-diagnosis --model /evidence/r5-before.sdf \
  --plugin /opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so \
  --observer-plugin /evidence/observer/libmooncar-step-observer.so \
  --image-observer /work/image-observer/mooncar-image-observer --label baseline-1
```

仅用于独立 ROS95 容器；每次调用新世界和唯一 partition。最多诊断 8 次、冻结校准
4 次，总共 12 次；失败启动也计数，每次 600 墙钟秒。诊断结果需先按授权计划判断，
不能只因脚本可继续就启动校准。通过诊断后可显式传 `--calibrate`，顺序为
1/0.5/1/0.5 ms；冻结失败永久停止，不跨签名补数。`campaign-stop.json` 禁止续跑。
`--image-sampling legacy|nanoseconds` 默认 legacy；整数模式只修满一秒的时间比较，
不改变相对采样规则，不能解决未收到图片造成的空档。本轮未运行该物理候选。

本轮两次诊断复现 RGB 1.7 s、深度 1.9 s 归档超标，原生帧头连续但 ROS 接收
缺少部分帧；具体桥接/DDS/接收根因未确认，未形成修复或启动校准，见
[图片空档报告](../reports/evidence/terrain-v2/r5-rgb-gap-20261007/README.md)。

## 采集与原 M6

```powershell
docker.exe exec mooncar-terrain-validation bash /source/containers/collect.sh
docker.exe exec -e MOONCAR_M6_REPORT=/evidence/m6-repeat.json mooncar-terrain-validation bash /work/scripts/accept_m6.sh --server-only
```

两个命令须分别在没有其他仿真实例时执行。`collect.sh` 启动 SLAM、地形节点和记录器，完成运动、最终保存、停止及档案检查。
原 M6 验收运行默认 NavFn，不启用网页运动许可门控；测试通过不等于实车通过。

## 实验导航

第一个终端运行 `docker.exe exec mooncar-terrain-validation bash /source/containers/navigation.sh`。
第二个终端启动网关：

```powershell
docker.exe exec mooncar-terrain-validation bash -lc 'source /work/ros2_ws/install/setup.bash; python3 /work/scripts/map_workbench.py --sessions /source/reports/evidence/terrain-workbench/gazebo-survey --host 0.0.0.0 --port 8766 --ros-archive /source/reports/evidence/terrain-workbench/gazebo-survey/survey-20260909T151316328964Z.npz --ros-sim-time'
```

浏览器访问 `http://127.0.0.1:8766`，选记录和目标。该启动脚本复用已观测区域 `(3.5,3,0)`。
网关调用 ComputePathToPose、FollowPath 和 Spin。网页须持续续租；停止网关时先发 SIGINT 并等退出，随后停止仿真。
任一时刻只有一个运动指令源。`navigation.sh` 会回收自己的仿真进程组；异常退出后先检查进程，再启动新实例。
运行包络 0.86 × 0.72 m，规划每侧额外留 0.15 m，运行保护不缩小车体；这些值不是实物标定结果。

可复验脚本：`run_navigation.py --goal 4.875 3.125 --mode fixed --output /evidence/compare-fixed.json`，full 同理；须先 source ROS overlay。
`check_navigation.py 4.875 3.125 succeeded /evidence/compare-fixed-check.json` 独立检查真值与零安全指令。
fixed/full 每次须重建同一初始世界，不能把前一次终点作为下一次起点。已有结果各 1 次，不能声称统计提升。
`--obstacle-on-path --obstacle-size 0.2` 为显式障碍实验，会创建并删除自己命名的 Gazebo 模型。
当前成功动态绕行未通过；无路径后安全停止有独立记录，不能将失败的绕行 JSON 改称成功。

## 只展示交付地图

```powershell
docker.exe exec mooncar-terrain-validation python3 /source/scripts/map_workbench.py --sessions /source/reports/evidence/terrain-workbench/gazebo-survey --host 0.0.0.0 --port 8766
```

此命令无 ROS 导航接口。最终保留的演示使用此模式，Gazebo 与运动网关已停止。
