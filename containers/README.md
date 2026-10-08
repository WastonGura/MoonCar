# 隔离 ROS 验证环境

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

2026-09-10：镜像构建、8 个 ROS 包、35 项 Python 与 C11 核心检查通过，实际 Gazebo 采集、Nav2 执行及 M6 回归已运行。
证据与已知限制见 `docs/terrain_workbench.md` 和 `reports/evidence/terrain-workbench/`。
镜像为 `mooncar-terrain:jazzy`，本次 image config SHA256 为 `07465b330e3ff467eefcb4d1102e32f9efcbcba91be8b2ac33e26997e249db43`。

## 构建与软件检查

```powershell
docker.exe build -f containers/terrain.Dockerfile -t mooncar-terrain:jazzy .
New-Item -ItemType Directory -Force tmp/ros_acceptance
docker.exe run -d --name mooncar-terrain-validation --cpus=4 --memory=6g --shm-size=1g `
  -p 127.0.0.1:8766:8766 `
  --mount type=bind,source=D:/Document/Repositories/MoonCar-mapping,target=/source,readonly `
  --mount type=bind,source=D:/Document/Repositories/MoonCar-mapping/tmp/ros_acceptance,target=/evidence `
  mooncar-terrain:jazzy
docker.exe exec mooncar-terrain-validation bash /source/containers/verify.sh
```

容器使用 ROS_DOMAIN_ID=76、软件渲染、容器网络，不连接物理车或旧 VM。
源工作区只读，构建在 `/work`，证据写入 `/evidence`；软件检查与 Gazebo 验收是两类证据。
若容器已存在，先确认是本项目验证容器，不重复创建。不要同时启动两个同域 Gazebo 世界。

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
