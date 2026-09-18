# 隔离 ROS 验证环境

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
