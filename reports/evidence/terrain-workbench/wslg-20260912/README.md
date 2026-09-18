# WSLg 接入检查 · 2026-09-12

- 使用现有 `mooncar-terrain:jazzy` 镜像，新建可删除的 GUI 客户端 `mooncar-wslg-viewer`，执行 `gz sim -g`。
- Windows 实际出现 `Gazebo Sim (docker-desktop)` 窗口，显示 MoonCar、石块和坡台；实际打开菜单并观察到场景缩放变化。检测到用户开始操作后停止自动鼠标输入。用户随后明确确认“可以正常缩放、拖动”，交互由用户手动验证通过。
- 实验 `experiment-a608e74eefb5` 的网页状态为 recording，深度传感器活跃，车体定位约 (3.5, 3.0)。本次接入检查未执行导航。
- `scripts/check_wslg.ps1` 通过：重复打开复用同一 GUI 容器，实验 ID 保持不变，Windows 8765 前端连接正常，WSLg socket 只读挂载。
- 运行前只有空闲的后端服务；本次新建一份验证实验，未重启后端、未替换旧档案。未安装依赖、未拉取镜像、未修改系统网络或显示配置。
- 原容器继续以 Xvfb 提供网页现场画面；WSLg 窗口是同一实验的另一个客户端。软件渲染继续沿用，未测 GPU 加速或交互延迟基准。

`state.json` 与 `viewer.json` 保存本次接口状态和 GUI 客户端配置摘要。使用方式见 `web/README.md` 的 WSLg 部分。
