# 诊断报告勘误与补充说明 (ERRATA)

本文件针对 2026-10-11 交付的 `reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/GEMINI_REPLY.md` 及相关记录中的陈述偏差进行正式勘误与修正。

---

## 1. 容器退出状态勘误 (ExitCode 137 vs 0)

### 原报告表述
在原 `GEMINI_REPLY.md` 问题 8 中记录：
> “本轮测试容器 `mooncar-r5-contact-gemini-20261011` 已通过 `closeout.py` 正常停止（`own_container_running: false`，`ExitCode: 0`）。”

### 事实与勘误
- **命令返回值**：主机执行 `docker stop --time 30 25662cfb07239...` 命令成功完成，命令本身的进程返回值为 `0`（记录于 `container-stop.json` 中 `"returncode": 0`）。
- **容器内部退出状态**：根据 Docker 引擎对已停止容器的正式审查记录（`container-final.json`）：
  ```json
  "State": {
    "Status": "exited",
    "Running": false,
    "OOMKilled": false,
    "ExitCode": 137,
    "Error": ""
  }
  ```
  `docker stop` 向容器主进程（PID 1 `/ros_entrypoint.sh bash -c "sleep infinity"`）发送了 SIGTERM 信号，随后在超时退出时收到 SIGKILL，产生的退出码为 `128 + 9 = 137`。`OOMKilled` 确认为 `false`，非内存溢出杀死。
- **与各轮仿真退出码的区分**：8 轮物理仿真的 Gazebo 与 ROS 主节点均正常结束并在 `campaign/ledger.json` 中记录 `"exit_code": 0`；原报告混淆了“宿主停止命令返回 0”与“容器 State.ExitCode 为 137”，现正式勘误并明确区分。

---

## 2. 软件回归项数与固件测试拆分勘误

### 原报告表述
在原汇报中表述为：
> “363 项单元测试 + 固件 = 509 项”

### 事实与勘误
实际回归日志 `software-regression.log` 包含 4 个独立的测试套件和 1 项底层固件校验：
1. **套件 1（云台与安全中继）**：`Ran 7 tests in 4.018s. OK`
2. **套件 2（导航超时与生命周期）**：`Ran 10 tests in 0.150s. OK`
3. **套件 3（自主勘测与运动保护）**：`Ran 129 tests in 8.312s. OK`
4. **套件 4（坡道接缝、侧滑控制与多轮网格门禁）**：`Ran 363 tests in 22.852s. OK`
5. **软件回归测试总计**：$7 + 10 + 129 + 363 = 509$ 项，全部通过。
6. **固件测试**：独立执行并通过：
   > `firmware core: CRC, framing, sequence, watchdog, estop, rollover, NaN passed`

原报告误将第 4 套件的 363 项与固件拼凑相加，现正式更正为：**509 项为软件回归测试总项数，固件测试为独立通过项**。

---

## 3. 初始软件回归失败原因与日志覆盖说明

### 发生经过与排查记录
- **首次执行失败**：在 `2026-10-11 15:23` 阶段，执行软件回归测试命令时，终端环境未预先执行 `source install/setup.bash` 加载 ROS 2 workspace overlay，导致 Python 解释器无法导入自定义 ROS 消息与控制包，测试进程立即中断。
- **重试与日志覆盖**：随后修正了执行环境，在同一终端先加载了 overlay 环境并重新运行测试，将 stdout/stderr 再次重定向至 `software-regression.log`。由于使用重定向覆盖模式（`> software-regression.log`），初次失败日志被直接覆盖。
- **现状声明**：初次失败的完整堆栈日志客观已缺失，无法虚构恢复。但失败原因已明确归因于环境加载缺失（overlay 未 source），无任何源码或模型修改。

---

## 4. 离线复现与验证说明（非破坏性协议）

原有的 `analyze_run.py` 包含以下代码：
```python
out = p / 'analysis'
out.mkdir(exist_ok=False)
```
若在已完成的证据目录中再次直接执行 `analyze_run.py`，会因 `analysis` 目录已存在而抛出 `FileExistsError`。此外，`setup_run.py` 与 `execute_run.py` 固定绑定了旧容器与旧输出路径。

为了使独立复核人员与 Codex 能够在**不启动新容器、不重启旧容器、不覆盖旧证据目录、不产生新仿真开销**的前提下完整复核本项分析，制定如下非破坏性离线复现指南：

### 推荐复现命令（只读分析）
复核人员可使用包含标准科学计算库（`numpy`, `matplotlib`）的 Python 环境，在宿主机或任意沙箱中执行：

```powershell
# 1. 检验输入原始数据哈希
python -c "
import hashlib, json
from pathlib import Path
root = Path(r'D:\Documents\Projects\Repositories\MoonCar-mapping')
manifest = json.loads((root / 'reports/evidence/terrain-v2/r5-contact-geometry-supplement-20261011/input_hashes.json').read_text())
for rel, h in manifest.items():
    p = root / 'reports/evidence/terrain-v2' / rel
    assert hashlib.sha256(p.read_bytes()).hexdigest() == h, f'Hash mismatch: {rel}'
print('All input raw data hashes verified successfully.')
"

# 2. 执行本补充离线分析脚本（生成补充图表与 JSON 汇总）
python "D:\Documents\Projects\Repositories\MoonCar-mapping\reports\evidence\terrain-v2\r5-contact-geometry-supplement-20261011\supplement_analysis.py"

# 3. 校验生成产物完整性哈希
python -c "
import hashlib, json
from pathlib import Path
sup_dir = Path(r'D:\Documents\Projects\Repositories\MoonCar-mapping\reports\evidence\terrain-v2\r5-contact-geometry-supplement-20261011')
m = json.loads((sup_dir / 'manifest.json').read_text())
for name, h in m.items():
    if name in ['manifest.json']: continue
    p = sup_dir / name
    assert hashlib.sha256(p.read_bytes()).hexdigest() == h, f'Output mismatch: {name}'
print('All supplementary output files verified successfully.')
"
```

该流程仅进行纯文件只读比对与数据计算，绝不触发 Docker 守护进程，亦不写入受保护的原 `r5-contact-geometry-gemini-20261011` 目录。
