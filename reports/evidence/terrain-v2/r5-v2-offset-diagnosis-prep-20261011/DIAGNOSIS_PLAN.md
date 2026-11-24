# 原 v2 约 1 厘米转向偏移：两轮诊断执行工具与方案报告 (DIAGNOSIS_PLAN)

- **任务依据**：`reports/evidence/terrain-v2/r5-v2-offset-tools-review-20261011/GEMINI_FIX_EXECUTION.md`
- **Codex 审查基线**：`reports/evidence/terrain-v2/r5-v2-offset-tools-review-20261011/README.md` 与 `review-results.json`
- **交付目录**：`reports/evidence/terrain-v2/r5-v2-offset-diagnosis-prep-20261011`
- **执行承诺**：**本阶段仅交付源码、世界副本、离线验证与检查输出，绝不运行 Gazebo，不消耗仿真次数，静待授权。**

---

## 一、两轮独立世界副本与真实控制链接入

### 1. 独立世界副本生成与精确差异
完全保留独立世界副本设计，不在宿主机修改正式模型：
- `worlds/world_shift_x05.sdf` (SHA-256: `8b549b1bd531cf67a076dab6a7e9bcd068d59edabb067a850904f128d51c3c0f`)
- `worlds/world_restore_x00.sdf` (SHA-256: `e71ea47b0a77507a96db84301a4dd1951115cbebf05b550032062e028abea3c7`)

与原轮次 `14-pure_turn-ramp-1000us-left/world.sdf` 的统一差异保持纯粹：
1. **恢复世界 (`world_restore_x00.sdf`)**：
   - 出生位姿严格保留 `<pose>0 0 .03 0 0 0</pose>`（**0 差异**）；
   - 仅将观测器与控制器输出路径重定向至 `/work/restore_x_0-*.csv`。
2. **平移世界 (`world_shift_x05.sdf`)**：
   - 仅多一个 X 差异：`<pose>0.05 0 .03 0 0 0</pose>`（严格保留原 $Z = 0.03\text{ m}$，绝不误用轮心名义离地高 $0.076\text{ m}$）；
   - 输出路径重定向至 `/work/test_shift_x_plus_5cm-*.csv`。

### 2. 真实执行链接入与单轮执行器
依据原控制链 `check_r5_turn_gate_validation.py` → `check_r5_turn_slip_round.py` → `check_r5_origin_round.py` → `check_r5_runtime.py`：
- 实现单轮入口 [`check_r5_v2_offset_round.py`](check_r5_v2_offset_round.py)：
  - 动态挂接 `runtime.prepare`，将指定的独立世界副本无缝注入运行环境；
  - 运行完整的 `check_r5_turn_gate_validation.main()`，复用原 `paused_steps` 调度、安全中继与全套采集验证流程。
- **几何筛选与数据资格严格分清**：
  - 原转弯门禁对位移超过 10 mm 报 `ValueError('Existing 1 cm turn screen failed')`。单轮执行器将其识别并登记为**预期的诊断筛选结果**（`diagnostic_screen_exceeded = True, eligible = True`），而绝不混同于输入故障或采集崩溃；
  - 数据缺失、`state=-1` 未知状态、停机失控等真正物理与采集失败则立即判定资格不合格（`eligible = False`），严格执行即刻中止规则。
- 调度器入口 [`run_diagnosis_campaign.py`](run_diagnosis_campaign.py)：
  - 支持 `--dry-run` 离线预检（展示容器规范 `mooncar-r5-v2-offset-diagnosis-20261011`、Jazzy、ROS95、4 CPU/6G/512M shm 与 POSIX 命令）；
  - 支持 `--run-round` 与 `--run-all`，内嵌 `--authorized` 安全锁，且包含子进程管理与停止规则逻辑。

---

## 二、接触源完整性与资格检查

[`evaluate_diagnosis_run.py`](evaluate_diagnosis_run.py) 资格检查严格对齐：
1. **缺失检查**：每步四轮必须全覆盖，若缺记录直接判定 `qualified = False`，失触计数显式为 `None`，严禁补零。
2. **未知状态检查**：`state` 必须为 0 或 1，出现 `-1` 判定不合格。
3. **显式空快照**：仅当数据完整且 `state == 0` 时记录为空快照区间，与几何离地计算解耦。
4. **有效浮点检查**：激活接触时法向、深度、受力必须全部为有限浮点数。
5. **覆盖保护**：CLI 默认拒绝覆盖已有输出文件（未带 `--overwrite` 时抛出错误并以状态码 1 退出）。

---

## 三、因果链证据完整保存（邻边求交与 17019 事件）

升级后的评估器完整提取物理因果证据：
1. **四轮对实际网格邻边求交**：
   - 输出四轮（FL, FR, RL, RR）最低母线对全部邻边（目标对角接缝、中央 X 轴、中央 Y 轴、中央主对角线、第四象限对角线）的相交窗口 `grid_edge_overlap_windows_all_wheels`。
2. **完整事件与序列独立保存**：
   - 将全部 4244 条关键事件及每步受力、局部冲量 $\vec{J}_{\text{local}}$、质心速度变化 $\Delta \vec{v}_{\text{com}}$ 完整写入独立 JSON Lines 文件；
   - 在汇总 JSON 的 `sample_critical_events` 中，严格保留初始起动段（5201–5210）、接缝交会窗口与**关键突变事件 17019** 及其附近完整受力时序。

---

## 四、离线替身验证与就绪状态

通过独立的离线替身测试脚本 [`test_execution_mock.py`](test_execution_mock.py)，在不启动 Gazebo 的前提下完成闭环自测：
- [PASS] 调度器 dry-run 容器、Jazzy 与 ROS95 规范输出正确；
- [PASS] 替身子进程正确消费 `world_shift_x05.sdf` 并将控制参数路由至控制链；
- [PASS] 10 mm 筛选超限正确识别为有效诊断输出而非程序崩溃；
- [PASS] 模拟数据缺失时即刻触发停止规则，终止两轮序列；
- [PASS] CLI 覆盖探测正确拒绝覆盖并退出；
- [PASS] 基准第 14 轮准确输出 17019 突变与全部邻边相交窗口。

### 结论：**已完全具备安全执行两轮物理诊断的全部工程与工具条件。**
> **当前状态承诺**：全部文件处于静态离线就绪状态。**未启动 Gazebo、未启动 Docker 容器、未执行任何物理仿真**，静待用户/Codex 授权！

---

## 交付文件核验清单

交付目录：
`D:\Documents\Projects\Repositories\MoonCar-mapping\reports\evidence\terrain-v2\r5-v2-offset-diagnosis-prep-20261011`

1. `DIAGNOSIS_PLAN.md`：本诊断执行工具与方案报告。
2. `campaign_plan.json`：精确机器可读配置（28.2s 调度、独立世界路径、出生姿态）。
3. `worlds/world_shift_x05.sdf`：独立平移世界副本（$X=0.05\text{ m}, Z=0.03\text{ m}$）。
4. `worlds/world_restore_x00.sdf`：独立恢复世界副本（$X=0.00\text{ m}, Z=0.03\text{ m}$）。
5. `run_diagnosis_campaign.py`：两轮调度主入口（含 `--dry-run`、`--run-round`、`--run-all` 与安全锁）。
6. `check_r5_v2_offset_round.py`：单轮物理执行入口（注入独立世界，接入真实控制链）。
7. `evaluate_diagnosis_run.py`：支持数据资格、四轮邻边求交、保存全部 4244 事件、输出 17019 与防覆盖机制。
8. `test_execution_mock.py`：离线替身子进程与自检验证脚本。
9. `README.md`：交付包说明与操作指引。
10. `manifest.json`：全目录文件 SHA-256 校验清单。
