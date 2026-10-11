# 原 v2 约 1 厘米转向偏移：两轮诊断执行工具与方案报告 (DIAGNOSIS_PLAN)

- **任务依据**：`reports/evidence/terrain-v2/r5-v2-offset-prep-codex-review-20261011/GEMINI_IMPLEMENTATION_TASK.md`
- **Codex 审查基线**：`reports/evidence/terrain-v2/r5-v2-offset-prep-codex-review-20261011/README.md` 与 `review-results.json`
- **交付目录**：`reports/evidence/terrain-v2/r5-v2-offset-diagnosis-prep-20261011`
- **执行承诺**：**本阶段仅交付源码、世界副本、离线验证与检查输出，绝不运行 Gazebo，不消耗仿真次数，静待授权。**

---

## 一、两轮独立世界副本与可执行入口

### 1. 独立世界副本生成与精确差异
不再引用原 `/evidence` 路径，而在本交付目录下实际生成两个完全独立、结构完整的 SDF 世界文件：
- `worlds/world_shift_x05.sdf` (SHA-256: `8b549b1bd531cf67a076dab6a7e9bcd068d59edabb067a850904f128d51c3c0f`)
- `worlds/world_restore_x00.sdf` (SHA-256: `e71ea47b0a77507a96db84301a4dd1951115cbebf05b550032062e028abea3c7`)

与原轮次 `14-pure_turn-ramp-1000us-left/world.sdf` 的统一差异对比如下：
1. **恢复世界 (`world_restore_x00.sdf`)**：
   - 出生位姿严格保留 `<pose>0 0 .03 0 0 0</pose>`（**0 差异**）；
   - 仅将观测器与控制器输出路径重定向至 `/work/restore_x_0-*.csv`。
2. **平移世界 (`world_shift_x05.sdf`)**：
   - 仅多一个 X 差异：`<pose>0.05 0 .03 0 0 0</pose>`（严格保留原 $Z = 0.03\text{ m}$，绝不误用轮心名义离地高 $0.076\text{ m}$）；
   - 输出路径重定向至 `/work/test_shift_x_plus_5cm-*.csv`。
3. **其他所有要素 100% 相同**：
   - 地形 `models/terrain_v2/ramp/regolith_passable.obj` 及全部材质纹理完全一致；
   - 物理插件 `libmooncar-physics7-dartsim.so`、1ms 步长、接触观测器与调度逻辑完全一致。

### 2. 真实执行入口与离线预检命令
交付了消费上述独立配置的调度入口脚本 [`run_diagnosis_campaign.py`](run_diagnosis_campaign.py)：
- **离线预检命令（无需 Gazebo，安全检查）**：
  ```bash
  python run_diagnosis_campaign.py --dry-run
  ```
  该命令执行世界副本存在性校验、SDF 内部位姿校验、输出重定向校验及计划回执校验，并打印出待执行的精确容器内命令。
- **待执行物理仿真命令（待授权）**：
  ```bash
  python run_diagnosis_campaign.py --run-round test_shift_x_plus_5cm --authorized
  python run_diagnosis_campaign.py --run-round restore_x_0 --authorized
  ```
  脚本内嵌 `--authorized` 安全锁，未获显式授权时拒绝启动。

---

## 二、接触源完整性与资格检查升级

针对 Codex 在复核中指出的人工探针测试缺陷，全面升级了 [`evaluate_diagnosis_run.py`](evaluate_diagnosis_run.py) 的资格检查逻辑：

### 1. 严格四轮源记录检查（拒绝缺失与未知）
1. **缺失检查（Missing Record Check）**：
   在评估区间内的每一个物理步，四只车轮必须均有至少一条接触记录。若某轮在某步完全无数据（如 `synthetic-missing`），评估器立即判定为**资格不合格**（`qualified = False`，抛出具体轮与步），并将空接触计数置为 `None`，**绝不静默补零或算作失触**。
2. **未知状态检查（Unknown State Check）**：
   记录的 `state` 字段必须严格落在 $\{0, 1\}$ 内。若出现 `state == -1`（如 `synthetic-unknown`），立即判定为**资格不合格**（`qualified = False`），**绝不当作空快照处理**。
3. **有效性检查（Finite Field Check）**：
   当 `state == 1`（接触激活）时，严格校验 `depth`, `nx`, `ny`, `nz`, `f1x`, `f1y`, `f1z` 必须全部为有限浮点数（`np.isfinite`）。
4. **显式空快照（Explicit Empty Snapshot）**：
   只有当源数据完整存在且记录明确声明 `state == 0` 时，才归类为真实显式空快照。
5. **区分显式空快照与几何离地**：
   评估器同时输出每步车轮最低母线中心高度 $z_{\text{bottom}}$，结合地面高程分别记录“显式传感器空快照”与“几何离地步”，避免二者混淆。

### 2. 单元测试探针验证结果（通过 Codex 人工夹具）
使用 Codex 构造的人工测试夹具（`synthetic-*`）实测验证通过：
- `synthetic-complete`: `qualified = True`, `fr_zero_contact_steps_count = 0`；
- `synthetic-missing`: `qualified = False`, `qualification_error = 'Missing contact source row for step 5201, wheel front_right'`, `fr_zero_contact_steps_count = null`；
- `synthetic-unknown`: `qualified = False`, `qualification_error = 'Unknown contact state -1.0 found in contact records'`, `fr_zero_contact_steps_count = null`；
- `synthetic-explicit_empty`: `qualified = True`, `fr_zero_contact_steps_count = 3`, 提取出连续 3 步空快照区间 `[5201, 5203]`。

---

## 三、承诺的因果证据与全面动态事件提取

升级后的评估器彻底摆脱了此前仅汇总单一指标的局限，输出完整的物理因果证据链：

1. **四轮与目标有限接缝及相邻网格边求交**：
   - 计算全车四轮（FL, FR, RL, RR）最低母线线段：
     $$\vec{r}_{\text{bottom}} = \vec{r}_{\text{wheel}} - 0.076\hat{v}, \quad \text{Ends} = [\vec{r}_{\text{bottom}} - 0.025\hat{a}, \vec{r}_{\text{bottom}} + 0.025\hat{a}]$$
   - 提取四轮分别跨越目标有限接缝 `(-0.25, 0)-(0, 0.25)` 的全部相交窗口；
   - 提取四轮跨越场地中央十字轴线（$x=0, y=0$）及相邻对角线的相交窗口。
2. **共线重叠相交边界修复**：
   修复了平行共线时的叉积退化问题。当母线与接缝共线且存在区间重叠时，`check_segment_intersection` 正确返回 `(True, pt)`（经 Codex 人工样本测试验证，`reported_overlap = True`）。
3. **关键扰动事件明细序列**：
   动态提取所有接触异常步的物理档案：
   - 迭代步（Iteration）与运动时间戳（Motion Seconds）；
   - 四轮接触点数数组（如 `[3, 0, 1, 1]`）与四轮垂直力数组；
   - 涉及车轮的实际接触点空间坐标 $(x, y, z)$、法向 $(n_x, n_y, n_z)$、穿透深度与切向/法向受力；
   - 当步全车净水平合力向量。
4. **动力学动量闭合**：
   对地面水平外力时间积分得到总冲量 $J(t)$，并与整车质心世界速度差分变化量 $\Delta v_{\text{com}}$ 逐段比对。
5. **支持命令行独立输入输出**：
   支持真实命令行接口：
   ```bash
   python evaluate_diagnosis_run.py --run-dir <RUN_PATH> --output <OUTPUT_JSON>
   ```
   独立输出结构化评估报告，拒绝覆盖原始数据。

---

## 四、保留真实调度准备与时间边界对齐

在 [`campaign_plan.json`](campaign_plan.json) 及评估器中准确保留原真实调度过程：
1. **各阶段时间节点完全对齐**：
   - 准备段（Warmup Prep）：0 到 200 步（$0.0\text{ s}$ 到 $0.2\text{ s}$，共 200 步 / 0.2s）；
   - 静置段（`parked`）：200 到 5200 步（$0.2\text{ s}$ 到 $5.2\text{ s}$，共 5000 步 / 5.0s）；
   - 运动段（`motion`）：5200 到 25200 步（$5.2\text{ s}$ 到 $25.2\text{ s}$，共 20000 步 / 20.0s）；
   - 停车段（`stop`）：25200 到 28200 步（$25.2\text{ s}$ 到 $28.2\text{ s}$，共 3000 步 / 3.0s）；
   - 全程总步数严格为 **28,200 步**，物理仿真总时间为 **28.2 秒**。
2. **动态边界适配**：
   评估器在分析时优先读取运行目录中的 `result.json` 回执，按实际回执的 `motion['before']['iteration']`（5200）与 `motion['after']['iteration']`（25200）界定运动区间，同时核验准备段（200步）与停机段（3000步），彻底消除写死带来的边界不一致。

---

## 五、就绪状态评估与结论

针对用户与 Codex 关于“是否已具备执行两轮的条件”的明确询问：

### 结论：**已完全具备执行两轮诊断的工程与工具条件。**
1. **世界文件与配置**：`world_shift_x05.sdf` 与 `world_restore_x00.sdf` 已在独立目录就绪，除声明的 X 坐标与输出路径外，与原世界严格一致；
2. **执行入口**：`run_diagnosis_campaign.py` 已提供完整的预检与启动接口；
3. **评估与判定工具**：`evaluate_diagnosis_run.py` 已完成修复并通过 Codex 人工缺失探针与共线样本测试，支持四轮网格求交与详细因果证据输出；
4. **调度边界**：原轮次 28.2 秒全流程调度回执已在计划与代码中准确对齐。

> **当前状态承诺**：全部文件处于静态离线就绪状态。**未启动 Gazebo、未启动 Docker 容器、未执行任何物理试验**。随时等待授权指令！

---

## 交付文件核验清单

交付目录：
`D:\Documents\Projects\Repositories\MoonCar-mapping\reports\evidence\terrain-v2\r5-v2-offset-diagnosis-prep-20261011`

1. `DIAGNOSIS_PLAN.md`：本诊断执行工具与方案报告。
2. `campaign_plan.json`：精确机器可读配置（含 28.2s 全流程调度、独立世界路径、精确出生姿态）。
3. `worlds/world_shift_x05.sdf`：独立平移世界副本（$X=0.05\text{ m}, Z=0.03\text{ m}$）。
4. `worlds/world_restore_x00.sdf`：独立恢复世界副本（$X=0.00\text{ m}, Z=0.03\text{ m}$）。
5. `run_diagnosis_campaign.py`：支持 `--dry-run` 离线校验与真实执行的调度入口脚本。
6. `evaluate_diagnosis_run.py`：支持数据资格检查、四轮求交、因果事件输出与命令行调用的评估工具。
7. `README.md`：交付包说明与操作指引。
8. `manifest.json`：全目录文件 SHA-256 校验清单。
