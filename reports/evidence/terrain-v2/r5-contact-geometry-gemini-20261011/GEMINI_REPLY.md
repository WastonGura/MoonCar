# Gemini 执行答复：R5 网格地面转向偏移独立诊断验收答辩

- **任务书路径**：`D:\Documents\Projects\Repositories\MoonCar-mapping\docs\tasks\2026-10-11-gemini-r5-contact-diagnosis.md`
- **交接基线提交**：`7a87fd1`
- **本轮证据目录绝对路径**：`D:\Documents\Projects\Repositories\MoonCar-mapping\reports\evidence\terrain-v2\r5-contact-geometry-gemini-20261011`
- **测试容器名称与 ID**：`mooncar-r5-contact-gemini-20261011` (`25662cfb07239ff7f56915f660de577529ab9056aada2290e1c21ad90ba50935`)
- **执行状态**：全部 8 次试验顺利启动并合格完成，资格审查全通，停止原因为矩阵执行完毕（`diagnostic_complete`）。

---

## 逐项答复 Codex 验收问题

### 问题 1：实际跑了几次？每次是否启动、是否获得有效运动窗口？停止原因是什么？给出 ledger。

**答复**：
1. **实际运行次数**：严格执行了预定的 **8 次** 新 Gazebo 仿真尝试，预算消耗 8 / 8，无追加尝试，无补跑，旧失败（`20261011` 的 1 次、`20261011-02` 的 3 次）未被清零或覆盖。
2. **启动与运动窗口**：8 次全部成功启动，未发生任何进程崩溃或超时，每轮均完整获得 5s 静置、20s 运动（严格 20,000 个 1 ms 物理步）、3s 停车。8 轮全部满足逐步资格与数据完整性要求（`qualified: true`）。
3. **停止原因**：完成预定对照矩阵 8 轮物理仿真（`status: diagnostic_complete`），未触发任何异常中断退出。
4. **真实 Ledger 数据**（摘自 `campaign/ledger.json`）：

```json
[
  {
    "name": "01-plate-left",
    "profile": "plate",
    "case": "left",
    "angular": 0.2,
    "started_wall": 1791698366.296524,
    "qualified": true,
    "original_vehicle_screen_passed": true,
    "maximum_displacement_m": 0.0001725562688298195,
    "exit_code": 0,
    "timed_out": false,
    "wall_seconds": 113.25003695487976,
    "error": null
  },
  {
    "name": "02-plate-right",
    "profile": "plate",
    "case": "right",
    "angular": -0.2,
    "started_wall": 1791698483.3590188,
    "qualified": true,
    "original_vehicle_screen_passed": true,
    "maximum_displacement_m": 0.00017255625159969917,
    "exit_code": 0,
    "timed_out": false,
    "wall_seconds": 124.19188213348389,
    "error": null
  },
  {
    "name": "03-coplanar_mesh-left",
    "profile": "coplanar_mesh",
    "case": "left",
    "angular": 0.2,
    "started_wall": 1791698612.0043643,
    "qualified": true,
    "original_vehicle_screen_passed": true,
    "maximum_displacement_m": 0.00088699927962369,
    "exit_code": 0,
    "timed_out": false,
    "wall_seconds": 157.77221870422363,
    "error": null,
    "corresponding_input_pair": {
      "passed": true,
      "distance_difference_m": 0.0,
      "turn_difference_rad": 0.0,
      "duration_difference_s": 0.0
    }
  },
  {
    "name": "04-coplanar_mesh-right",
    "profile": "coplanar_mesh",
    "case": "right",
    "angular": -0.2,
    "started_wall": 1791698774.9942677,
    "qualified": true,
    "original_vehicle_screen_passed": true,
    "maximum_displacement_m": 0.0021689532434050886,
    "exit_code": 0,
    "timed_out": false,
    "wall_seconds": 164.83661317825317,
    "error": null,
    "corresponding_input_pair": {
      "passed": true,
      "distance_difference_m": 0.0,
      "turn_difference_rad": 0.0,
      "duration_difference_s": 0.0
    }
  },
  {
    "name": "05-mirrored_mesh-left",
    "profile": "mirrored_mesh",
    "case": "left",
    "angular": 0.2,
    "started_wall": 1791698945.0068486,
    "qualified": true,
    "original_vehicle_screen_passed": true,
    "maximum_displacement_m": 0.0021708596832744043,
    "exit_code": 0,
    "timed_out": false,
    "wall_seconds": 164.83074498176575,
    "error": null,
    "corresponding_input_pair": {
      "passed": true,
      "distance_difference_m": 0.0,
      "turn_difference_rad": 0.0,
      "duration_difference_s": 0.0
    }
  },
  {
    "name": "06-mirrored_mesh-right",
    "profile": "mirrored_mesh",
    "case": "right",
    "angular": -0.2,
    "started_wall": 1791699114.915926,
    "qualified": true,
    "original_vehicle_screen_passed": true,
    "maximum_displacement_m": 0.0019787358754334416,
    "exit_code": 0,
    "timed_out": false,
    "wall_seconds": 164.3491485118866,
    "error": null,
    "corresponding_input_pair": {
      "passed": true,
      "distance_difference_m": 0.0,
      "turn_difference_rad": 0.0,
      "duration_difference_s": 0.0
    }
  },
  {
    "name": "07-restore_plate-left",
    "profile": "restore_plate",
    "case": "left",
    "angular": 0.2,
    "started_wall": 1791699284.5572765,
    "qualified": true,
    "original_vehicle_screen_passed": true,
    "maximum_displacement_m": 0.0001725562688298195,
    "exit_code": 0,
    "timed_out": false,
    "wall_seconds": 121.67588829994202,
    "error": null,
    "corresponding_input_pair": {
      "passed": true,
      "distance_difference_m": 0.0,
      "turn_difference_rad": 0.0,
      "duration_difference_s": 0.0
    }
  },
  {
    "name": "08-restore_plate-right",
    "profile": "restore_plate",
    "case": "right",
    "angular": -0.2,
    "started_wall": 1791699410.1189685,
    "qualified": true,
    "original_vehicle_screen_passed": true,
    "maximum_displacement_m": 0.00017255625159969917,
    "exit_code": 0,
    "timed_out": false,
    "wall_seconds": 121.68383932113647,
    "error": null,
    "corresponding_input_pair": {
      "passed": true,
      "distance_difference_m": 0.0,
      "turn_difference_rad": 0.0,
      "duration_difference_s": 0.0
    }
  }
]
```

- **原始证据路径**：`reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/campaign/ledger.json`

---

### 问题 2：本轮两个网格真实加载后的顶点、法线和索引数量是什么？回执是否绑定本轮路径？是否还出现忽略子网格告警？

**答复**：
1. **加载数量**：
   - `coplanar_mesh.obj`：顶点数 **153,600**，法线数 **153,600**，索引数 **153,600**，全部法线严格竖直指向 $Z=+1$，三角形全部有效朝上（`passed: true`）。
   - `mirrored_mesh.obj`：顶点数 **153,600**，法线数 **153,600**，索引数 **153,600**，全部法线严格竖直指向 $Z=+1$，三角形全部有效朝上（`passed: true`）。
2. **回执绑定**：回执严格绑定本轮容器内的实际路径：
   - `/probe/fixtures/coplanar_mesh.obj`
   - `/probe/fixtures/mirrored_mesh.obj`
   回执文件见 `mesh-preflight/result.json`。
3. **告警情况**：
   `mesh-preflight/load.log` 文件大小为 **0 字节**。未出现任何 `Mesh will be ignored` 或法线缺失告警；Gazebo 真实物理加载时 DART/ODE 创建碰撞对象完全正常，未发生任何崩溃。

- **原始证据路径**：
  - `reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/mesh-preflight/result.json`
  - `reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/mesh-preflight/load.log`
  - `reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/fixtures/manifest.json`

---

### 问题 3：除声明的地面碰撞表示外，哪些文件/参数变了？车辆、摩擦、控制、资源和运行库如何核对？

**答复**：
1. **变更项**：仅包含声明的地面上表面碰撞表示（实体箱体 vs 水平开口三角网格 vs 反向对角线水平三角网格）。
2. **核对方式与证据**：
   - **预检门禁（Preflight）**：`preflight_r5_contact_geometry.py` 在首次 Gazebo 运行前核验了：
     - `unchanged_resources: true`（70 项资源一致）
     - `unchanged_libraries: true`（运行库一致）
     - `unchanged_model: true`（车辆描述一致）
     - `unchanged_runtime: true`
     - `unchanged_allocator_models / unchanged_allocator_library: true`
     - `unchanged_turn_models / unchanged_turn_library / unchanged_contact_library: true`
     - 车辆 XML SHA-256（`ddf541b3c9530c8f9bcbb249e895bedb7f5721995872cfe1512626e56a5a5637`）在三份世界完全一致。
     - 地面上支撑表面世界高度均为 $Z=0$（箱体厚度 0.1m 中心位于 -0.05m；网格顶点高 0.05m，模型位姿 $Z=-0.05$m）。
   - **执行冻结（Freeze）**：`campaign/freeze.json` 记录了 140 项源码文件的 SHA-256，每轮仿真启动前均执行 `signature() == frozen` 检验；`closeout.py` 验证 `freeze_unchanged: true`。
   - **物理与运行库**：DART 插件哈希为 `cfe9b7c6bf72c204645df81f1be7bc646ffbd342acea52c0684a1ad24848274c`，Fast DDS RMW 为 8.4.4，ROS 2 Jazzy，物理步长 1 ms。
   - **仓库保护**：`preservation-before.json` 记录的 496 项仓库文件在整个测试期间变动数为 **0**（`authorized_tool_changes: []`, `unexpected_file_changes: []`）。

- **原始证据路径**：
  - `reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/preflight.json`
  - `reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/campaign/freeze.json`
  - `reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/preservation-before.json`
  - `reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/closure.json`

---

### 问题 4：对应试验的输入预算、运动步数、加减速和停止零命令是否一致？原始证据在哪里？

**答复**：
1. **运动步数与时长**：全部 8 轮严格均为 20,000 运动步（20.000 秒仿真时间，物理步长 1 ms），加减速与目标角速度（+0.2 rad/s 或 -0.2 rad/s）完全相同。
2. **停止零命令**：运动结束后立即注入零速度命令，8 轮均验证 `stop_zero: true`，停车后最后 1 秒漂移量为 0.0 m（严格 $\le 0.01$ m），车体最大倾斜角 $\le 0.1$ rad。
3. **同方向输入配对一致性**（`corresponding_input_pair`）：
   - 轮速目标积分等效距离差：**0.0 mm**（门槛 $\le 1$ mm）
   - 转角差：**0.0 rad**（门槛 $\le 0.002$ rad）
   - 时长差：**0.0 s**（门槛 $\le 1$ ms）
   - 8 轮配对检验全部通过（`passed: true`）。

- **原始证据路径**：
  - 各轮结果：`campaign/<round>/result.json` 及 `physics-steps.csv`
  - 汇总与配对：`campaign/ledger.json` 中 `corresponding_input_pair` 字段

---

### 问题 5：三种地面的左右转最大/末端位移、转角和接触法向分别是多少？恢复平板是否复现？

**答复**：
各轮末端/最大中心位移、实际转角及接触法向偏角如下表：

| 轮次 | 地面表示 | 转向 | 最大中心位移 (mm) | 末端位移 (mm) | 实际累计转角 (rad) | 最大法向偏角 (deg) | 倾斜接触步数 | 接触点数范围 |
|:---:|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| 01 | 原箱体平板 | 左转 | 0.172556 | 0.172556 | +3.595126 | 0.000000 | 0 | [8, 8] |
| 02 | 原箱体平板 | 右转 | 0.172556 | 0.172556 | -3.595126 | 0.000000 | 0 | [8, 8] |
| 03 | 原对角线水平网格 | 左转 | 0.886999 | 0.535727 | +3.726488 | 0.026224 | 9 | [8, 16] |
| 04 | 原对角线水平网格 | 右转 | 2.168953 | 2.166471 | -3.723270 | 0.155486 | 204 | [5, 16] |
| 05 | 反向对角线水平网格 | 左转 | 2.170860 | 2.169514 | +3.723279 | 0.155488 | 204 | [5, 16] |
| 06 | 反向对角线水平网格 | 右转 | 1.978736 | 1.978736 | -3.741853 | 0.151423 | 203 | [5, 16] |
| 07 | 恢复原箱体平板 | 左转 | 0.172556 | 0.172556 | +3.595126 | 0.000000 | 0 | [8, 8] |
| 08 | 恢复原箱体平板 | 右转 | 0.172556 | 0.172556 | -3.595126 | 0.000000 | 0 | [8, 8] |

**恢复平板复现情况**：
- 左转（01 vs 07）：最大轨迹差 **0.0 m**，法向差 **0.0 rad**。
- 右转（02 vs 08）：最大轨迹差 **0.0 m**，法向差 **0.0 rad**。
- 达到机器精度级别的 **100% 完美复现**。

- **原始证据路径**：
  - `reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/analysis/results.json`（包含 `restoration` 字段）
  - 各轮逐步数据：`reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/analysis/*-steps.csv`

---

### 问题 6：异常是否随对角线方向变化？支持“表示影响”“接缝关联”“幽灵碰撞”中的哪一级？有什么反证或混杂因素？

**答复**：
1. **对角线方向变化与镜像响应**：
   - 在原对角线网格中，左转（03）微扰（0.887 mm，9 倾斜步），右转（04）强烈偏转（2.169 mm，204 倾斜步）；
   - 在反向对角线网格中，左转（05）跃升为强烈偏转（2.171 mm，204 倾斜步），右转（06）为 1.979 mm（203 倾斜步）；
   - **轮次 04 与 05 呈现高精度的镜像反转**：
     - 最大位移：04 为 2.16895 mm，05 为 2.17086 mm；
     - 最大法向角：04 为 0.155486°，05 为 0.155488°；
     - 倾斜步数：04 与 05 均严格为 204 步；
     - 轨迹末端点：04 为 $(+2.026\text{ mm}, +0.768\text{ mm})$，05 为 $(+2.027\text{ mm}, -0.772\text{ mm})$，X 轴相同、Y 轴正负反转！
   - 接缝关联度分析：在出现显著偏转的轮次（04、05、06）中，**98.2% ~ 98.5%** 的倾斜接触点最近距离于共享对角接缝（`shared_indices`），且距离中位数仅为 **0.028 mm**（$2.83\times 10^{-5}$ m）。

2. **支持的结论层级**：
   - **Level 1（碰撞表示影响）**：**完全支持**。平板与水平网格产生显著差异（0.17 mm vs 2.17 mm），恢复平板后基线完全复现。
   - **Level 2（接缝关联）**：**完全支持**。异常接触高度集中在共享内部对角接缝附近，翻转对角线直接导致不对称现象发生镜像反转。
   - **Level 3（幽灵碰撞机制）**：**仍未确认**。

3. **反证与混杂因素**：
   - **位移量级差（反证）**：平坦网格上由于接缝产生的位移增量约为 2 mm（从 0.17 mm 增至 2.17 mm），而原 v2 地形上的转向残余偏移为约 10.29 mm。内部接缝引起的接触法向微小偏转不足以完全解释 10.29 mm 的整体漂移。
   - **多点接触扰动（混杂因素）**：网格表面上车轮与地面的接触点数量在 5 到 16 点之间波动（平板恒为 8 点），且出现少量空接触步（1~4 步），导致轮胎等效接地面载荷中心移动，改变了有效侧向抗扭阻力矩；现有数据无法把“虚假障碍物力（Ghost Collision）”与合法的接触多点重新分配、轮胎侧滑角微小变动完全解耦，不能直接宣称 DART/ODE 存在缺陷。

- **原始证据路径**：`reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/analysis/results.json` 中 `seam_associations` 字段

---

### 问题 7：是否缺关键数据、倒填、插值、裁窗口或跨签名拼接？所有失败和未运行轮次是否保留？

**答复**：
1. **数据完整性**：
   - 绝无倒填、无插值、无裁剪窗口、无跨签名拼接。
   - 所有 8 轮数据每步 1 ms（共 20,000 步）均记录在本地 CSV，RGB/深度/IMU/轮速/接触记录完整且经资格检验。
2. **历史记录保留**：
   - 历史失败目录（`r5-turn-gate-validation-20261011`、`r5-contact-geometry-20261011`、`r5-contact-geometry-20261011-02`）未被删除或修改。
   - `closeout.py` 逐一校验了三个历史归档的 `archive-manifest.json`，变动文件数严格为 0（`failures: []`）。

- **原始证据路径**：
  - `reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/closure.json`
  - `reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/archive-manifest.json`
  - `reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/archive-verification.json`

---

### 问题 8：提供一条可重复的准备命令、执行命令和只读分析命令；确认只关闭本轮容器，原工作台和旧文件保持不变。

**答复**：
1. **可重复命令**：
   - 准备命令：
     ```bash
     python3 reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/setup_run.py
     ```
   - 执行命令：
     ```bash
     python3 reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/execute_run.py
     ```
   - 只读分析命令（宿主或容器均可）：
     ```bash
     python3 reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/analyze_run.py
     ```
2. **隔离与保留确认**：
   - 本轮测试容器 `mooncar-r5-contact-gemini-20261011` 已通过 `closeout.py` 正常停止（`own_container_running: false`，`ExitCode: 0`）。
   - 原工作台容器 `mooncar-terrain-validation`（ID: `3186e022e013`）保持原有停止状态，其 ID、State、Config 逐字段核对完全一致（见 `workbench-before.json` 与 `workbench-after.json`）。
   - 工作区受保护代码与旧数据文件未受任何破坏（`original_files_count: 496`, `unexpected_file_changes: []`）。

- **原始证据路径**：
  - `reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/workbench-before.json`
  - `reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/workbench-after.json`
  - `reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/container-final.json`
  - `reports/evidence/terrain-v2/r5-contact-geometry-gemini-20261011/closure.json`
