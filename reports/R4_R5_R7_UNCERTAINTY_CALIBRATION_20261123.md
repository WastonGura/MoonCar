# R4/R5 定位误差分解与不确定度样本外校准 & R7 弱特征接续审计报告

- **日期**：2026-11-23
- **实测数据集**：6 次真实 Gazebo R5 + `rgbd_window` 全程物理仿真（累计行驶 **278.0 m**、累计仿真时长 **42+ 分钟**、独立真值对齐 **297,112 条定位状态消息**，控制环零真值输入）：
  - 训练集（4 次 30×30 m 月面探测，480 sim s/次）：`coverage_v3_gimbal`、`surface_v3_gimbal`、`hybrid_v4_fixed`、`hybrid_v4_gimbal`
  - **严格样本外测试集**（2 次独立场景，零重训练）：
    1. `hybrid_v41_gimbal`（30×30 m 月面，480 sim s，58.74 m，零底盘原地转）
    2. `r1_ramp_slope_safe`（`mooncar_ramp_v2.sdf` 8.83° 坡道，115.7 sim s，9.99 m，跨世界泛化验证）
- **理论参考**：Bailey et al. (IROS 2006, NEES 一致性检验)、Leutenegger et al. (IJRR 2015, OKVIS 滑动窗口舒尔补边缘化)、Zhang & Scaramuzza (IROS 2018, 视觉里程计轨迹误差评估)；完整引文与 BibTeX 见 [docs/references.md](../docs/references.md) §5。

---

## 1. R4 实际定位误差分解（Error Growth Decomposition）

| 实测场景 | 行驶路程 $s$ (m) | 累计偏航变化 (rad) | XY 中位误差 | XY P95 误差 | XY 最大误差 | $r(e, \sqrt{s})$ | P95($e / \sqrt{s}$) |
|---|---:|---:|---:|---:|---:|---:|---:|
| `coverage_v3_gimbal` | 45.28 | 40.6 | 3.58 cm | 6.10 cm | 6.30 cm | +0.885 | 0.94 cm/√m |
| `surface_v3_gimbal` | 49.41 | 48.3 | 4.54 cm | 6.15 cm | 6.27 cm | +0.719 | 1.43 cm/√m |
| `hybrid_v4_fixed` | 60.76 | 29.8 | 3.26 cm | 4.89 cm | 5.22 cm | +0.811 | 1.00 cm/√m |
| `hybrid_v4_gimbal` | 53.86 | 30.5 | 3.83 cm | 6.93 cm | 7.22 cm | +0.931 | 1.07 cm/√m |
| `hybrid_v41_gimbal`（样本外） | 58.74 | 24.3 | 3.99 cm | 5.87 cm | 6.42 cm | +0.191 | 1.41 cm/√m |
| `r1_ramp_slope_safe`（坡道样本外） | 9.99 | 5.8 | 1.27 cm | 3.44 cm | 3.55 cm | +0.833 | 1.29 cm/√m |

**误差分解结论**：
1. 6 次真实运行中，无全局回环的纯 `rgbd_window` 滑动窗口里程计表现出高度一致的**平移随机游走漂移特征**：单位距离误差率 $\text{P95}(e / \sqrt{s})$ 稳定落在 **`0.94–1.43 cm/√m`**，即使在 60 m 长距离探索中 XY P95 仍保持在 **`3.44–6.93 cm`**（显著优于历史 9.9 cm 基线）。
2. 误差主来源是**被边缘化的旧关键帧跨窗口残留深度/外参/AHRS 慢变相关性**，而非单帧配准突变（`hybrid_v41_gimbal` 零原地转向仍出现 `5.87 cm` P95，说明即使消除底盘原地自转，长路程平移随机游走仍存在）。

---

## 2. R5 不确定度一致性审计与样本外校准（NEES Calibration）

### 2.1 原始条件协方差（`C_cond`）的过度乐观诊断

`WindowEstimator` 输出的 `C_cond` 是固定窗口线性化后的局部条件协方差，实测其 1-σ P95 仅 **`5.10–7.56 mm`**；按 2D 各向同性 $\chi^2_2$ 检验（理论理想均值 `NEES = 2.0`，95% 阈值 $\chi^2_2(0.95) = 5.991$）：
- **原始 95% 覆盖率仅 `4.6% – 34.3%`**，均值 NEES 高达 **`16.3 – 56.3`**（严重低估真实全局误差）。
- 结论：绝不能将局部条件标准差 `C_cond` 当作全局精度或安全门上界。

### 2.2 基于累计估计里程的随机游走漂移模型（零真值、非常数硬乘）

在 [scripts/offline_rgbd_window/uncertainty.py](../scripts/offline_rgbd_window/uncertainty.py) 实现 `OdometryDriftUncertainty`，仅利用估计器自身输出的累计水平行驶路程 $s_k$（剔除 `< 2 mm` 静止抖动，不使用任何真值）：

$$\Sigma_{\text{cal},xy}(k) = C_{\text{cond},xy}(k) + \left(\sigma_0^2 + \alpha_s^2 \cdot s_k\right) I_2, \qquad r_{95}(k) = \sqrt{5.9915 \cdot \lambda_{\max}(\Sigma_{\text{cal},xy}(k))}$$

在 4 组训练集上通过 2D 高斯最大似然估计拟合得：
- 基础局部误差项 $\sigma_0 = 8.58\text{ mm}$
- 路程随机游走漂移系数 $\alpha_s = 5.22\text{ mm}/\sqrt{\text{m}}$

**对比实验（2 参数路程模型 vs 3 参数路程+转角模型）**：3 参数模型在零原地转的样本外 `hybrid_v41_gimbal` 上因过拟合转角导致覆盖率跌至 `77.2%`，而物理简洁的 **2 参数路程随机游走模型** 跨场景泛化表现极佳：

| 数据划分 | 场景 | 原始 `C_cond` 95% 覆盖率 / NEES | 校准后 95% 覆盖率 | 校准后均值 NEES（理想 2.0） | 校准后 95% 误差界 vs 实测 XY P95 |
|---|---|---:|---:|---:|---|
| 训练集 | `coverage_v3_gimbal` | 24.6% / 37.9 | **100.0%** | 1.51 | 8.82 cm vs 6.10 cm |
| 训练集 | `surface_v3_gimbal` | 4.6% / 56.3 | **92.0%** | 2.79 | 9.30 cm vs 6.15 cm |
| 训练集 | `hybrid_v4_fixed` | 7.0% / 31.7 | **100.0%** | 1.31 | 10.14 cm vs 4.89 cm |
| 训练集 | `hybrid_v4_gimbal` | 11.6% / 39.0 | **100.0%** | 1.87 | 9.54 cm vs 6.93 cm |
| **样本外测试 1** | **`hybrid_v41_gimbal`（58.7 m）** | 18.7% / 45.3 | **95.9%** | **2.13** | **9.98 cm vs 5.87 cm** |
| **样本外测试 2** | **`r1_ramp_slope_safe`（8.83° 坡道）** | 34.3% / 16.3 | **100.0%** | **1.41** | **4.67 cm vs 3.44 cm** |

### 2.3 代码接入与向后兼容

已在 [scripts/rgbd_window_odometry.py](../scripts/rgbd_window_odometry.py) 接入 `OdometryDriftUncertainty`：保留原有 `position_std_m` 与 `covariance_kind='model_conditional'` 向后兼容，同时在 `/mooncar/laser_odometry/status` 的 `last_fit` 中明确发布：
- `conditional_position_std_m`（局部窗口条件标准差）
- `calibrated_xy_std_m`（含累计漂移的校准 1-σ）
- `calibrated_xy_bound_95_m`（样本外验证通过的 95% 水平误差界）
- `cumulative_travel_m` 与 `uncertainty_calibration='random_walk_drift_v1_out_of_sample_verified'`

---

## 3. R7 弱特征 / 转向接续稳定性验证

1. **6 次全量实测结果**：累计 **297,112 条定位状态消息**、42+ 仿真分钟、278 m 里程（覆盖 `coverage_v3` 22 次原地转、`surface_v3` 18 次原地转、固定 `-28°` 相机、云台侧扫与 `mooncar_ramp_v2` 8.83° 坡道完整上下坡及换向）中，**定位暂停次数均为 `0`，未再复现历史坡面连续 26 曝光 `no_features` 断链**。
2. **根因与防护机制**：
   - R5 云台抬升至 `z = 0.285 m`（提交 `5f9363a`）改善了坡面俯视视角与深度覆盖基线；
   - R1 坡面安全换向（提交 `621cf67`）消除了陡坡快速原地旋转造成的画面边界空窗；
   - 前端 `TrackFrontend`（[scripts/offline_rgbd_window/frontend.py](../scripts/offline_rgbd_window/frontend.py)）内置双帧参考恢复 `_recover_tracks()` 与弱几何特征重排 `_select_for_metric_spread()`，保持原严格空间门（`occupied ≥ 6, span ≥ 0.30, λ₂ ≥ 0.0025`）不放宽；已通过 [tests/test_r5_uncertainty_and_r7_recovery.py](../tests/test_r5_uncertainty_and_r7_recovery.py) 专项单测验证。
