# 验证记录

2026-09-12，复用 mooncar-terrain-validation 中已有 Python、NumPy、Matplotlib 与 ROS 环境；没有安装依赖。

- `test_terrain_reference.py`：1 项通过。覆盖倾斜盒体、旋转球/椭球、视觉圆柱、边缘分类、缺失分母和不支持几何拒绝。
- `test_survey_evaluation.py`：3 项通过。包含真实命令行后处理入口的合成实验：报告自动生成独立参考分析，原始精度验收字段仍为空；最近运行 37.423 秒。
- 本目录 `check_evidence.py`：通过。复核三组输出哈希、原输入哈希、固定区域格数、分类分母、从逐格数据重新计算的误差、Q 分箱和停车点坐标差。
- `continuous-20260912/check_evidence.py`：再次通过，原三组连续运行与故障测试证据保持完整。
- 已查看高度/坡度六图、Q 误差与数量图、覆盖状态图；黑白配色，坐标及单位可读。
- 现场只读检查结束后：experiment-23b486559f42，采集服务 recording，运动 active=false，导航 ready=true。本次未控制车辆移动。

这不是多位置或全轨迹对齐验收，也不是硬件精度测试。
