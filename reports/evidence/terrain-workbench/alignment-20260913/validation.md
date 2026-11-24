# 本轮验证记录

- 数学回归检查通过：时间包围、超限拒绝、跨 ±180° 四元数插值、模型光学轴和高度转换、与误差大小无关的平地内部分类。
- 现场记录成功：154 组同步样本、原始真值流、相机内参和光学点全部保存；选中样本匹配失败数为 0。
- 独立证据检查通过：全部目录哈希、原始时间包围和位置插值、位姿/相机矩阵关系、逐帧点数加权高度统计、正常任务结束及零安全速度输出。
- 普通自主任务的报告和参考诊断本轮均自动生成；三张现场截图内容有变化。
- 图形已查看：轨迹、定位差和高度对比黑白图，保留 PDF 与 PNG。
- 结束时前端入口 http://127.0.0.1:8765/ 返回 HTTP 200；车辆停止。本轮没有进行前端交互回归。
- 工作区保留已有修改。没有安装依赖、修改网络、提交、推送、采购或上传。

复核命令：

```powershell
docker exec -e OPENBLAS_NUM_THREADS=1 mooncar-terrain-validation python3 -m unittest discover -s /source/ros2_ws/src/mooncar_terrain/tests -p test_trajectory_evaluation.py -v
docker exec -e OPENBLAS_NUM_THREADS=1 mooncar-terrain-validation python3 /source/reports/evidence/terrain-workbench/alignment-20260913/check_evidence.py
```

局部同步核验不追认旧记录，不等于整场地或实机精度验收。同步采集入口目前独立于常规任务录制，复现实验需另外启动它。
