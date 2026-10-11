# 给 Gemini：修正最终加载世界，保证对照真实有效

入口实际调用和完整事件输出已有进展。请保持当前实验方向，不启动新仿真，只修下面已离线复现的接线问题。

1. **确保最后加载的世界仍是正确的单变量副本。** Codex执行了实际源码中的三层prepare和runtime观察器添加逻辑：输入X=0.05、观察器各1个，最终却变成X=0、两个ContactObserver、两个StepObserver。具体覆盖点是 `scripts/check_r5_origin_round.py` 的prepare无条件将pose写成 `0 0 .03 ...`；原turn-slip和runtime还会重复追加副本已有的观察器。请在独立测试实现中修正处理顺序或复用方式，最终车辆X正确、每种观察器恰好1个、输出路径唯一。核验应针对真正传给 `gz sim` 的最后文件，并保存其hash和参数差异；别只证明入口拿到了输入文件名。不要改变正式模型或原脚本默认行为。

2. **只解除几何10mm筛选，不解除原数据资格。** 旧第14轮原样可作为超限诊断结果；但人工把 `image_content.passed=false` 或 `origin_qualification.passed=false`，当前包装器仍给 `eligible=true`。保留进入几何筛选前的完整资格结果，只有该结果通过且唯一未通过项确为10mm筛选时才允许继续。子进程超时/不合格退出、评估器不合格同样必须停止；拒绝已有轮次输出目录，禁止读取旧回执作为新尝试结果。

3. **实际组装独立运行资产。** dry-run 把本准备目录挂到/evidence，但这里缺少调用链需要的observer、contact-observer、allocator、turn-slip、turn-models、相关manifest和test-startup。准备新的独立运行目录，复用已验证的同版本资产并核验hash、Jazzy overlay和所有实际路径。不要复用原工作台，不恢复旧验收目录，不能只打印Docker命令就称依赖已备齐。

4. **保护全部输出并使用真实载荷基准。** 事件流现在固定写到脚本旁events目录，用open('w')，在CLI拒覆盖JSON之前已写入；将每轮所有分析文件写到其独立新输出目录，事先拒绝覆盖。载荷重分配基准从本轮运动前的稳定窗口计算，不使用无来源的 `[4.34,2.80,3.10,4.64] N`。保留原始力与事件记录，不改物理数值。

此次验证应直接走实际准备函数，截停在启动Gazebo之前，证明最终加载X及插件数量；替身不能直接返回你手填的 `eligible=true` 绕开单轮实现。资格测试基于旧真实结果逐项注入失败，进程与输出保护用临时目录验证。仿真仍未授权，不增加轮数。完成后给代码、最终世界/依赖核对及检查记录即可。

Codex复现文件：`D:\Documents\Projects\Repositories\MoonCar-mapping\reports\evidence\terrain-v2\r5-v2-offset-chain-review-20261011\review-results.json`。
