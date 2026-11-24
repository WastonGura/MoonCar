"""Offline report and chart from a completed read-only mass audit."""
import argparse
import csv
import json
from pathlib import Path


def report(result, catalog, destination):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    result, catalog, destination = Path(result), Path(catalog), Path(destination)
    a = json.loads(result.read_text(encoding='utf-8'))
    c = json.loads(catalog.read_text(encoding='utf-8'))
    if not a['passed']:
        raise ValueError('Cannot render a passing report for failed calculation')
    parts = a['parts']
    with (destination/'measurement-template.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.writer(f)
        w.writerow(['id','component','quantity','onboard_or_external','exact_model_revision',
                    'included_in_other_component','measured_mass_kg','mass_uncertainty_kg',
                    'mount_x_m','mount_y_m','mount_z_m','mount_rpy_rad','local_com_xyz_m',
                    'measurement_frame','method','repeat_values','date','source_file_sha256'])
        for e in a['procurement']:
            w.writerow([e['id'],e['name'],e['quantity'],*['']*15])
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), constrained_layout=True)
    labels = [p['link'].replace('_link', '') for p in parts]
    y = np.arange(len(parts))
    axes[0].barh(y, [p['mass_kg'] for p in parts], color='#637d99')
    values = [p['com_contribution_m'][0]*1000 for p in parts]
    axes[1].barh(y, values, color=['#b96750' if v < 0 else '#4d8c7e' for v in values])
    axes[0].set(yticks=y, yticklabels=labels, xlabel='Modeled mass (kg)', title='Current simplified components')
    axes[1].set(yticks=y, yticklabels=labels, xlabel='Contribution to whole COM X (mm)', title=f"Sum = {a['urdf']['com_m'][0]*1000:.6f} mm")
    for ax in axes:
        ax.invert_yaxis(); ax.grid(axis='x', alpha=.2); ax.set_axisbelow(True)
    axes[1].axvline(0, color='black', linewidth=.6)
    fig.suptitle('R5 static mass audit: zero gimbal, base_footprint frame\nModel consistency only; no hardware mass validation', fontsize=13)
    fig.savefig(destination/'mass-contributions.png', dpi=180)
    plt.close(fig)

    total = a['urdf']
    lines = ['# R5 质量分布与质心审计', '',
             '**结论：模型自洽，真实质量分布未确认。** 本轮没有新增 Gazebo 仿真，没有修改车辆、质心、控制、里程计或 DART。约 31 cm 转向偏移没有在本轮修复。', '',
             '## 计算结果', '',
             f"当前源码重新展开的 URDF 与转换后的 SDF 均为 **{total['total_mass_kg']:.3f} kg**。云台和车轮关节归零，统一在 `base_footprint` 中计算整车质心：", '',
             f"`({total['com_m'][0]:.12f}, {total['com_m'][1]:.12f}, {total['com_m'][2]:.12f}) m`。", '',
             '整车质心由 `Σ(m_i × r_i) / Σm_i` 计算；每个部件的惯量先旋转到统一坐标，再用平行轴公式相加。URDF 独立求和与 SDF 固定关节合并结果交叉核对。', '',
             f"质量差 {a['crosscheck']['absolute_delta']['mass_kg']:.3g} kg；质心最大轴差 {max(a['crosscheck']['absolute_delta']['com_m']):.3g} m；惯量最大项差 {np.max(a['crosscheck']['absolute_delta']['inertia_kg_m2']):.3g} kg·m²。分别低于 1e-6 kg、1e-6 m、1e-8 kg·m²。未发现该计算链路中的单位、坐标或固定关节合并错误。", '',
             '合并后的 SDF `base_footprint` 质量为 7.425 kg，含多个原始固定连杆；不能把它再与原 URDF 底盘、电池等相加，否则会重复计重。', '',
             '![部件质量与质心贡献](mass-contributions.png)', '',
             '电池对质心 X 的贡献约 −11.621 mm；雷达、云台及相机的向前贡献抵消一部分，四轮前后贡献在当前对称假设下互相抵消，最终为 −6.238 mm。贡献取决于当前原点和假设参数，不是实测分布。', '',
             '## 逐部件依据', '',
             '| 部件 | 质量 kg | 质量依据 | 局部质心 / 安装 / 惯量依据 | 质心位置（车体坐标）m | X贡献 mm | 来源 |',
             '|---|---:|---|---|---|---:|---|']
    for p in parts:
        prov = p['provenance']
        lines.append(f"| {p['link']} | {p['mass_kg']:.3f} | {prov['mass']['classification']} | {' / '.join(prov[k]['classification'] for k in ('local_com','mount','inertia'))} | {tuple(round(x,6) for x in p['com_m'])} | {p['com_contribution_m'][0]*1000:.6f} | {', '.join(prov['mass']['sources'])} |")
    lines += ['', '所有零件的局部质心及惯量目前均采用均匀盒体/圆柱近似；安装位置均缺实物装配确认。雷达和相机的厂家外形不证明内部质量均匀。', '',
              '- 裸底盘 5.7 kg 在采购文档中以条件估计出现；未找到对应四驱版本、说明重量包含范围的可靠厂家原表。模型把它拆成车体 4.1 kg、四轮各 0.4 kg，没有分件称重依据。电机、减速箱、编码器在模型中被整体抽象，具体质量归属仍未闭合。',
              '- P7120S 3.05 kg 有归档原手册第 12 页支持；局部质心放在均匀盒体中心、装车 joint 位置 `(-0.035,0,0.088) m` 均为假设。这个 joint 的坐标系是 `base_link`，换到 `base_footprint` 后电池质心 Z 为 0.188 m，不是 0.088 m。',
              '- S3 的 115 g 来自 2026-10-01 官方页面快照，当前网页提取未再次提供该项。OAK 61 g 来自此前官方店铺核验转录，原始店铺抓取失败，可信度弱于原始原表；当前页面提取未重新确认质量。',
              '- 采购记录 SKU 21545 写“该销售版本无 IMU”，当前官方 OAK 页面写 BMI270。版本差异尚未闭合，不能把 61 g 当作已称重的采购版本，也不能因此取消独立 SEN0374。',
              '- 云台结构 0.08/0.06/0.04 kg 共 0.18 kg、IMU 0.08 kg 均为工程值；采购表没有云台执行器。', '',
              '## 采购与遗漏清单', '',
              '以下覆盖当前采购 01–20 项，套装中的车载件与外置充电器、桌面电源、调试器分开说明。缺失质量保持空值，不作为零；“独立建模”也不代表该套装所有附件已计入。', '',
              '| 编号 | 部件 | 模型状态 | 核对结果 |', '|---|---|---|---|']
    for e in a['procurement']:
        lines.append(f"| {e['id']} | {e['name']} | {e['model_status']} | {e['detail']} |")
    lines += ['', '四个 TFmini 的名义模块质量共 48 g，尚未入模；安装位置和支架/线束质量缺失。其余遗漏器件总质量未知，不能据此算出完整实车总质量或质心误差范围。C50C、D50A 是否包含于所谓裸底盘重量不明，既不能默认已计入，也不能盲目另加导致重复。', '',
              '## 参数敏感度与测量优先级', '',
              '`result/components.csv` 和 `result/audit.json` 保存每个部件的三轴结果。质量敏感度为 `(r_i−C)/M`，按增加 1 g 报告；平移敏感度为 `m_i/M`，按该部件单独沿车体坐标轴移动 1 mm 报告，不联动子部件，不改变模型。它们是局部导数，不是有限改动实验，也不是物理误差界限。', '',
              '以当前电池为例，单独向前移动 1 mm 会使整车质心 X 移动约 0.332 mm；增加 1 g 电池质量对应质心 X 导数约 −0.00313 mm。没有执行这些变更。未知部件无法按实际质量排序，以下优先级按现有影响和缺失范围制定。', '']
    for p in a['measurement_priorities']:
        lines += [f"{p['priority']}. **{p['item']}**：{p['method']} {p['reason']}"]
    lines += ['', '## 来源、复现与判定限制', '',
              '新容器只做 xacro、SDF 转换、静态审计与软件检查；源目录及同级硬件资料只读挂载。`models/` 是当前源码新展开结果，`catalog-ready/` 为本轮来源快照及哈希，`result/` 保存完整数值、逐部件位姿链、来源分类和敏感度。没有用旧模型替代本轮展开。', '',
              '独立入口：`audit_r5_mass.py --urdf ... --sdf ... --catalog ... --output 新目录`；输出目录已存在时拒绝覆盖，来源快照必须通过哈希核验。`prepare_r5_mass_audit.sh` 在独立 Jazzy 容器生成模型和来源；报告脚本只读审计结果。需要同级 MoonCar 采购资料及冻结工作树，单个本轮提交不包含以前的其他未提交依赖。', '',
              '| 来源 ID | 日期 / 版本 | 定位 / 限制 | SHA-256 |', '|---|---|---|---|']
    for name, source in c['sources'].items():
        lines.append(f"| {name} | {source.get('document_date') or '未注明'} / {source['version']} | {source['locator']} | `{source['sha256']}` |")
    lines += ['', '来源冲突与不同方案保留在 `result/audit.json`，不平均：旧 8 kg 整备目标不是新实测值；历史分买电机路线不计入当前 R5；R550 资料与普通京东首页不适用于所选 R5，已排除。', '',
              '当前结果只证明简化模型的质量计算一致。没有完整实物数据，不能判定实际质心准确，也不能把“质心影响支点”改写成“质心设置错误”。未生成修复候选，未消除 31 cm 偏移，未判定转向或整车通过。', '',
              '`measurement-template.csv` 提供后续装配、称重和坐标记录空表；所有未测字段保持空白。软件检查、原文件保留及容器结束记录见 `completion.json`；工作日志保存本轮结论和后续测量任务。']
    (destination/'README.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('result','catalog','destination'):p.add_argument('--'+k,required=True)
    args=p.parse_args();report(args.result,args.catalog,args.destination)
